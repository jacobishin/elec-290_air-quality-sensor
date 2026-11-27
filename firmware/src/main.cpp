/*
ELEC 290 Section 002-01 - Harmful Gas Monitor Firmware

Hardware (aligned with ELEC290 Pinout.pdf + MQ-7 Heater Circuit):
ESP32-WROVER-E
SGP30 (I2C)    - eCO2 and TVOC sensor (GPIO 21/22)
MQ-7 (ADC1)    - CO sensor (AO -> GPIO 36; S8050 PWM -> GPIO 14
DHT22          - Temp and humidity sensor (GPIO 13)
PMS7003 (UART) - PM2.5 sensor (PMS RX -> GPIO 33, PMS TX -> GPIO 19)
Passive Buzzer - Audio alert (GPIO 25, LEDC Ch 4)
2.0" ST7789    - SPI LCD (GPIO 23/18/5/27/26) + Joystick (GPIO 34/35/32)
*/

#include <Arduino.h>
#include <math.h>

#include "local_communication.h" // Display, Joystick, and Buzzer module
#include "sensors.h"             // MQ-7 (direct/heater-cycle), SGP30, PMS7003, DHT22
#include "storage.h"        // LittleFS ping-pong CSV & Core 0 Firebase sync
#include "tests.h"               // Serial verification test suite


// ============================================================
// CONFIGURATION
// ============================================================

/*
 * BUILT-IN SERIAL MONITOR COMMANDS (115200 baud, single-char, case-insensitive)
 * Set ENABLE_VERIFICATION_TESTS to 1 below, open Serial Monitor, and send:
 *   Storage & Diagnostics:
 *     d - Dump the entire active LittleFS CSV log file
 *     r - Reset/wipe both CSV log files and NVS upload bookmark back to File 1
 *     s - Print storage file size, unsent bytes, Wi-Fi, and NTP sync status
 *     p - Print the latest logged window averages, alert status, and fault counts
 *     f - Inject a simulated Storage fault
 *     c - Clear all active fault flags/counters and reset any active danger alert
 *   Verification & Simulation Tests (RTM R1, R3, R4, R6, R7, R8):
 *     1 - Inject valid mock readings for all 6 sensors to LCD & loggedData
 *     2 - Inject invalid readings (NaN CO, negative PM2.5) + INVALID fault
 *     3 - Toggle simulated emergency Danger Alert (flashing screen + buzzer)
 *     4 - Toggle forced Wi-Fi disconnect / reconnect (offline buffering test)
 *     5 - Toggle 60-second uptime/storage heartbeat log for endurance testing
 * Set ENABLE_VERIFICATION_TESTS to 0 when done
 */

#define ENABLE_VERIFICATION_TESTS 0

// Wi-Fi & Firebase Realtime Database credentials
const char* WIFI_SSID           = "jacob";
const char* WIFI_PASSWORD       = "341145100";
const char* FIREBASE_URL        = "https://gas-monitor-6e3eb-default-rtdb.firebaseio.com/devices/testmonitor/readings.json";
const char* FIREBASE_AUTH_TOKEN = "1YfvkBGVE4v2KK5FFd7N1QkqjXl5opGkmFKVwLUd";

// Pin assignments (single source of truth for all modules)
extern const int I2C_SDA_SGP30_PIN  = 21; // SGP30 SDA         -> GPIO 21
extern const int I2C_SCL_SGP30_PIN  = 22; // SGP30 SCL         -> GPIO 22

extern const int I2C_SDA_SCREEN_PIN = 23; // ST7789 MOSI (SDA) -> GPIO 23
extern const int I2C_SCL_SCREEN_PIN = 18; // ST7789 SCK  (SCL) -> GPIO 18
extern const int SCREEN_CS_PIN      = 5;  // ST7789 CS         -> GPIO 5
extern const int SCREEN_DC_PIN      = 27; // ST7789 DC         -> GPIO 27
extern const int SCREEN_RST_PIN     = 26; // ST7789 RST        -> GPIO 26

extern const int JOY_X_PIN          = 34; // Joystick VRx      -> GPIO 34
extern const int JOY_Y_PIN          = 35; // Joystick VRy      -> GPIO 35
extern const int JOY_SW_PIN         = 32; // Joystick SW       -> GPIO 32

extern const int PMS_TX_PIN         = 33; // ESP32 TX -> PMS7003 RX (GPIO 33)
extern const int PMS_RX_PIN         = 19; // ESP32 RX <- PMS7003 TX (GPIO 19)

extern const int MQ7_PIN            = 36; // MQ-7 AO           -> GPIO 36
extern const int DHT_PIN            = 13; // DHT22 OUT         -> GPIO 13

extern const int BUZZER_PIN         = 25; // Passive Buzzer    -> GPIO 25 (LEDC Ch 4)

// System intervals (ms)
const unsigned long ALERT_CHECK_INTERVAL     = 1000UL;   // 1s fast danger check (RTM R3 <= 2s response)
const unsigned long DISPLAY_INTERVAL         = 5000UL;   // 5s LCD & Wi-Fi refresh window
const unsigned long LOG_INTERVAL             = 15000UL; // 5-minute CSV log & Firebase window
extern const unsigned long TRANSMIT_INTERVAL = 2000UL;   // 2s background Firebase upload check

// Air quality thresholds (CO and eCO2 only; -1.0f = unconfigured placeholder)
extern const float C0_THRESHOLD1_PPM   = -1.0f;
extern const float ECO2_THRESHOLD1_PPM = 1000.0f;

extern const float C0_THRESHOLD2_PPM   = 200.0f;
extern const float ECO2_THRESHOLD2_PPM = 5000.0f;

const int allowedSensorFaults    = 1;
const int allowedInvalidReadings = 1;
const int allowedStaleReadings   = 1;
const int allowedStorageFaults   = 1;
const int allowedWifiFaults      = 1;


// ============================================================
// GLOBAL SYSTEM STATE
// ============================================================

ProcessedData displayData = {}; // Finalized 5-second data (LCD & Buzzer)
ProcessedData loggedData  = {}; // Finalized 5-minute data (CSV & Firebase)
FaultStatus faults        = {};

bool dangerAlertActive    = false;
String activeAlertMessage = "Normal";

static unsigned long lastAlertCheck   = 0;
static unsigned long lastDisplayAndUI = 0;
static unsigned long lastWindowClose  = 0;


// ============================================================
// DATA PROCESSING & FAULT DETECTION
// ============================================================

static void finalizeChannel(const SensorTotal &src, AverageData &dst) {
  if (src.count > 0) {
    dst.average     = src.sum / static_cast<float>(src.count);
    dst.maximum     = src.maximum;
    dst.sampleCount = src.count;
    dst.valid       = isfinite(dst.average);
  } else {
    dst.average     = NAN;
    dst.maximum     = NAN;
    dst.sampleCount = 0;
    dst.valid       = false;
  }
}

// In heater-cycle mode, holds the latest valid CO reading across windows between 150s cycles
static void finalizeCOChannel(const SensorTotal &src, AverageData &dst, unsigned long now) {
  if (src.count > 0) {
    finalizeChannel(src, dst);
    return;
  }

#if MQ7_USE_HEATER_CIRCUIT
  if (rawData.coPPM.valid && isfinite(rawData.coPPM.value) &&
      (now - rawData.coPPM.timestamp <= MQ7_STALE_TIMEOUT_MS)) {
    dst.average     = rawData.coPPM.value;
    dst.maximum     = rawData.coPPM.value;
    dst.sampleCount = 1;
    dst.valid       = true;
    return;
  }
#endif

  dst.average     = NAN;
  dst.maximum     = NAN;
  dst.sampleCount = 0;
  dst.valid       = false;
}

static void finalizeWindow(SensorAccumulators &acc, ProcessedData &out) {
  unsigned long now = millis();
  out.windowEndTimestamp = now;

  finalizeCOChannel(acc.coPPM,          out.coPPM, now);
  finalizeChannel(acc.tvocPPB,          out.tvocPPB);
  finalizeChannel(acc.eco2PPM,          out.eco2PPM);
  finalizeChannel(acc.temperatureC,     out.temperatureC);
  finalizeChannel(acc.humidityPercent,  out.humidityPercent);
  finalizeChannel(acc.pm25UGM3,         out.pm25UGM3);
  clearAccumulators(acc);
}

static void updateFaultCounter(bool &status, int &num, int allowed, bool override) {
  if (override) {
    num    = 0;
    status = false;
  } else {
    num++;
    if (num >= allowed) status = true;
  }
}

int faultDetections(faultenum fault, bool override) {
  switch (fault) {
    case SENSOR:  updateFaultCounter(faults.sensor.status,         faults.sensor.num,         allowedSensorFaults,    override); break;
    case INVALID: updateFaultCounter(faults.readingInvalid.status, faults.readingInvalid.num, allowedInvalidReadings, override); break;
    case STALE:   updateFaultCounter(faults.readingStale.status,   faults.readingStale.num,   allowedStaleReadings,   override); break;
    case STORAGE: updateFaultCounter(faults.storage.status,        faults.storage.num,        allowedStorageFaults,   override); break;
    case WIFI:    updateFaultCounter(faults.wifi.status,           faults.wifi.num,           allowedWifiFaults,      override); break;
    default: break;
  }
  return 0;
}

// Auto-clear transient INVALID / STALE faults once all active (non-warmup) sensors report healthy
static void checkTransientFaultRecovery(const ProcessedData &data) {
  bool coReady  = data.coPPM.valid || isCOInWarmup();
  bool sgpReady = (data.tvocPPB.valid && data.eco2PPM.valid) || isSGP30InWarmup();
  bool thReady  = data.temperatureC.valid && data.humidityPercent.valid;
  bool pmReady  = data.pm25UGM3.valid;

  if (coReady && sgpReady && thReady && pmReady) {
    if (faults.readingInvalid.status) faultDetections(INVALID, true);
    if (faults.readingStale.status)   faultDetections(STALE, true);
  }
}


// ============================================================
// DECISION LOGIC
// ============================================================

static void evaluateAirQualityLive() {
  bool danger     = false;
  const char* msg = "Normal";

  if (C0_THRESHOLD2_PPM > 0.0f && rawData.coPPM.valid &&
      isfinite(rawData.coPPM.value) && rawData.coPPM.value >= C0_THRESHOLD2_PPM) {
    danger = true;
    msg    = "HIGH CO";
  } else if (ECO2_THRESHOLD2_PPM > 0.0f && rawData.eco2PPM.valid &&
             isfinite(rawData.eco2PPM.value) && rawData.eco2PPM.value >= ECO2_THRESHOLD2_PPM) {
    danger = true;
    msg    = "HIGH eCO2";
  }

  dangerAlertActive  = danger;
  activeAlertMessage = msg;
}


// ============================================================
// SETUP & MAIN LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing system...");

  initBuzzerHardware();
  initializeSensors();
  initDisplayAndJoystickHardware();
  initializeStorage();
  initializeCommunication();

  checkWiFiConnection();
  renderDisplay(displayData);
  Serial.println("Harmful Gas Monitor Started.\n");
}

void loop() {
  unsigned long currentTime = millis();

  // --- Section 2: Sensor Acquisition ---
  readSensors(currentTime);

  // --- Section 4: 1-second Fast Danger Evaluation (RTM R3 <= 2s latency) ---
  if (currentTime - lastAlertCheck >= ALERT_CHECK_INTERVAL) {
    lastAlertCheck = currentTime;
    evaluateAirQualityLive();
  }

  // --- Section 3 & 6: 5-second Window Processing, Wi-Fi Check & LCD Refresh ---
  if (currentTime - lastDisplayAndUI >= DISPLAY_INTERVAL) {
    lastDisplayAndUI = currentTime;
    finalizeWindow(safetyAccumulators, displayData);
    checkTransientFaultRecovery(displayData);
    checkWiFiConnection();
    renderDisplay(displayData);
  }

  // --- Section 3 & 5: 5-minute Window Processing & LittleFS CSV Logging ---
  if (currentTime - lastWindowClose >= LOG_INTERVAL) {
    lastWindowClose = currentTime;
    finalizeWindow(accumulators, loggedData);
    storeData(loggedData);
  }

  // --- Section 4 & 6: Continuous Buzzer, Joystick/Alert UI & Core 0 Firebase Sync ---
  refreshBuzzerState(currentTime);
  handleJoystickInput(currentTime, displayData);
  pollFirebaseSync(currentTime);

#if ENABLE_VERIFICATION_TESTS
  runTestCommands();
#endif

  // Yield Core 1 briefly (200 Hz loop) to reduce idle power draw on battery
  delay(5);
}
