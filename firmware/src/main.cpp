/* 
ELEC 290 Section 002-01 - Harmful Gas Monitor Firmware

Hardware (aligned with ELEC290 Pinout.pdf + MQ-7 Heater Circuit):
ESP32-WROVER-E
SGP30 (I2C)    - eCO2 and TVOC sensor (GPIO 21/22)
MQ-7 (ADC1)    - CO sensor (AO via 10k/20k divider -> GPIO 36; S8050 PWM -> GPIO 14)
DHT22          - Temp and humidity sensor (GPIO 13)
PMS7003 (UART) - PM2.5 sensor (GPIO 33/19)
Passive Buzzer - Audio alert (GPIO 25, LEDC Ch 4)
2.0" ST7789    - SPI LCD (GPIO 23/18/5/27/26) + Joystick (GPIO 34/35/32)
*/

#include <Arduino.h>
#include <math.h>
#include <time.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <LittleFS.h>
#include <Preferences.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SGP30.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <PMS7003.h>

#include "local_communication.h" // Display, Joystick, and Buzzer module
#include "tests.h"               // Serial verification test suite


// ============================================================
/* CONFIGURATION */
// ============================================================

// 1 = enable Serial Monitor verification test suite, 0 = disable for final release
#define ENABLE_VERIFICATION_TESTS 0

// Comment out to disable 150s heater cycling and run constant 5V polling (1s interval)
#define USE_MQ7_HEATER_CYCLE

// Wi-Fi & Firebase Realtime Database credentials
const char* WIFI_SSID           = "jacob";
const char* WIFI_PASSWORD       = "341145100";
const char* FIREBASE_URL        = "https://gas-monitor-6e3eb-default-rtdb.firebaseio.com/devices/testmonitor/readings.json";
const char* FIREBASE_AUTH_TOKEN = "1YfvkBGVE4v2KK5FFd7N1QkqjXl5opGkmFKVwLUd";

// Pin assignments (aligned with ELEC290 Pinout.pdf — ESP32-WROVER-E)
const int I2C_SDA_SGP30_PIN         = 21; // SGP30/CCS811 SDA -> GPIO 21
const int I2C_SCL_SGP30_PIN         = 22; // SGP30/CCS811 SCL -> GPIO 22

extern const int I2C_SDA_SCREEN_PIN = 23; // ST7789 MOSI (SDA) -> GPIO 23
extern const int I2C_SCL_SCREEN_PIN = 18; // ST7789 SCK  (SCL) -> GPIO 18
extern const int SCREEN_CS_PIN      = 5;  // ST7789 CS         -> GPIO 5
extern const int SCREEN_DC_PIN      = 27; // ST7789 DC         -> GPIO 27
extern const int SCREEN_RST_PIN     = 26; // ST7789 RST        -> GPIO 26

extern const int JOY_X_PIN          = 34; // Joystick VRx -> GPIO 34
extern const int JOY_Y_PIN          = 35; // Joystick VRy -> GPIO 35
extern const int JOY_SW_PIN         = 32; // Joystick SW  -> GPIO 32

const int PMS_RX_PIN                = 33; // PMS7003 UART2 RX -> GPIO 33
const int PMS_TX_PIN                = 19; // PMS7003 UART2 TX -> GPIO 19

const int MQ7_PIN                   = 36; // MQ-7 AO (via 10k/20k divider) -> GPIO 36
const int DHT_PIN                   = 13; // DHT22 OUT -> GPIO 13

extern const int BUZZER_PIN         = 25; // Passive Buzzer -> GPIO 25 (LEDC Ch 4)

// MQ-7 calibration & divider constants
const float MQ7_DIVIDER_SCALE = 1.5f;     // Undoes 10 kOhm / 20 kOhm divider (set to 1.0f if no divider)
const float MQ7_RL_OHMS       = 10000.0f; // Onboard 10 kOhm load resistor on 4-pin module
const float MQ7_R0_OHMS       = 10000.0f; // TODO: Update with measured clean-air R0 calibration value

// Sensor sampling intervals (ms)
const unsigned long MQ7_SAMPLE_INTERVAL     = 1000;
const unsigned long SGP30_SAMPLE_INTERVAL   = 1000;
const unsigned long TEMPHUM_SAMPLE_INTERVAL = 2000; // DHT22 requires >= 2000 ms between reads
const unsigned long PM25_SAMPLE_INTERVAL    = 1000;

// Stale reading multiplier & MQ-7 cycle timeout
const unsigned long STALE_TIMEOUT_MULTIPLIER = 3;
#ifdef USE_MQ7_HEATER_CYCLE
const unsigned long MQ7_CYCLE_MS         = 150000UL; // 60s purge + 90s sense
const unsigned long MQ7_STALE_TIMEOUT_MS = 160000UL; // > 155s to avoid false stale faults
#else
const unsigned long MQ7_STALE_TIMEOUT_MS = MQ7_SAMPLE_INTERVAL * STALE_TIMEOUT_MULTIPLIER;
#endif

// Data logging & communication intervals (ms)
const unsigned long LOG_INTERVAL      = 60000; // 60s CSV log window
const unsigned long DISPLAY_INTERVAL  = 5000;  // 5s display/alert/Wi-Fi window
const unsigned long TRANSMIT_INTERVAL = 2000;  // 2s background Firebase upload interval

// File system constants (64 KB per file fits safely inside 192 KB min_spiffs.csv LittleFS)
const char* LOG_FILE_1_PATH    = "/sensor_log_1.csv";
const char* LOG_FILE_2_PATH    = "/sensor_log_2.csv";
const size_t MAX_LOG_FILE_SIZE = 65536;

const char* CSV_HEADER = "timestamp_ms,alert_active,"
                         "co_avg,co_count,co_valid,"
                         "tvoc_avg,tvoc_count,tvoc_valid,"
                         "eco2_avg,eco2_count,eco2_valid,"
                         "temp_avg,temp_count,temp_valid,"
                         "hum_avg,hum_count,hum_valid,"
                         "pm25_avg,pm25_count,pm25_valid";

// Air quality thresholds (-1.0f = unconfigured placeholder)
extern const float C0_THRESHOLD1_PPM    = -1.0f;
extern const float TVOC_THRESHOLD1_PPB  = -1.0f;
extern const float ECO2_THRESHOLD1_PPM  = -1.0f;
extern const float PM25_THRESHOLD1_UGM3 = -1.0f;

extern const float C0_THRESHOLD2_PPM    = 200.0f;
extern const float TVOC_THRESHOLD2_PPB  = -1.0f;
extern const float ECO2_THRESHOLD2_PPM  = 4000.0f;
extern const float PM25_THRESHOLD2_UGM3 = -1.0f;

const int allowedSensorFaults    = 1;
const int allowedInvalidReadings = 1;
const int allowedStaleReadings   = 1;
const int allowedStorageFaults   = 1;
const int allowedWifiFaults      = 1;


// ============================================================
/* DATA STRUCTURES */
// ============================================================

struct SensorReading {
  float value = NAN;
  bool valid = false;
  unsigned long timestamp = 0;
};

struct MQ7Data          { SensorReading coPPM; };
struct SGP30Data        { SensorReading tvocPPB; SensorReading eco2PPM; };
struct TempHumidityData { SensorReading temperatureC; SensorReading humidityPercent; };
struct PM25Data         { SensorReading concentration; };

struct RawSensorData {
  MQ7Data mq7;
  SGP30Data sgp30;
  TempHumidityData tempHum;
  PM25Data pm25;
};

struct SensorTotal {
  float sum = 0.0f;
  float maximum = -INFINITY;
  float minimum = INFINITY;
  unsigned int count = 0;
};

struct SensorAccumulators {
  SensorTotal coPPM;
  SensorTotal tvocPPB;
  SensorTotal eco2PPM;
  SensorTotal temperatureC;
  SensorTotal humidityPercent;
  SensorTotal pm25UGM3;
};

// FreeRTOS IPC payloads (Core 1 owns LittleFS/NVS, Core 0 owns HTTPS)
struct UploadJob {
  char json[512];
  size_t nextOffset;
  uint8_t fileIndex;
};

struct UploadResult {
  bool success;
  size_t nextOffset;
  uint8_t fileIndex;
};


// ============================================================
/* GLOBAL VARIABLES */
// ============================================================

RawSensorData rawData = {};

SensorAccumulators safetyAccumulators = {}; // Running 5-second accumulators
ProcessedData displayData = {};             // Finalized 5-second data (LCD & Buzzer)

SensorAccumulators accumulators = {};       // Running 60-second accumulators
ProcessedData loggedData = {};              // Finalized 60-second data (CSV & Firebase)

FaultStatus faults = {};

bool wifiConnected        = false;
bool dangerAlertActive    = false;
String activeAlertMessage = "Normal";

// Hardware instances
Adafruit_SGP30 sgp;
DHT dht(DHT_PIN, DHT22);
GuL::PMS7003 pms(Serial2);
bool sgp30Initialized = false;

// LittleFS & NVS sync state
Preferences syncPrefs;
uint8_t activeLogFileIndex    = 1;
size_t lastUploadedByteOffset = 0;
size_t currentLogFileSize     = 0;
bool timeSyncedThisBoot       = false;
unsigned long bootEpochSec    = 0;

// FreeRTOS Core 0 Firebase worker handles
static QueueHandle_t firebaseUploadQueue = nullptr;
static QueueHandle_t firebaseResultQueue = nullptr;
static bool uploadInFlight               = false;

// Non-blocking timers
unsigned long lastMQ7Read      = 0;
unsigned long lastSGP30Read    = 0;
unsigned long lastTempHumRead  = 0;
unsigned long lastPM25Read     = 0;
unsigned long lastDisplayAndUI = 0;
unsigned long lastWindowClose  = 0;
unsigned long lastTransmission = 0;


// MQ7 Heater cycle
#ifdef USE_MQ7_HEATER_CYCLE
  const int MQ7_HEATER_PIN = 14; // 470 Ohm -> S8050 Middle Pin (Base)
  const int MQ7_PWM_CH     = 0;  // Safe: buzzer uses LEDC Channel 4

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    static void mq7HeaterPwmBegin()          { ledcAttachChannel(MQ7_HEATER_PIN, 1000, 8, MQ7_PWM_CH); }
    static void mq7HeaterPwmWrite(uint8_t d) { ledcWrite(MQ7_HEATER_PIN, d); }
  #else
    static void mq7HeaterPwmBegin() {
      ledcSetup(MQ7_PWM_CH, 1000, 8);
      ledcAttachPin(MQ7_HEATER_PIN, MQ7_PWM_CH);
    }
    static void mq7HeaterPwmWrite(uint8_t d) { ledcWrite(MQ7_PWM_CH, d); }
  #endif

  // Manages 150s cycle (60s @ 5V purge, 89.5s @ 1.4V RMS sense).
  // Returns TRUE once at 149.5s with solid GND for a clean GPIO 36 read.
  bool mq7ShouldSampleNow(unsigned long now) {
    static bool initialized      = false;
    static bool sampledThisCycle = false;
    static int  lastDuty         = -1;

    if (!initialized) {
      mq7HeaterPwmBegin();
      initialized = true;
    }

    unsigned long cyclePos = now % MQ7_CYCLE_MS;

    if (cyclePos < 60000UL) {
      if (lastDuty != 255) { mq7HeaterPwmWrite(255); lastDuty = 255; } // 5.0V Purge (100% duty)
      sampledThisCycle = false;
      return false;
    } else if (cyclePos < 149500UL) {
      if (lastDuty != 22)  { mq7HeaterPwmWrite(22);  lastDuty = 22;  } // 1.4V RMS Sense (8.7% duty)
      return false;
    } else if (!sampledThisCycle) {
      mq7HeaterPwmWrite(255); // Solid GND for 5 ms so AO settles without PWM chopping
      lastDuty = 255;
      delay(5);
      sampledThisCycle = true;
      return true;
    }
    return false;
  }
#endif


// ============================================================
/* FUNCTION DECLARATIONS */
// ============================================================

// 1. Initialization
void initializeSystem();
void initializeSensors();
void initializeDisplay();
void initializeStorage();
void initializeCommunication();
void initializeAlerts();

// 2. Data Acquisition
void readSensors(unsigned long currentTime);
bool readmq7();
bool readsgp30();
bool readpm25();
bool readhum();
bool readtemp();
void clearSafetyAccumulators();
void clearAccumulators();

// 3. Data Processing & Fault Detection
int processData(bool option);
int faultDetections(faultenum fault, bool override);

// 4. Decision Logic
void evaluateAirQuality(const ProcessedData &data);
void updateAlerts();

// 5. Data Storage
const char* getLogFilePath(bool active = true);
bool initNewLogFile(uint8_t fileIndex);
void storeData(const ProcessedData &data);

// 6. Communication
bool csvLineToJson(const String &csvLine, char* jsonOut, size_t jsonOutSize);
bool sendJsonToFirebase(const char* jsonPayload);
void firebaseUploadTask(void* param);
void updateCommunication(unsigned long currentTime);
void updateDisplay(const ProcessedData &data);
void transmitData();
void checkWiFiConnection();


// ============================================================
/* SETUP & MAIN LOOP */
// ============================================================

void setup() {
  Serial.begin(115200);
  initializeSystem();
  checkWiFiConnection();
  updateDisplay(displayData);
  Serial.println("Harmful Gas Monitor Started.\n");
}

void loop() {
  unsigned long currentTime = millis();

  // --- Section 2: Sensor Acquisition
  readSensors(currentTime);

  // --- Section 3 & 4: 5-second Processing, Air-Quality Decision & Display ---
  if (currentTime - lastDisplayAndUI >= DISPLAY_INTERVAL) {
    lastDisplayAndUI = currentTime;
    processData(false);
    evaluateAirQuality(displayData);
    checkWiFiConnection();
    updateDisplay(displayData);
  }

  // --- Section 3 & 5: 60-second Processing & LittleFS CSV Logging ---
  if (currentTime - lastWindowClose >= LOG_INTERVAL) {
    lastWindowClose = currentTime;
    processData(true);
    storeData(loggedData);
  }

  // --- Section 4: Continuous Buzzer State Machine ---
  updateAlerts();

  // --- Section 6: Joystick, Alert Flashing & Async Core 0 Firebase Upload ---
  updateCommunication(currentTime);

#if ENABLE_VERIFICATION_TESTS
  runTestCommands();
#endif
}


// ============================================================
// 1. INITIALIZATION
// ============================================================

void initializeSystem() {
  Serial.println("Initializing system...");
  initializeAlerts();
  initializeSensors();
  initializeDisplay();
  initializeStorage();
  initializeCommunication();
  Serial.println("Initialization complete.");
}

void initializeSensors() {
  Serial.println("Initializing sensor pins and buses...");
  pinMode(MQ7_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ7_PIN, ADC_11db);

#ifdef USE_MQ7_HEATER_CYCLE
  mq7ShouldSampleNow(millis()); // Start 60s 5V purge phase immediately at boot
#endif

  Wire.begin(I2C_SDA_SGP30_PIN, I2C_SCL_SGP30_PIN);
  dht.begin();

  sgp30Initialized = sgp.begin();
  if (!sgp30Initialized) {
    Serial.println("WARNING: SGP30 sensor not found on I2C!");
    faultDetections(SENSOR, false);
  }

  Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
}

void initializeDisplay() {
  initDisplayAndJoystickHardware();
}

void initializeStorage() {
  Serial.println("Initializing LittleFS and NVS storage...");
  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS mount failed!");
    faultDetections(STORAGE, false);
    return;
  }

  syncPrefs.begin("gas_sync", false);

  activeLogFileIndex = syncPrefs.getUChar("active_file", 1);
  if (activeLogFileIndex != 1 && activeLogFileIndex != 2) activeLogFileIndex = 1;
  lastUploadedByteOffset = syncPrefs.getUInt("upload_offset", 0);

  const char* activePath = getLogFilePath();
  if (!LittleFS.exists(activePath)) {
    if (initNewLogFile(activeLogFileIndex)) {
      Serial.printf("Created %s and set offset past header.\n", activePath);
    } else {
      faultDetections(STORAGE, false);
      return;
    }
  } else {
    File logFile = LittleFS.open(activePath, "r");
    if (logFile) {
      currentLogFileSize = logFile.size();
      logFile.close();
      if (lastUploadedByteOffset > currentLogFileSize) {
        lastUploadedByteOffset = 0;
        syncPrefs.putUInt("upload_offset", 0);
      }
    } else {
      faultDetections(STORAGE, false);
      return;
    }
  }

  faultDetections(STORAGE, true);
}

void initializeCommunication() {
  Serial.println("Initializing communication...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);         // Automatic Modem-Sleep to conserve battery
  WiFi.setAutoReconnect(true); // Non-blocking background reconnect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Background NTP time synchronization (UTC / Unix Epoch)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // Launch Core 0 HTTPS upload worker (8 KB stack for mbedTLS)
  firebaseUploadQueue = xQueueCreate(1, sizeof(UploadJob));
  firebaseResultQueue = xQueueCreate(1, sizeof(UploadResult));
  if (firebaseUploadQueue && firebaseResultQueue) {
    xTaskCreatePinnedToCore(
      firebaseUploadTask,
      "firebase_tx",
      8192,
      nullptr,
      1,
      nullptr,
      0
    );
  }
}

void initializeAlerts() {
  initBuzzerHardware();
}


// ============================================================
// 2. DATA ACQUISITION
// ============================================================

static void addSample(SensorTotal &safety, SensorTotal &logged, float val) {
  safety.sum += val;
  safety.count++;
  if (val > safety.maximum) safety.maximum = val;
  if (val < safety.minimum) safety.minimum = val;

  logged.sum += val;
  logged.count++;
  if (val > logged.maximum) logged.maximum = val;
  if (val < logged.minimum) logged.minimum = val;
}

static void resetSensorTotal(SensorTotal &t) {
  t.sum     = 0.0f;
  t.maximum = -INFINITY;
  t.minimum = INFINITY;
  t.count   = 0;
}

void clearSafetyAccumulators() {
  resetSensorTotal(safetyAccumulators.coPPM);
  resetSensorTotal(safetyAccumulators.tvocPPB);
  resetSensorTotal(safetyAccumulators.eco2PPM);
  resetSensorTotal(safetyAccumulators.temperatureC);
  resetSensorTotal(safetyAccumulators.humidityPercent);
  resetSensorTotal(safetyAccumulators.pm25UGM3);
}

void clearAccumulators() {
  resetSensorTotal(accumulators.coPPM);
  resetSensorTotal(accumulators.tvocPPB);
  resetSensorTotal(accumulators.eco2PPM);
  resetSensorTotal(accumulators.temperatureC);
  resetSensorTotal(accumulators.humidityPercent);
  resetSensorTotal(accumulators.pm25UGM3);
}

void readSensors(unsigned long currentTime) {
  // MQ-7: Continuous heater cycle when enabled, or 1s polling when disabled
#ifdef USE_MQ7_HEATER_CYCLE
  if (mq7ShouldSampleNow(currentTime)) {
    lastMQ7Read = currentTime;
    readmq7();
  } else if (rawData.mq7.coPPM.timestamp > 0 &&
             (currentTime - rawData.mq7.coPPM.timestamp > MQ7_STALE_TIMEOUT_MS)) {
    // Only flag stale if a previously valid cycle stops updating (> 160s); ignore initial 149.5s warm-up
    if (rawData.mq7.coPPM.valid) {
      rawData.mq7.coPPM.valid = false;
      faultDetections(STALE, false);
    }
  }
#else
  if (currentTime - lastMQ7Read >= MQ7_SAMPLE_INTERVAL) {
    lastMQ7Read = currentTime;
    readmq7();
  }
#endif

  if (currentTime - lastSGP30Read >= SGP30_SAMPLE_INTERVAL) {
    lastSGP30Read = currentTime;
    readsgp30();
  }
  if (currentTime - lastTempHumRead >= TEMPHUM_SAMPLE_INTERVAL) {
    lastTempHumRead = currentTime;
    readtemp();
    readhum();
  }
  if (currentTime - lastPM25Read >= PM25_SAMPLE_INTERVAL) {
    lastPM25Read = currentTime;
    readpm25();
  }
}

bool readmq7() {
  // Average 4 rapid ADC reads during the 5 ms solid-ground window to reduce noise
  long adcSum = 0;
  for (int i = 0; i < 4; i++) adcSum += analogRead(MQ7_PIN);
  float rawADC = adcSum / 4.0f;

  // Undo 10 kOhm / 20 kOhm protection divider to recover actual MQ-7 AO voltage (0..5V)
  float pinVoltage      = (rawADC / 4095.0f) * 3.3f;
  float actualAOVoltage = pinVoltage * MQ7_DIVIDER_SCALE;

  if (!isfinite(actualAOVoltage) || actualAOVoltage <= 0.01f || actualAOVoltage >= 4.99f) {
    rawData.mq7.coPPM.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  // Compute sensor resistance Rs and convert via MQ-7 power-law curve (calibrate MQ7_R0_OHMS in clean air)
  float rs    = MQ7_RL_OHMS * (5.0f - actualAOVoltage) / actualAOVoltage;
  float ratio = rs / MQ7_R0_OHMS;
  float coPPM = 99.042f * powf(ratio, -1.518f);

  if (!isfinite(coPPM) || coPPM < 0.0f) {
    rawData.mq7.coPPM.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.mq7.coPPM.value     = coPPM;
  rawData.mq7.coPPM.valid     = true;
  rawData.mq7.coPPM.timestamp = millis();
  addSample(safetyAccumulators.coPPM, accumulators.coPPM, coPPM);
  return true;
}

bool readsgp30() {
  if (!sgp30Initialized || !sgp.IAQmeasure()) {
    rawData.sgp30.eco2PPM.valid = false;
    rawData.sgp30.tvocPPB.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  unsigned long now = millis();
  float eco2Val = static_cast<float>(sgp.eCO2);
  float tvocVal = static_cast<float>(sgp.TVOC);

  rawData.sgp30.eco2PPM.value     = eco2Val;
  rawData.sgp30.eco2PPM.valid     = true;
  rawData.sgp30.eco2PPM.timestamp = now;
  addSample(safetyAccumulators.eco2PPM, accumulators.eco2PPM, eco2Val);

  rawData.sgp30.tvocPPB.value     = tvocVal;
  rawData.sgp30.tvocPPB.valid     = true;
  rawData.sgp30.tvocPPB.timestamp = now;
  addSample(safetyAccumulators.tvocPPB, accumulators.tvocPPB, tvocVal);
  return true;
}

bool readpm25() {
  if (!pms.read()) {
    if (rawData.pm25.concentration.timestamp > 0 &&
        (millis() - rawData.pm25.concentration.timestamp > PM25_SAMPLE_INTERVAL * STALE_TIMEOUT_MULTIPLIER)) {
      rawData.pm25.concentration.valid = false;
      faultDetections(STALE, false);
    }
    return false;
  }

  float pm25Val = static_cast<float>(pms.getPM2_5_ATM());
  if (!isfinite(pm25Val) || pm25Val < 0.0f || pm25Val > 1000.0f) {
    rawData.pm25.concentration.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.pm25.concentration.value     = pm25Val;
  rawData.pm25.concentration.valid     = true;
  rawData.pm25.concentration.timestamp = millis();
  addSample(safetyAccumulators.pm25UGM3, accumulators.pm25UGM3, pm25Val);
  return true;
}

bool readhum() {
  float val = dht.readHumidity();
  if (isnan(val) || val < 15.0f || val > 100.0f) {
    rawData.tempHum.humidityPercent.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.tempHum.humidityPercent.value     = val;
  rawData.tempHum.humidityPercent.valid     = true;
  rawData.tempHum.humidityPercent.timestamp = millis();
  addSample(safetyAccumulators.humidityPercent, accumulators.humidityPercent, val);
  return true;
}

bool readtemp() {
  float val = dht.readTemperature();
  if (isnan(val) || val < 5.0f || val > 80.0f) {
    rawData.tempHum.temperatureC.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.tempHum.temperatureC.value     = val;
  rawData.tempHum.temperatureC.valid     = true;
  rawData.tempHum.temperatureC.timestamp = millis();
  addSample(safetyAccumulators.temperatureC, accumulators.temperatureC, val);
  return true;
}


// ============================================================
// 3. DATA PROCESSING & FAULT DETECTION
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

// Holds the last valid CO reading across 5s & 60s windows between 150s MQ-7 heater cycles
static void finalizeCOChannel(const SensorTotal &src, AverageData &dst, unsigned long now) {
  if (src.count > 0) {
    finalizeChannel(src, dst);
    return;
  }

#ifdef USE_MQ7_HEATER_CYCLE
  if (rawData.mq7.coPPM.valid && isfinite(rawData.mq7.coPPM.value) &&
      (now - rawData.mq7.coPPM.timestamp <= MQ7_STALE_TIMEOUT_MS)) {
    dst.average     = rawData.mq7.coPPM.value;
    dst.maximum     = rawData.mq7.coPPM.value;
    dst.sampleCount = 1; // Held valid sample from current 150s thermal cycle
    dst.valid       = true;
    return;
  }
#endif

  // Initial 149.5s warm-up or sensor stale/invalid
  dst.average     = NAN;
  dst.maximum     = NAN;
  dst.sampleCount = 0;
  dst.valid       = false;
}

// option == true -> finalize 60s loggedData; option == false -> finalize 5s displayData
int processData(bool option) {
  unsigned long now = millis();

  if (option) {
    loggedData.windowEndTimestamp = now;
    finalizeCOChannel(accumulators.coPPM,          loggedData.coPPM,           now);
    finalizeChannel(accumulators.tvocPPB,          loggedData.tvocPPB);
    finalizeChannel(accumulators.eco2PPM,          loggedData.eco2PPM);
    finalizeChannel(accumulators.temperatureC,     loggedData.temperatureC);
    finalizeChannel(accumulators.humidityPercent,  loggedData.humidityPercent);
    finalizeChannel(accumulators.pm25UGM3,         loggedData.pm25UGM3);
    clearAccumulators();
  } else {
    displayData.windowEndTimestamp = now;
    finalizeCOChannel(safetyAccumulators.coPPM,         displayData.coPPM,           now);
    finalizeChannel(safetyAccumulators.tvocPPB,         displayData.tvocPPB);
    finalizeChannel(safetyAccumulators.eco2PPM,         displayData.eco2PPM);
    finalizeChannel(safetyAccumulators.temperatureC,    displayData.temperatureC);
    finalizeChannel(safetyAccumulators.humidityPercent, displayData.humidityPercent);
    finalizeChannel(safetyAccumulators.pm25UGM3,        displayData.pm25UGM3);
    clearSafetyAccumulators();
  }

  return 0;
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


// ============================================================
// 4. DECISION LOGIC
// ============================================================

void evaluateAirQuality(const ProcessedData &data) {
  bool danger = false;
  const char* msg = "Normal";

  if (C0_THRESHOLD2_PPM > 0.0f && data.coPPM.valid && data.coPPM.average >= C0_THRESHOLD2_PPM) {
    danger = true; msg = "HIGH CO";
  } else if (ECO2_THRESHOLD2_PPM > 0.0f && data.eco2PPM.valid && data.eco2PPM.average >= ECO2_THRESHOLD2_PPM) {
    danger = true; msg = "HIGH eCO2";
  } else if (TVOC_THRESHOLD2_PPB > 0.0f && data.tvocPPB.valid && data.tvocPPB.average >= TVOC_THRESHOLD2_PPB) {
    danger = true; msg = "HIGH TVOC";
  } else if (PM25_THRESHOLD2_UGM3 > 0.0f && data.pm25UGM3.valid && data.pm25UGM3.average >= PM25_THRESHOLD2_UGM3) {
    danger = true; msg = "HIGH PM2.5";
  }

  dangerAlertActive  = danger;
  activeAlertMessage = msg;
}

void updateAlerts() {
  refreshBuzzerState(millis());
}


// ============================================================
// 5. DATA STORAGE
// ============================================================

const char* getLogFilePath(bool active) {
  return ((activeLogFileIndex == 1) == active) ? LOG_FILE_1_PATH : LOG_FILE_2_PATH;
}

bool initNewLogFile(uint8_t fileIndex) {
  activeLogFileIndex = fileIndex;
  File f = LittleFS.open(getLogFilePath(), "w");
  if (!f) {
    faultDetections(STORAGE, false);
    return false;
  }

  f.println(CSV_HEADER);
  currentLogFileSize = lastUploadedByteOffset = f.size();
  f.close();

  syncPrefs.putUChar("active_file", activeLogFileIndex);
  syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
  faultDetections(STORAGE, true);
  return true;
}

void storeData(const ProcessedData &data) {
  if (faults.storage.status) return;

  time_t now = time(nullptr);
  unsigned long timestampSec = (now > 1700000000) ? static_cast<unsigned long>(now) : (data.windowEndTimestamp / 1000);

  char csvRow[256];
  int pos = snprintf(csvRow, sizeof(csvRow), "%lu,%d", timestampSec, dangerAlertActive ? 1 : 0);
  if (pos < 0 || static_cast<size_t>(pos) >= sizeof(csvRow)) return;

  const AverageData* m[6] = {
    &data.coPPM, &data.tvocPPB, &data.eco2PPM,
    &data.temperatureC, &data.humidityPercent, &data.pm25UGM3
  };
  for (int i = 0; i < 6; i++) {
    int n = 0;
    if (m[i]->valid && isfinite(m[i]->average)) {
      n = snprintf(csvRow + pos, sizeof(csvRow) - pos, ",%.2f,%u,1", m[i]->average, m[i]->sampleCount);
    } else {
      n = snprintf(csvRow + pos, sizeof(csvRow) - pos, ",NAN,%u,%d", m[i]->sampleCount, m[i]->valid ? 1 : 0);
    }
    if (n < 0 || static_cast<size_t>(pos + n) >= sizeof(csvRow)) return;
    pos += n;
  }

  if (currentLogFileSize + pos + 2 > MAX_LOG_FILE_SIZE) {
    if (!initNewLogFile((activeLogFileIndex == 1) ? 2 : 1)) return;
    Serial.printf("Switched active log to %s.\n", getLogFilePath());
  }

  File logFile = LittleFS.open(getLogFilePath(), "a");
  if (!logFile) {
    faultDetections(STORAGE, false);
    return;
  }

  size_t written = logFile.println(csvRow);
  currentLogFileSize = logFile.size();
  logFile.close();
  if (written < static_cast<size_t>(pos)) {
    faultDetections(STORAGE, false);
  }
}


// ============================================================
// 6. COMMUNICATION
// ============================================================

bool csvLineToJson(const String &csvLine, char* jsonOut, size_t jsonOutSize) {
  if (csvLine.startsWith("timestamp_")) return false;

  char buf[256];
  strncpy(buf, csvLine.c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char* tok[20];
  int n = 0;
  for (char* p = strtok(buf, ","); p && n < 20; p = strtok(nullptr, ",")) tok[n++] = p;
  if (n < 20) return false;

  unsigned long ts = strtoul(tok[0], nullptr, 10);
  if (ts < 1700000000UL && bootEpochSec > 0) {
    ts += bootEpochSec;
  }

  const char* keys[6] = {"co", "tvoc", "eco2", "temp", "hum", "pm25"};
  int pos = snprintf(jsonOut, jsonOutSize, "{\"timestamp_ms\":%lu", ts);
  if (pos < 0 || static_cast<size_t>(pos) >= jsonOutSize) return false;

  for (int i = 0; i < 6; i++) {
    int b = 2 + (i * 3);
    int w = snprintf(jsonOut + pos, jsonOutSize - pos,
                     ",\"%s\":{\"avg\":%s,\"count\":%s,\"valid\":%s}",
                     keys[i],
                     strcmp(tok[b], "NAN") ? tok[b] : "null",
                     tok[b + 1], tok[b + 2]);
    if (w < 0 || static_cast<size_t>(pos + w) >= jsonOutSize) return false;
    pos += w;
  }

  int w = snprintf(jsonOut + pos, jsonOutSize - pos, "}");
  return (w > 0 && static_cast<size_t>(pos + w) < jsonOutSize);
}

bool sendJsonToFirebase(const char* jsonPayload) {
  if (strlen(FIREBASE_URL) == 0) {
    Serial.printf("Sent to firebase: %s\n", jsonPayload);
    return true;
  }

  static WiFiClientSecure client;
  client.setInsecure();

  char fullUrl[256];
  if (strlen(FIREBASE_AUTH_TOKEN) > 0) {
    snprintf(fullUrl, sizeof(fullUrl), "%s?auth=%s", FIREBASE_URL, FIREBASE_AUTH_TOKEN);
  } else {
    snprintf(fullUrl, sizeof(fullUrl), "%s", FIREBASE_URL);
  }

  HTTPClient http;
  http.setReuse(true);
  http.setConnectTimeout(3000);
  http.setTimeout(5000);

  if (!http.begin(client, fullUrl)) return false;

  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t*)jsonPayload, strlen(jsonPayload));
  http.end();
  return (code >= 200 && code < 300);
}

void firebaseUploadTask(void* param) {
  UploadJob job;
  for (;;) {
    if (xQueueReceive(firebaseUploadQueue, &job, portMAX_DELAY) == pdTRUE) {
      bool ok = sendJsonToFirebase(job.json);
      UploadResult res = { ok, job.nextOffset, job.fileIndex };
      xQueueSend(firebaseResultQueue, &res, portMAX_DELAY);
    }
  }
}

void updateCommunication(unsigned long currentTime) {
  handleJoystickInput(currentTime, displayData);

  if (uploadInFlight && firebaseResultQueue) {
    UploadResult res;
    if (xQueueReceive(firebaseResultQueue, &res, 0) == pdTRUE) {
      uploadInFlight = false;
      lastTransmission = currentTime;
      if (res.success && res.fileIndex == activeLogFileIndex) {
        lastUploadedByteOffset = res.nextOffset;
        syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
      }
    }
  }

  if (!uploadInFlight && wifiConnected && (currentTime - lastTransmission >= TRANSMIT_INTERVAL)) {
    lastTransmission = currentTime;
    transmitData();
  }
}

void updateDisplay(const ProcessedData &data) {
  renderDisplay(data);
}

void transmitData() {
  if (uploadInFlight || !firebaseUploadQueue || faults.storage.status ||
      !wifiConnected || lastUploadedByteOffset >= currentLogFileSize) {
    return;
  }

  File logFile = LittleFS.open(getLogFilePath(), "r");
  if (!logFile) return;

  logFile.seek(lastUploadedByteOffset);
  String line = logFile.readStringUntil('\n');
  line.trim();
  size_t nextOffset = logFile.position();
  logFile.close();

  UploadJob job = {};
  job.nextOffset = nextOffset;
  job.fileIndex  = activeLogFileIndex;

  if (line.isEmpty() || !csvLineToJson(line, job.json, sizeof(job.json))) {
    lastUploadedByteOffset = nextOffset;
    syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
    return;
  }

  if (xQueueSend(firebaseUploadQueue, &job, 0) == pdTRUE) {
    uploadInFlight = true;
  }
}

void checkWiFiConnection() {
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  faultDetections(WIFI, wifiConnected);

  if (wifiConnected && !timeSyncedThisBoot) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      timeSyncedThisBoot = true;
      bootEpochSec       = static_cast<unsigned long>(now) - (millis() / 1000);
    }
  }
}