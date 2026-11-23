/* 
ELEC 290 Section 002-01- Harmful gas sensor firmware

Hardware:
ESP32-WROVER-E
SGP30 (I2C) - eCO2 and TVOC sensor
MQ-7 - CO sensor
DHT22 - temp and humid sensor
PMS7003 (UART) - pm2.5 sensor
Buzzer
2.0" ST7789 LCD + Joystick
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
#include "tests.h"               // Testing helper functions



// ============================================================
/* CONFIGURATION */
// ============================================================

// 1 = enable Serial Monitor verification test suite, 0 = disable for final release
#define ENABLE_VERIFICATION_TESTS 0

// Wi-Fi & database credentials
const char* WIFI_SSID = "jacob";
const char* WIFI_PASSWORD = "341145100";

const char* FIREBASE_URL ="https://gas-monitor-6e3eb-default-rtdb.firebaseio.com/devices/testmonitor/readings.json"; 
const char* FIREBASE_AUTH_TOKEN = "1YfvkBGVE4v2KK5FFd7N1QkqjXl5opGkmFKVwLUd";

// Pin assignments (aligned with ELEC290 Pinout.pdf — ESP32-WROVER-E)
// I2C Gas Sensor (CCS811 / SGP30)
const int I2C_SDA_SGP30_PIN  = 21; // SDA -> GPIO 21
const int I2C_SCL_SGP30_PIN  = 22; // SCL -> GPIO 22

// 2.0 Inch Screen Pinout (ST7789 Hardware VSPI)
extern const int I2C_SDA_SCREEN_PIN = 23; // SDA (MOSI) -> GPIO 23
extern const int I2C_SCL_SCREEN_PIN = 18; // SCL (SCK)  -> GPIO 18
extern const int SCREEN_CS_PIN      = 5;  // CS         -> GPIO 5
extern const int SCREEN_DC_PIN      = 27; // DC         -> GPIO 27
extern const int SCREEN_RST_PIN     = 26; // RST        -> GPIO 26

// Joystick Pinout
extern const int JOY_X_PIN  = 34; // VRx -> GPIO 34
extern const int JOY_Y_PIN  = 35; // VRy -> GPIO 35
extern const int JOY_SW_PIN = 32; // SW  -> GPIO 32

// PM2.5 (PMS7003 UART2)
const int PMS_RX_PIN = 33; // RX -> GPIO 33
const int PMS_TX_PIN = 19; // TX -> GPIO 19

// Analog & Digital Sensors
const int MQ7_PIN = 36; // MQ-7 AO  -> GPIO 36
const int DHT_PIN = 13; // DHT22 OUT -> GPIO 13 (preserved; not listed in Pinout PDF)

// Buzzer
extern const int BUZZER_PIN = 25; // Long Pin -> GPIO 25

// Sensor sampling intervals
const unsigned long MQ7_SAMPLE_INTERVAL     = 1000;
const unsigned long CCS811_SAMPLE_INTERVAL  = 1000;
const unsigned long TEMPHUM_SAMPLE_INTERVAL = 1000;
const unsigned long PM25_SAMPLE_INTERVAL    = 1000;

// Stale reading multiplier (stale reading timeout = sample interval * multiplier)
const unsigned long STALE_TIMEOUT_MULTIPLIER = 3;

// Data logging intervals
const unsigned long LOG_INTERVAL      = 60000; // how often information is logged in flash, as well as uploaded to the website
const unsigned long DISPLAY_INTERVAL  = 5000;  // how often the display is updated
const unsigned long TRANSMIT_INTERVAL = 2000;  // in cases where the wi-fi drops then reconnects, it will send unsent logs every # seconds

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
// Note: ProcessedData, AverageData, FaultStatus, and faultenum are 
// included via "local_communication.h" so all .cpp files share the same types.

// Raw sensor data structure
struct SensorReading {
  float value = NAN;
  bool valid = false;
  unsigned long timestamp = 0;
}; 

// Individual sensors
struct MQ7Data {
  SensorReading coPPM;
};

struct SGP30Data {
  SensorReading tvocPPB;
  SensorReading eco2PPM;
};

struct TempHumidityData {
  SensorReading temperatureC;
  SensorReading humidityPercent;
};

struct PM25Data {
  SensorReading concentration; // ug/m3
};

// Real-time data, used to update LCD and buzzer
struct RawSensorData {
  MQ7Data mq7;
  SGP30Data sgp30;
  TempHumidityData tempHum;
  PM25Data pm25;
};

// Running total during 60s and 5s window
struct SensorTotal {
  float sum = 0.0f;
  float maximum = -INFINITY;
  float minimum = INFINITY;
  unsigned int count = 0;
};

// Contains SensorTotal structs for each read in value (raw data), wrapped in a SensorAccumulator struct
struct SensorAccumulators {
  SensorTotal coPPM;
  SensorTotal tvocPPB;
  SensorTotal eco2PPM;
  SensorTotal temperatureC;
  SensorTotal humidityPercent;
  SensorTotal pm25UGM3;
};

// FreeRTOS IPC payloads (Sub-Option 1B: Core 1 owns flash, Core 0 owns HTTPS)
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
ProcessedData loggedData = {};              // Finalized 60-second average & peak data (CSV & Firebase)

FaultStatus faults = {};

bool wifiConnected        = false;
bool dangerAlertActive    = false;
String activeAlertMessage = "Normal";

// Hardware instances
Adafruit_SGP30 sgp;
DHT dht(DHT_PIN, DHT22);
GuL::PMS7003 pms(Serial2); 

// byte offset for last cloud upload in csv file
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


// ============================================================
/* FUNCTION DECLARATIONS */
// ============================================================

// Initialization
void initializeSystem();
void initializeSensors();
void initializeDisplay();
void initializeStorage();
void initializeCommunication();
void initializeAlerts();

// Data acquisition
void readSensors();
bool readmq7();
bool readsgp30();
bool readpm25();
bool readhum();
bool readtemp();
void accumulateData(bool fiveSec, bool sixtySec);
void clearSafetyAccumulators();
void clearAccumulators();
bool isMax(float val1, float val2);
bool isMin(float val1, float val2);

// Data processing & fault detection
int processData(bool option);
int faultDetections(faultenum fault, bool override);

// Decision logic
void updateAlerts();

// Data Storage
const char* getLogFilePath(bool active = true);
bool initNewLogFile(uint8_t fileIndex);
void storeData(const ProcessedData &data);

// Communication
bool csvLineToJson(const String &csvLine, char* jsonOut, size_t jsonOutSize);
bool sendJsonToFirebase(const char* jsonPayload);
void firebaseUploadTask(void* param);
void updateCommunication(unsigned long currentTime);
void updateDisplay(const ProcessedData &data);
void transmitData();
void checkWiFiConnection();


// ============================================================
/* SETUP */
// ============================================================

void setup() {
  Serial.begin(115200);
  initializeSystem();
  checkWiFiConnection();
  updateDisplay(displayData);
  Serial.println("Harmful Gas Monitor Started.\n");
}

// ============================================================
/* MAIN LOOP */
// ============================================================
void loop() {
  unsigned long currentTime = millis();

  // Data acquisition
  if (currentTime - lastPM25Read >= PM25_SAMPLE_INTERVAL) {
    lastPM25Read = currentTime;
    readpm25();
  }

  // Data processing and fault detection

  // Decision logic
  updateAlerts();

  // --- Section 6: Communication (Wi-Fi/Display Check, Joystick, Alert Flashing, Async Firebase Upload) ---
  if (currentTime - lastDisplayAndUI >= DISPLAY_INTERVAL) {
    lastDisplayAndUI = currentTime;
    checkWiFiConnection();
    updateDisplay(displayData);
  }
  updateCommunication(currentTime);

  // Data storage

  // Testing commands (from tests.cpp)
#if ENABLE_VERIFICATION_TESTS
  runTestCommands();
#endif
}

// ============================================================
/* INITIALIZATION */
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

  Wire.begin(I2C_SDA_SGP30_PIN, I2C_SCL_SGP30_PIN);
  dht.begin();

  if (!sgp.begin()) {
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

  // Configure background NTP time synchronization (UTC / Unix Epoch)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // Create 1-slot IPC queues and launch Core 0 HTTPS upload worker (8 KB stack for TLS)
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
/* DATA ACQUISITION */
// ============================================================

void readSensors() {
  // Read in all sensors into rawData and accumulators
}

bool readmq7() {
  float val = static_cast<float>(analogRead(MQ7_PIN));
  // TODO: Convert raw ADC value to CO ppm before summing

  rawData.mq7.coPPM.value     = val;
  rawData.mq7.coPPM.valid     = isfinite(val);
  rawData.mq7.coPPM.timestamp = millis();

  safetyAccumulators.coPPM.sum += val;
  accumulators.coPPM.sum       += val;
  safetyAccumulators.coPPM.count++;
  accumulators.coPPM.count++;

  if (isMax(val, safetyAccumulators.coPPM.maximum)) {
    safetyAccumulators.coPPM.maximum = val;
  }
  if (isMax(val, accumulators.coPPM.maximum)) {
    accumulators.coPPM.maximum = val;
  }
  if (isMin(val, safetyAccumulators.coPPM.minimum)) {
    safetyAccumulators.coPPM.minimum = val;
  }
  if (isMin(val, accumulators.coPPM.minimum)) {
    accumulators.coPPM.minimum = val;
  }
  return true;
}

bool readsgp30() {
  if (!sgp.IAQmeasure()) {
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

  safetyAccumulators.eco2PPM.sum += eco2Val;
  accumulators.eco2PPM.sum       += eco2Val;
  safetyAccumulators.eco2PPM.count++;
  accumulators.eco2PPM.count++;

  if (isMax(eco2Val, safetyAccumulators.eco2PPM.maximum)) {
    safetyAccumulators.eco2PPM.maximum = eco2Val;
  }
  if (isMax(eco2Val, accumulators.eco2PPM.maximum)) {
    accumulators.eco2PPM.maximum = eco2Val;
  }
  if (isMin(eco2Val, safetyAccumulators.eco2PPM.minimum)) {
    safetyAccumulators.eco2PPM.minimum = eco2Val;
  }
  if (isMin(eco2Val, accumulators.eco2PPM.minimum)) {
    accumulators.eco2PPM.minimum = eco2Val;
  }

  rawData.sgp30.tvocPPB.value     = tvocVal;
  rawData.sgp30.tvocPPB.valid     = true;
  rawData.sgp30.tvocPPB.timestamp = now;

  safetyAccumulators.tvocPPB.sum += tvocVal;
  accumulators.tvocPPB.sum       += tvocVal;
  safetyAccumulators.tvocPPB.count++;
  accumulators.tvocPPB.count++;

  if (isMax(tvocVal, safetyAccumulators.tvocPPB.maximum)) {
    safetyAccumulators.tvocPPB.maximum = tvocVal;
  }
  if (isMax(tvocVal, accumulators.tvocPPB.maximum)) {
    accumulators.tvocPPB.maximum = tvocVal;
  }
  if (isMin(tvocVal, safetyAccumulators.tvocPPB.minimum)) {
    safetyAccumulators.tvocPPB.minimum = tvocVal;
  }
  if (isMin(tvocVal, accumulators.tvocPPB.minimum)) {
    accumulators.tvocPPB.minimum = tvocVal;
  }

  return true;
}

bool readpm25() {
  // Poll UART2 for a complete 32-byte PMS7003 frame
  if (!pms.read()) {
    // Check if the last valid reading has exceeded the stale timeout (1000 ms * 3 = 3000 ms)
    if (rawData.pm25.concentration.timestamp > 0 &&
        (millis() - rawData.pm25.concentration.timestamp > PM25_SAMPLE_INTERVAL * STALE_TIMEOUT_MULTIPLIER)) {
      rawData.pm25.concentration.valid = false;
      faultDetections(STALE, false);
    }
    return false;
  }

  // Retrieve atmospheric PM2.5 concentration (ug/m3)
  float pm25Val = static_cast<float>(pms.getPM2_5_atm());

  // Validate physical sensor bounds (PMS7003 effective range: 0 to 1000 ug/m3)
  if (!isfinite(pm25Val) || pm25Val < 0.0f || pm25Val > 1000.0f) {
    rawData.pm25.concentration.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  // Store valid real-time reading
  rawData.pm25.concentration.value     = pm25Val;
  rawData.pm25.concentration.valid     = true;
  rawData.pm25.concentration.timestamp = millis();

  // Accumulate into 5-second (safety) and 60-second (logging) windows
  safetyAccumulators.pm25UGM3.sum += pm25Val;
  accumulators.pm25UGM3.sum       += pm25Val;
  safetyAccumulators.pm25UGM3.count++;
  accumulators.pm25UGM3.count++;

  if (isMax(pm25Val, safetyAccumulators.pm25UGM3.maximum)) {
    safetyAccumulators.pm25UGM3.maximum = pm25Val;
  }
  if (isMax(pm25Val, accumulators.pm25UGM3.maximum)) {
    accumulators.pm25UGM3.maximum = pm25Val;
  }
  if (isMin(pm25Val, safetyAccumulators.pm25UGM3.minimum)) {
    safetyAccumulators.pm25UGM3.minimum = pm25Val;
  }
  if (isMin(pm25Val, accumulators.pm25UGM3.minimum)) {
    accumulators.pm25UGM3.minimum = pm25Val;
  }

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

  safetyAccumulators.humidityPercent.sum += val;
  accumulators.humidityPercent.sum       += val;
  safetyAccumulators.humidityPercent.count++;
  accumulators.humidityPercent.count++;

  if (isMax(val, safetyAccumulators.humidityPercent.maximum)) {
    safetyAccumulators.humidityPercent.maximum = val;
  }
  if (isMax(val, accumulators.humidityPercent.maximum)) {
    accumulators.humidityPercent.maximum = val;
  }
  if (isMin(val, safetyAccumulators.humidityPercent.minimum)) {
    safetyAccumulators.humidityPercent.minimum = val;
  }
  if (isMin(val, accumulators.humidityPercent.minimum)) {
    accumulators.humidityPercent.minimum = val;
  }
  return true;
}

bool readtemp() {
  float val = dht.readTemperature();
  if (isnan(val) || val < 5.0f) {
    rawData.tempHum.temperatureC.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.tempHum.temperatureC.value     = val;
  rawData.tempHum.temperatureC.valid     = true;
  rawData.tempHum.temperatureC.timestamp = millis();

  safetyAccumulators.temperatureC.sum += val;
  accumulators.temperatureC.sum       += val;
  safetyAccumulators.temperatureC.count++;
  accumulators.temperatureC.count++;

  if (isMax(val, safetyAccumulators.temperatureC.maximum)) {
    safetyAccumulators.temperatureC.maximum = val;
  }
  if (isMax(val, accumulators.temperatureC.maximum)) {
    accumulators.temperatureC.maximum = val;
  }
  if (isMin(val, safetyAccumulators.temperatureC.minimum)) {
    safetyAccumulators.temperatureC.minimum = val;
  }
  if (isMin(val, accumulators.temperatureC.minimum)) {
    accumulators.temperatureC.minimum = val;
  }
  return true;
}

void accumulateData(bool fiveSec, bool sixtySec) {
  // safetyAccumulators is for 5 second display update
  // accumulators is for 60 second file write
  if (sixtySec) {
    processData(true);
  } else if (fiveSec) {
    processData(false);
  }
}

static void resetSensorTotal(SensorTotal &t) {
  t.sum     = 0.0f;
  t.maximum = -INFINITY;
  t.minimum = INFINITY;
  t.count   = 0;
}

/// @brief resets safety accumulators
void clearSafetyAccumulators() {
  resetSensorTotal(safetyAccumulators.coPPM);
  resetSensorTotal(safetyAccumulators.tvocPPB);
  resetSensorTotal(safetyAccumulators.eco2PPM);
  resetSensorTotal(safetyAccumulators.temperatureC);
  resetSensorTotal(safetyAccumulators.humidityPercent);
  resetSensorTotal(safetyAccumulators.pm25UGM3);
}

/// @brief resets accumulators
void clearAccumulators() {
  resetSensorTotal(accumulators.coPPM);
  resetSensorTotal(accumulators.tvocPPB);
  resetSensorTotal(accumulators.eco2PPM);
  resetSensorTotal(accumulators.temperatureC);
  resetSensorTotal(accumulators.humidityPercent);
  resetSensorTotal(accumulators.pm25UGM3);
}

/// @brief returns true if val1 > val2 
bool isMax(float val1, float val2) {
  return (val1 > val2);
}

/// @brief returns true if val1 < val2
bool isMin(float val1, float val2) {
  return (val1 < val2);
}


// ============================================================
/* DATA PROCESSING & FAULT DETECTION */
// ============================================================

static void finalizeChannel(const SensorTotal &src, AverageData &dst, bool checkStale) {
  if (src.count > 0) {
    dst.average     = src.sum / static_cast<float>(src.count);
    dst.maximum     = src.maximum;
    dst.sampleCount = src.count;
    dst.valid       = isfinite(dst.average);
    if (checkStale && src.count > 1 && dst.average == dst.maximum) {
      faultDetections(STALE, false);
    }
  } else {
    dst.average     = NAN;
    dst.maximum     = NAN;
    dst.sampleCount = 0;
    dst.valid       = false;
  }
}

/// @brief processes the raw data into averaged data. If option == true,
///        processes both 60-second loggedData and 5-second displayData.
int processData(bool option) {
  unsigned long now = millis();

  if (option) {
    loggedData.windowEndTimestamp = now;
    finalizeChannel(accumulators.coPPM,           loggedData.coPPM,           false);
    finalizeChannel(accumulators.tvocPPB,         loggedData.tvocPPB,         false);
    finalizeChannel(accumulators.eco2PPM,         loggedData.eco2PPM,         false);
    finalizeChannel(accumulators.temperatureC,    loggedData.temperatureC,    false);
    finalizeChannel(accumulators.humidityPercent, loggedData.humidityPercent, false);
    finalizeChannel(accumulators.pm25UGM3,        loggedData.pm25UGM3,        false);
    clearAccumulators();
  }

  displayData.windowEndTimestamp = now;
  finalizeChannel(safetyAccumulators.coPPM,           displayData.coPPM,           false);
  finalizeChannel(safetyAccumulators.tvocPPB,         displayData.tvocPPB,         true);
  finalizeChannel(safetyAccumulators.eco2PPM,         displayData.eco2PPM,         true);
  finalizeChannel(safetyAccumulators.temperatureC,    displayData.temperatureC,    true);
  finalizeChannel(safetyAccumulators.humidityPercent, displayData.humidityPercent, true);
  finalizeChannel(safetyAccumulators.pm25UGM3,        displayData.pm25UGM3,        true);

  updateDisplay(displayData);
  clearSafetyAccumulators();
  return 0;
}

/// @brief Increases the number of faults, and determines if faults should be updated 
///        and displayed on the LCD.
int faultDetections(faultenum fault, bool override) {
  if (fault == SENSOR) {
    faults.sensor.num++;
    if (faults.sensor.num >= allowedSensorFaults) {
      faults.sensor.status = true;
    }
    if (override) {
      faults.sensor.num = 0;
      faults.sensor.status = false;
    }
  }

  if (fault == INVALID) {
    faults.readingInvalid.num++;
    if (faults.readingInvalid.num >= allowedInvalidReadings) {
      faults.readingInvalid.status = true;
    }
    if (override) {
      faults.readingInvalid.num = 0;
      faults.readingInvalid.status = false;
    }
  }

  if (fault == STALE) {
    faults.readingStale.num++;
    if (faults.readingStale.num >= allowedStaleReadings) {
      faults.readingStale.status = true;
    }
    if (override) {
      faults.readingStale.num = 0;
      faults.readingStale.status = false;
    }
  }

  if (fault == STORAGE) {
    faults.storage.num++;
    if (faults.storage.num >= allowedStorageFaults) {
      faults.storage.status = true;
    }
    if (override) {
      faults.storage.num = 0;
      faults.storage.status = false;
    }
  }

  if (fault == WIFI) {
    faults.wifi.num++;
    if (faults.wifi.num >= allowedWifiFaults) {
      faults.wifi.status = true;
    }
    if (override) {
      faults.wifi.num = 0;
      faults.wifi.status = false;
    }
  }

  return 0;
}


// ============================================================
/* DECISION LOGIC */
// ============================================================

// updates the buzzer state (pulsing alarm when dangerAlertActive is true, or muted via joystick click)
void updateAlerts() {
  refreshBuzzerState(millis());
}


// ============================================================
// 5. DATA STORAGE
// ============================================================

// returns the file path for the active log file (when active is true) or the inactive file (when false)
const char* getLogFilePath(bool active) {
  return ((activeLogFileIndex == 1) == active) ? LOG_FILE_1_PATH : LOG_FILE_2_PATH;
}

// creates or overwrites a log file with the CSV header, updates the cached file size, and saves state to NVS
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

// formats the 60-second processed sensor data (avg, count, valid) into a CSV row, rotates files if full, and appends it to flash
void storeData(const ProcessedData &data) {
  if (faults.storage.status) return;

  // use real unix time if NTP has synced (> 1700000000), otherwise log relative seconds since boot
  time_t now = time(nullptr);
  unsigned long timestampSec = (now > 1700000000) ? static_cast<unsigned long>(now) : (data.windowEndTimestamp / 1000);

  // start building the csv row with the timestamp and whether an alert is active
  char csvRow[256];
  int pos = snprintf(csvRow, sizeof(csvRow), "%lu,%d", timestampSec, dangerAlertActive ? 1 : 0);

  // loop through all 6 sensor summaries in ProcessedData and append their avg, count, and valid flag (no max)
  const AverageData* m = &data.coPPM;
  for (int i = 0; i < 6; i++) {
    if (m[i].valid && isfinite(m[i].average)) {
      pos += snprintf(csvRow + pos, sizeof(csvRow) - pos,
                      ",%.2f,%u,1", m[i].average, m[i].sampleCount);
    } else {
      pos += snprintf(csvRow + pos, sizeof(csvRow) - pos,
                      ",NAN,%u,%d", m[i].sampleCount, m[i].valid ? 1 : 0);
    }
  }

  // if adding a new row would make the file too big, wipe the other file and switch over to it
  if (currentLogFileSize + pos + 2 > MAX_LOG_FILE_SIZE) {
    if (!initNewLogFile((activeLogFileIndex == 1) ? 2 : 1)) return;
    Serial.printf("Switched active log to %s.\n", getLogFilePath());
  }

  // open the active log file in append mode, write the new csv row, and update the cached file size
  File logFile = LittleFS.open(getLogFilePath(), "a");
  if (!logFile) {
    faultDetections(STORAGE, false);
    return;
  }

  size_t written = logFile.println(csvRow);
  currentLogFileSize = logFile.size();
  logFile.close();
  if (written < (size_t)pos) {
    faultDetections(STORAGE, false);
  }
}


// ============================================================
// 6. COMMUNICATION
// ============================================================

// takes a single 20-column comma-separated row from the CSV file and converts it into JSON for Firebase
bool csvLineToJson(const String &csvLine, char* jsonOut, size_t jsonOutSize) {
  if (csvLine.startsWith("timestamp_")) return false;

  // copy the csv line into a temporary char buffer and split it at every comma into 20 tokens
  char buf[256];
  strncpy(buf, csvLine.c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char* tok[20];
  int n = 0;
  for (char* p = strtok(buf, ","); p && n < 20; p = strtok(nullptr, ",")) tok[n++] = p;
  if (n < 20) return false;

  // if the row was logged offline using seconds since boot (< 1700000000) and NTP is now synced, convert it to up-to-date unix time
  unsigned long ts = strtoul(tok[0], nullptr, 10);
  if (ts < 1700000000UL && bootEpochSec > 0) {
    ts += bootEpochSec;
  }

  // write the timestamp at the start of the json (tok[1] / alert_active is intentionally omitted)
  const char* keys[6] = {"co", "tvoc", "eco2", "temp", "hum", "pm25"};
  int pos = snprintf(jsonOut, jsonOutSize, "{\"timestamp_ms\":%lu", ts);

  // ============================================================
  // ORIGINAL: Nested JSON (subfields: avg, count, valid)
  // ============================================================
  for (int i = 0; i < 6; i++) {
    int b = 2 + (i * 3);
    pos += snprintf(jsonOut + pos, jsonOutSize - pos,
                    ",\"%s\":{\"avg\":%s,\"count\":%s,\"valid\":%s}",
                    keys[i],
                    strcmp(tok[b], "NAN") ? tok[b] : "null",
                    tok[b + 1], tok[b + 2]);
  }

  /*
  // Backup: no subfields
  for (int i = 0; i < 6; i++) {
    int b = 2 + (i * 3);
    pos += snprintf(jsonOut + pos, jsonOutSize - pos,
                    ",\"%s_avg\":%s,\"%s_count\":%s,\"%s_valid\":%s",
                    keys[i], strcmp(tok[b], "NAN") ? tok[b] : "null",
                    keys[i], tok[b + 1],
                    keys[i], tok[b + 2]);
  }
  */

  snprintf(jsonOut + pos, jsonOutSize - pos, "}");
  return true;
}

// runs on Core 0: sends the formatted JSON string to Firebase over HTTPS POST and returns true if accepted
bool sendJsonToFirebase(const char* jsonPayload) {
  if (strlen(FIREBASE_URL) == 0) {
    Serial.printf("Sent to firebase: %s\n", jsonPayload);
    return true;
  }

  static WiFiClientSecure client;
  client.setInsecure(); // Skip root CA verification for simplicity

  HTTPClient http;
  http.setReuse(true);
  http.setConnectTimeout(3000);
  http.setTimeout(3000);

  String fullUrl = String(FIREBASE_URL) + (strlen(FIREBASE_AUTH_TOKEN) ? ("?auth=" + String(FIREBASE_AUTH_TOKEN)) : "");
  if (!http.begin(client, fullUrl)) return false;

  http.addHeader("Content-Type", "application/json");
  int code = http.POST(reinterpret_cast<const uint8_t*>(jsonPayload), strlen(jsonPayload));
  http.end();
  return (code >= 200 && code < 300);
}

// Core 0 FreeRTOS worker task: blocks until Core 1 queues an upload job, performs HTTPS POST, returns result
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

// Core 1: polls joystick, harvests completed Core 0 uploads, and dispatches the next CSV row every TRANSMIT_INTERVAL
void updateCommunication(unsigned long currentTime) {
  handleJoystickInput(currentTime, displayData);

  // Check if Core 0 finished the in-flight HTTPS upload (non-blocking)
  if (uploadInFlight && firebaseResultQueue) {
    UploadResult res;
    if (xQueueReceive(firebaseResultQueue, &res, 0) == pdTRUE) {
      uploadInFlight = false;
      lastTransmission = currentTime;
      // Advance NVS bookmark only if upload succeeded and log file did not rotate mid-upload
      if (res.success && res.fileIndex == activeLogFileIndex) {
        lastUploadedByteOffset = res.nextOffset;
        syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
      }
    }
  }

  // Queue the next unsent row when Core 0 is idle and Wi-Fi is connected
  if (!uploadInFlight && wifiConnected && (currentTime - lastTransmission >= TRANSMIT_INTERVAL)) {
    lastTransmission = currentTime;
    transmitData();
  }
}

// updates the 2.0" ST7789 LCD screen
void updateDisplay(const ProcessedData &data) {
  renderDisplay(data);
}

// Core 1: reads the next unsent CSV row from LittleFS, formats JSON, and hands it off to Core 0
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

  // Skip empty/header/malformed lines directly on Core 1 without waking Core 0
  if (line.isEmpty() || !csvLineToJson(line, job.json, sizeof(job.json))) {
    lastUploadedByteOffset = nextOffset;
    syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
    return;
  }

  if (xQueueSend(firebaseUploadQueue, &job, 0) == pdTRUE) {
    uploadInFlight = true;
  }
}

// checks if wi-fi is connected and calculates the boot epoch time the first time NTP gets real time
void checkWiFiConnection() {
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  faultDetections(WIFI, wifiConnected);

  // the first time wi-fi connects and the NTP clock gets a valid unix timestamp (> 1700000000),
  // calculate the exact unix time when the esp32 booted so offline timestamps can be converted on upload
  if (wifiConnected && !timeSyncedThisBoot) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      timeSyncedThisBoot = true;
      bootEpochSec       = static_cast<unsigned long>(now) - (millis() / 1000);
    }
  }
}