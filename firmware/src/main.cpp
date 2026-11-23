/* 
ELEC 290 Section 002-01- Harmful gas sensor firmware

Hardware:
ESP32-WROVER
SGP30 - eCO2 and TVOC sensor
MQ-7 - CO sensor
DHT22 - temp and humid sensor
PMS7003 - pm2.5 sensor
Buzzer
LCD
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

const char* FIREBASE_URL ="https://gas-monitor-6e3eb-default-rtdb.firebaseio.com/devices/testmonitor/readings.json"; // idk if itll be a url or an endpoint
const char* FIREBASE_AUTH_TOKEN = "1YfvkBGVE4v2KK5FFd7N1QkqjXl5opGkmFKVwLUd"; // idk if actual authentication is necessary

// Pin assignments (ESP32 GPIO numbers from schematic U1)
const int I2C_SDA_SGP30_PIN  = 21; // IO21 (Schematic physical pin 33)
const int I2C_SCL_SGP30_PIN  = 22; // IO22 (Schematic physical pin 36)

extern const int I2C_SDA_SCREEN_PIN = 23; // IO23 (Schematic physical pin 37 - SDA/MOSI on CN1)
extern const int I2C_SCL_SCREEN_PIN = 18; // IO18 (Schematic physical pin 30 - SCL/SCK on CN1)
extern const int SCREEN_CS_PIN      = 5;  // IO5  (Schematic physical pin 29 - CS on CN1)
extern const int SCREEN_DC_PIN      = 27; // IO27 (Schematic physical pin 12 - DC on CN1)
extern const int SCREEN_RST_PIN     = 26; // IO26 (Schematic physical pin 11 - RST on CN1)

// Joystick pins (H3)
extern const int JOY_X_PIN  = 34; // IO34 (Schematic physical pin 6 - VRx)
extern const int JOY_Y_PIN  = 35; // IO35 (Schematic physical pin 7 - VRy)
extern const int JOY_SW_PIN = 32; // IO32 (Schematic physical pin 8 - SW)

const int PMS_RX_PIN = 33; // IO33 (Schematic physical pin 9 - RX)
const int PMS_TX_PIN = 19; // IO19 (Schematic physical pin 31 - TX)

//sensor pins
const int MQ7_PIN   = 36; // SENSOR_VP / IO36 (Schematic physical pin 4 - AO)
const int DHT_PIN   = 13; // IO13 (Schematic physical pin 16 - OUT)
const int SGP30_PIN = 21; // Uses I2C (IO21 / IO22)
const int PM25_PIN  = 33; // Uses UART2 (IO33 / IO19)

extern const int BUZZER_PIN = 25; // IO25 (Schematic physical pin 10 - PIN 25)

// Sensor sampling intervals
const unsigned long MQ7_SAMPLE_INTERVAL = 1000;
const unsigned long CCS811_SAMPLE_INTERVAL = 1000;
const unsigned long TEMPHUM_SAMPLE_INTERVAL = 1000;
const unsigned long PM25_SAMPLE_INTERVAL = 1000;

// Stale reading multiplier (stale reading timeout = sample interval * multiplier)
const unsigned long STALE_TIMEOUT_MULTIPLIER = 3;


// Data loggging intervals
const unsigned long LOG_INTERVAL = 60000; // how often information is logged in flash, as well as uploaded to the website
const unsigned long DISPLAY_INTERVAL = 5000; // how often the display is updated
const unsigned long TRANSMIT_INTERVAL = 2000; // in cases where the wi-fi drops then reconnects, it will send unsent logs every # seconds


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
ProcessedData displayData = {};             // Finalized 5-second trimmed-mean data (LCD & Buzzer)

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
void acquireMQ7(unsigned long currentTime);
void acquireSGP30(unsigned long currentTime);
void acquireTempHumidity(unsigned long currentTime);
void acquirePM25(unsigned long currentTime);
bool readpm25();
void clearSafetyAccumulators();
void clearAccumulators();
bool isMax(float val1, float val2);
bool isMin(int val1, int val2);


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
  unsigned long lastTime1 = 0;
  unsigned long lastTimetemphum = 0;
  unsigned long display = 0;
  unsigned long send = 0;
// ============================================================
/* MAIN LOOP */
// ============================================================
void loop() {
  unsigned long currentTime = millis();

  // Data acquisitiion
  if (currentTime - lastPM25Read >= PM25_SAMPLE_INTERVAL) {
    lastPM25Read = currentTime;
    readpm25();
  }

  // data processing and fault detection

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
/* INITIALIZATION*/
// ============================================================

void initializeSystem() {
  Serial.println("Initializing system..."); // for debug purposes
  initializeAlerts();
  initializeSensors();
  initializeDisplay();
  initializeStorage();
  initializeCommunication();
  Serial.println("Initialization complete."); // for debug purposes
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

/// @brief sensors are read into the rawData struct, and then fed into accumulateData()
///        to obtain an average for the raw data. 
/// @param rawData 
void readSensors(){
  //rawSensorData struct containing individual sensor data, which is updated every sensor read
  //sensorAccumulator struct which that raw data is added in sum, and count++ to later be used
  //in data processing
  
  //read in all data 
}

/// @brief reads in data from each sensor, storing as a float, then adds the value to
///        its appropriate accumulator. If it is the highest that was seen, in the 
///        last period, also update the maximum value.

///        This is simply reading in values and accumulating it into categories.
///        processData will have the average values that get displayed and stored
bool readmq7() {
  rawData.mq7.coPPM.value = analogRead(MQ7_PIN);
  int r = 10000;


  rawData.mq7.coPPM.value;
  // more needs to be done before summing

  safetyAccumulators.coPPM.sum += rawData.mq7.coPPM.value;
  accumulators.coPPM.sum += rawData.mq7.coPPM.value;
  safetyAccumulators.coPPM.count++;
  accumulators.coPPM.count++;
  if (isMax(rawData.mq7.coPPM.value, safetyAccumulators.coPPM.maximum)){
    safetyAccumulators.coPPM.maximum = rawData.mq7.coPPM.value;
  }
  if (isMax(rawData.mq7.coPPM.value, accumulators.coPPM.maximum)){
    accumulators.coPPM.maximum = rawData.mq7.coPPM.value;
  }

  if (isMin(rawData.mq7.coPPM.value, safetyAccumulators.coPPM.minimum)){
    safetyAccumulators.coPPM.minimum = rawData.mq7.coPPM.value;
  }
  if (isMin(rawData.mq7.coPPM.value, accumulators.coPPM.minimum)){
    accumulators.coPPM.minimum= rawData.mq7.coPPM.value;
  }
  return true;
}

bool readsgp30(){
  
  if (!sgp.IAQmeasure()){ //IAQmeasure return value is a bool
      faults.readingInvalid.num++;
      return false;
      //faultDetections(STORAGE, false);
  }
  rawData.sgp30.eco2PPM.value = sgp.eCO2; //i think this is in ppm

  safetyAccumulators.eco2PPM.sum+=rawData.sgp30.eco2PPM.value;
  accumulators.eco2PPM.sum+=rawData.sgp30.eco2PPM.value;
  safetyAccumulators.eco2PPM.count++;
  accumulators.eco2PPM.count++;

  if (isMax(rawData.sgp30.eco2PPM.value, safetyAccumulators.eco2PPM.maximum)){
    safetyAccumulators.eco2PPM.maximum = rawData.sgp30.eco2PPM.value;
  }
  if (isMax(rawData.sgp30.eco2PPM.value, accumulators.eco2PPM.maximum)){
    accumulators.eco2PPM.maximum= rawData.sgp30.eco2PPM.value;
  }

  if (isMin(rawData.sgp30.eco2PPM.value, safetyAccumulators.eco2PPM.minimum)){
    safetyAccumulators.eco2PPM.minimum = rawData.sgp30.eco2PPM.value;
  }
  if (isMin(rawData.sgp30.eco2PPM.value, accumulators.eco2PPM.minimum)){
    accumulators.eco2PPM.minimum= rawData.sgp30.eco2PPM.value;
  }

  rawData.sgp30.tvocPPB.value = sgp.TVOC; //i think this is in ppb
    
  safetyAccumulators.tvocPPB.sum+=rawData.sgp30.tvocPPB.value;
  accumulators.tvocPPB.sum+=rawData.sgp30.tvocPPB.value;
  safetyAccumulators.tvocPPB.count++;
  accumulators.tvocPPB.count++;
  
  if (isMax(rawData.sgp30.tvocPPB.value, safetyAccumulators.tvocPPB.maximum)){
    safetyAccumulators.tvocPPB.maximum = rawData.sgp30.tvocPPB.value;
  }
  if (isMax(rawData.sgp30.tvocPPB.value, accumulators.tvocPPB.maximum)){
    accumulators.tvocPPB.maximum=rawData.sgp30.tvocPPB.value;
  }

  if (isMin(rawData.sgp30.tvocPPB.value, safetyAccumulators.tvocPPB.minimum)){
    safetyAccumulators.tvocPPB.minimum = rawData.sgp30.tvocPPB.value;
  }
  if (isMin(rawData.sgp30.tvocPPB.value, accumulators.tvocPPB.minimum)){
    accumulators.tvocPPB.minimum=rawData.sgp30.tvocPPB.value;
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
  if (pm25Val < safetyAccumulators.pm25UGM3.minimum) {
    safetyAccumulators.pm25UGM3.minimum = pm25Val;
  }
  if (pm25Val < accumulators.pm25UGM3.minimum) {
    accumulators.pm25UGM3.minimum = pm25Val;
  }

  return true;
}

bool readhum(){

  rawData.tempHum.humidityPercent.value = dht.readHumidity();
  if (rawData.tempHum.humidityPercent.value < 15){
    faultDetections(INVALID, false);
    return false;
  }

  safetyAccumulators.humidityPercent.sum+=rawData.tempHum.humidityPercent.value;
  accumulators.humidityPercent.sum+=rawData.tempHum.humidityPercent.value;
  safetyAccumulators.humidityPercent.count++;
  accumulators.humidityPercent.count++;
  
  if (isMax(rawData.tempHum.humidityPercent.value, safetyAccumulators.humidityPercent.maximum)){
    safetyAccumulators.humidityPercent.maximum = rawData.tempHum.humidityPercent.value;
  }
  if (isMax(rawData.tempHum.humidityPercent.value, accumulators.humidityPercent.maximum)){
    accumulators.humidityPercent.maximum=rawData.tempHum.humidityPercent.value;
  }

  if (isMin(rawData.tempHum.humidityPercent.value, safetyAccumulators.humidityPercent.minimum)){
    safetyAccumulators.humidityPercent.minimum = rawData.tempHum.humidityPercent.value;
  }
  if (isMin(rawData.tempHum.humidityPercent.value, accumulators.humidityPercent.minimum)){
    accumulators.humidityPercent.minimum=rawData.tempHum.humidityPercent.value;
  }
  return 1;
}

bool readtemp(){

  rawData.tempHum.temperatureC.value = dht.readTemperature();
  if (rawData.tempHum.temperatureC.value < 5){
      faultDetections(INVALID, false);
      return false;
  }
  safetyAccumulators.temperatureC.sum+=rawData.tempHum.temperatureC.value;
  accumulators.temperatureC.sum+=rawData.tempHum.temperatureC.value;
  safetyAccumulators.temperatureC.count++;
  accumulators.temperatureC.count++;
  
  if (isMax(rawData.tempHum.temperatureC.value, safetyAccumulators.temperatureC.maximum)){
    safetyAccumulators.temperatureC.maximum = rawData.tempHum.temperatureC.value;
  }
  if (isMax(rawData.tempHum.temperatureC.value, accumulators.temperatureC.maximum)){
    accumulators.temperatureC.maximum=rawData.tempHum.temperatureC.value;
  }

  if (isMin(rawData.tempHum.temperatureC.value, safetyAccumulators.temperatureC.minimum)){
    safetyAccumulators.temperatureC.minimum = rawData.tempHum.temperatureC.value;
  }
  if (isMin(rawData.tempHum.temperatureC.value, accumulators.temperatureC.minimum)){
    accumulators.temperatureC.minimum=rawData.tempHum.temperatureC.value;
  }

  return true;
}


void accumulateData(bool fiveSec, bool sixtySec){
  //safetyAccumulators is for 5 second display update
  //accumulators is for 60 second file write
  if (fiveSec){
    processData(0);
    clearSafetyAccumulators();
  }
  if(sixtySec){
    processData(1);
    clearAccumulators();
  }

}

/// @brief resets safety accumulators
void clearSafetyAccumulators(){
  safetyAccumulators.coPPM.count = 0;
  safetyAccumulators.coPPM.maximum = 0;
  safetyAccumulators.coPPM.sum = 0;
  safetyAccumulators.eco2PPM.count = 0;
  safetyAccumulators.eco2PPM.maximum = 0;
  safetyAccumulators.eco2PPM.sum = 0;
  safetyAccumulators.humidityPercent.count = 0;
  safetyAccumulators.humidityPercent.maximum = 0;
  safetyAccumulators.humidityPercent.sum = 0;
  safetyAccumulators.pm25UGM3.count = 0;
  safetyAccumulators.pm25UGM3.maximum = -INFINITY;
  safetyAccumulators.pm25UGM3.minimum = INFINITY;
  safetyAccumulators.pm25UGM3.sum = 0.0f;
  safetyAccumulators.temperatureC.count = 0;
  safetyAccumulators.temperatureC.maximum = 0;
  safetyAccumulators.temperatureC.sum = 0;
  safetyAccumulators.tvocPPB.count = 0;
  safetyAccumulators.tvocPPB.maximum = 0;
  safetyAccumulators.tvocPPB.sum = 0;
}

/// @brief resets accumulators
void clearAccumulators(){
  accumulators.coPPM.count = 0;
  accumulators.coPPM.maximum = 0;
  accumulators.coPPM.sum = 0;
  accumulators.eco2PPM.count = 0;
  accumulators.eco2PPM.maximum = 0;
  accumulators.eco2PPM.sum = 0;
  accumulators.humidityPercent.count = 0;
  accumulators.humidityPercent.maximum = 0;
  accumulators.humidityPercent.sum = 0;
  accumulators.pm25UGM3.count = 0;
  accumulators.pm25UGM3.maximum = -INFINITY;
  accumulators.pm25UGM3.minimum = INFINITY;
  accumulators.pm25UGM3.sum = 0.0f;
  accumulators.temperatureC.count = 0;
  accumulators.temperatureC.maximum = 0;
  accumulators.temperatureC.sum = 0;
  accumulators.tvocPPB.count = 0;
  accumulators.tvocPPB.maximum = 0;
  accumulators.tvocPPB.sum = 0;
}

/// @brief returns true if val1 > val2 
/// @param val1 
/// @param val2 
/// @return bool
bool isMax(float val1, float val2){
  return (val1 > val2) ? true: false;
}

/// @brief returns true if val1 < val2
/// @param val1 
/// @param val2 
/// @return bool
bool isMin(int val1, int val2){
  return (val1 < val2) ? true: false;
}

// ============================================================
/* DATA PROCESSING & FAULT DETECTION */
// ============================================================

//!!Copy and pasted here so I don't have to scroll all the way back up 
/*struct SensorTotal {
  float sum = 0.0f;
  float maximum = -INFINITY;
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
struct AverageData {
  float average = NAN;
  float maximum = NAN;
  unsigned int sampleCount = 0;
  bool valid = false;
};
struct ProcessedData {
  unsigned long windowEndTimestamp = 0;
  AverageData coPPM;
  AverageData tvocPPB;
  AverageData eco2PPM;
  AverageData temperatureC;
  AverageData humidityPercent;
  AverageData pm25UGM3;
};*/

/// @brief processes the raw data into averaged data. If this is called with 
///        option = true, process the 60 second data. else, process the 5 second data.
/// @param option 
/// @return 
int processData(bool option){

  if (option){
    loggedData.coPPM.average=accumulators.coPPM.sum/accumulators.coPPM.count;
    loggedData.coPPM.maximum=accumulators.coPPM.maximum;
    //add windowendtimestamp

    loggedData.eco2PPM.average=accumulators.eco2PPM.sum/accumulators.eco2PPM.count;
    loggedData.eco2PPM.maximum=accumulators.eco2PPM.maximum;
    //add windowendtimestamp
    
    loggedData.humidityPercent.average=accumulators.humidityPercent.sum/accumulators.humidityPercent.count;
    loggedData.humidityPercent.maximum=accumulators.humidityPercent.maximum;
    //add windowendtimestamp

    loggedData.pm25UGM3.average=accumulators.pm25UGM3.sum/accumulators.pm25UGM3.count;
    loggedData.pm25UGM3.maximum=accumulators.pm25UGM3.maximum;
    //add windowendtimestamp

    loggedData.temperatureC.average=accumulators.temperatureC.sum/accumulators.temperatureC.count;
    loggedData.temperatureC.maximum=accumulators.temperatureC.maximum;
    //add windowendtimestamp

    loggedData.tvocPPB.average=accumulators.tvocPPB.sum/accumulators.tvocPPB.count;
    loggedData.tvocPPB.maximum=accumulators.tvocPPB.maximum;
    clearAccumulators();
    //add windowendtimestamp
    
    //maybe call storedata here?
  }
    //it will also update the screen data, so not in an else
    displayData.coPPM.average=safetyAccumulators.coPPM.sum/safetyAccumulators.coPPM.count;
    displayData.coPPM.maximum=safetyAccumulators.coPPM.maximum;
  
    displayData.eco2PPM.average=safetyAccumulators.eco2PPM.sum/safetyAccumulators.eco2PPM.count;
    displayData.eco2PPM.maximum=safetyAccumulators.eco2PPM.maximum;
    if (displayData.eco2PPM.average = displayData.eco2PPM.maximum){
      faultDetections(STALE, false);
    }

    displayData.humidityPercent.average=safetyAccumulators.humidityPercent.sum/safetyAccumulators.humidityPercent.count;
    displayData.humidityPercent.maximum=safetyAccumulators.humidityPercent.maximum;
    if (displayData.humidityPercent.average = displayData.humidityPercent.maximum){
      faultDetections(STALE, false);
    }

    displayData.pm25UGM3.average=safetyAccumulators.pm25UGM3.sum/safetyAccumulators.pm25UGM3.count;
    displayData.pm25UGM3.maximum=safetyAccumulators.pm25UGM3.maximum;
    if (displayData.pm25UGM3.average = displayData.pm25UGM3.maximum){
      faultDetections(STALE, false);
    }

    displayData.temperatureC.average=safetyAccumulators.temperatureC.sum/safetyAccumulators.temperatureC.count;
    displayData.temperatureC.maximum=safetyAccumulators.temperatureC.maximum;
    if (displayData.temperatureC.average = displayData.temperatureC.maximum){
      faultDetections(STALE, false);
    }

    displayData.tvocPPB.average=safetyAccumulators.tvocPPB.sum/safetyAccumulators.tvocPPB.count;
    displayData.tvocPPB.maximum=safetyAccumulators.tvocPPB.maximum;
    if (displayData.tvocPPB.average = displayData.tvocPPB.maximum){
      faultDetections(STALE, false);
    }
    updateDisplay(displayData);
    clearSafetyAccumulators();
    //maybe call updatedisplay?
}

/// @brief Increases the number of faults, and determines if faults should be updated 
///        and displayed on the LCD.
/// @return 
int faultDetections(faultenum fault, bool override){
  /*
    EVERY time a fault is reported, this should be called to log that a fault has happened.
    Storage has a requirement of 1 report -> it is going to be a true fault if any storage error occurs
    Wifi will simply show that it is not connected to wifi - intentionally or unintentionally
    Sensors should display errors whenever it has an issue, but is only a 'true fault' after so many
        --> This meaning readingInvalid or readingStale.
        --> Actual sensor issues are like storage, sensor.status = true persisting until it connects 
  
    overrideFault resets the fault display once connection has been established/issue addressed.
    if 0, do not override any.
    else, follow enum guide
      
  */
 if (fault == SENSOR){
    faults.sensor.num++;
    if (faults.sensor.num >= allowedSensorFaults){
      faults.sensor.status = true;
    }
    if (override){
      faults.sensor.num = 0;
      faults.sensor.status = false;  
    }  
    //update LCD
 }

 if (fault == INVALID){
    faults.readingInvalid.num++;
    if (faults.readingInvalid.num >= allowedInvalidReadings){
      faults.readingInvalid.status = true;
    }
    if (override){
      faults.readingInvalid.num = 0;
      faults.readingInvalid.status = false;  
    }  
    //update LCD
 }

 if (fault == STALE){
    faults.readingStale.num++;
    if (faults.readingStale.num >= allowedStaleReadings){
      faults.readingStale.status = true;
    }
    if (override){
      faults.readingStale.num = 0;
      faults.readingStale.status = false;  
    }  
    //update LCD
 }

 if (fault == STORAGE){
    faults.storage.num++;
    if (faults.storage.num >= allowedStorageFaults){
      faults.storage.status = true;
    }
    if (override){
      faults.storage.num = 0;
      faults.storage.status = false;  
    }  
    //update LCD
 }
 
 if (fault == WIFI){
    faults.wifi.num++;
    if (faults.wifi.num >= allowedWifiFaults){
      faults.wifi.status = true;
    }
    if (override){
      faults.wifi.num = 0;
      faults.wifi.status = false;  
    }  
    //update LCD
 }
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

  /* ============================================================
  // BACKUP OPTION 2: Flat JSON (no subfields, matches CSV_HEADER keys)
  // To use: comment out the ORIGINAL loop above and uncomment this block.
  // ============================================================
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