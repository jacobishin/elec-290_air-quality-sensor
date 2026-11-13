/* 
ELEC 290 Section 002-01- Harmful gas sensor firmware

Hardware:
ESP32-WROVER
SGP30
MQ-7
DHT11
PMS7003
Buzzer
LCD
*/

#include <Arduino.h>
#include <math.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <LittleFS.h>
#include <Preferences.h>

#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h> // not sure if this is the right library

// ============================================================
/* CONFIGURATION */
// ============================================================

// Wi-Fi & database credentials
const char* WIFI_SSID = "WIFI_NAME";
const char* WIFI_PASSWORD = "WIFI_PASSWORD";

const char* FIREBASE_URL =""; // idk if itll be a url or an endpoint
const char* FIREBASE_AUTH_TOKEN = ""; // idk if actual authentication is necessary

// Pin assignments
const int I2C_SDA_PIN = NULL;
const int I2C_SCL_PIN = NULL;

const int PMS_RX_PIN = NULL;
const int PMS_TX_PIN = NULL;

const int MQ7_PIN  = NULL;
const int DHT_PIN = NULL;
const int BUZZER_PIN = NULL;

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


// File system constants
const char* LOG_FILE_1_PATH    = "/sensor_log_1.csv";
const char* LOG_FILE_2_PATH    = "/sensor_log_2.csv";
const size_t MAX_LOG_FILE_SIZE = 100000;

const char* CSV_HEADER = "timestamp_ms,alert_active,"
                         "co_avg,co_max,co_count,co_valid,"
                         "tvoc_avg,tvoc_max,tvoc_count,tvoc_valid,"
                         "eco2_avg,eco2_max,eco2_count,eco2_valid,"
                         "temp_avg,temp_max,temp_count,temp_valid,"
                         "hum_avg,hum_max,hum_count,hum_valid,"
                         "pm25_avg,pm25_max,pm25_count,pm25_valid";


// Air quality thresholds
const float C0_THRESHOLD1_PPM = NULL;
const float TVOC_THRESHOLD1_PPB = NULL;
const float ECO2_THRESHOLD1_PPM = NULL;
const float PM25_THRESHOLD1_UGM3 = NULL;

const float C0_THRESHOLD2_PPM = NULL;
const float TVOC_THRESHOLD2_PPB = NULL;
const float ECO2_THRESHOLD2_PPM = NULL;
const float PM25_THRESHOLD2_UGM3 = NULL;


// ============================================================
/* DATA STRUCTURES */
// ============================================================

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

// Processed data structs
// Finalized 60-second and 5-second window
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
};

struct FaultStatus {
  bool sensorFault    = false;
  bool invalidReading = false;
  bool staleReading   = false;
  bool storageFault   = false;
  bool wifiFault      = false;
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
DHT dht(DHT_PIN, DHT11);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// byte offset for last cloud upload in csv file
Preferences syncPrefs;
uint8_t activeLogFileIndex    = 1;
size_t lastUploadedByteOffset = 0;

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


// Data acquisition


// Data processing & fault detection

// the general idea is that the sensors will take an average reading every 5 seconds and display it, 
// then take the average of 60 seconds and save and upload it every 60 seconds. each sensor has 
// its own sampling interval. for the average readings every 5 seconds, it should remove the highest and lowest reading, then take the average of whats left


// Decision logic


// Communication


// Data Storage


// ============================================================
/* SETUP */
// ============================================================



// ============================================================
/* MAIN LOOP */
// ============================================================
void loop() {
    // Data acquisitiion

    // data processing and fault detection

    // Decision logic

    // Communicaton

    // Data storage

}

// ============================================================
/* INITIALIZATION*/
// ============================================================

// ============================================================
/* DATA ACQUISITION */
// ============================================================

// ============================================================
/* DATA PROCESSING & FAULT DETECTION */
// ============================================================

// ============================================================
/* DECISION LOGIC */
// ============================================================

// ============================================================
/* COMMUNICATION */
// ============================================================

// ============================================================
/* DATA STORAGE */
// ============================================================

