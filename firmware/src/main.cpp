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


// Air quality thresholds
const float C0_THRESHOLD_PPM = NULL;
const float TVOC_THRESHOLD_PPB = NULL;
const float ECO2_THRESHOLD_PPM = NULL;
const float PM25_THRESHOLD_UGM3 = NULL;

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

// Running total during 60s window
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
// Finalized 60-second window summary for CSV storage & Firebase
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
int processData(struct SensorAccumulators){
  //compare sensor readings to threshold values 


  //return enum error report if error occuring
}

int faultDetections(struct FaultStatus* listOfFaults){
  /*
    if faults are detected:
    listOfFaults->sensorFault = true;
    listOfFaults->invalidReading = true;
    listOfFaults-> staleReading = true;
    listOfFaults-> storageFault = false;
    listOfFaults->wifiFault = false;

    
  */
}
// ============================================================
/* DECISION LOGIC */
// ============================================================

// ============================================================
/* COMMUNICATION */
// ============================================================

// ============================================================
/* DATA STORAGE */
// ============================================================

