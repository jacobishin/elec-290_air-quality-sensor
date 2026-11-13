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
#include <time.h>

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
bool timeSyncedThisBoot       = false;
unsigned long bootEpochSec    = 0;

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
void readSensors(unsigned long currentTime);
void acquireMQ7(unsigned long currentTime);
void acquireSGP30(unsigned long currentTime);
void acquireTempHumidity(unsigned long currentTime);
void acquirePM25(unsigned long currentTime);


// Data processing & fault detection

// the general idea is that the sensors will take an average reading every 5 seconds and display it, 
// then take the average of 60 seconds and save and upload it every 60 seconds. each sensor has 
// its own sampling interval. for the average readings every 5 seconds, 
// it should remove the highest and lowest reading, then take the average of whats left


// Decision logic


// Data Storage
const char getActiveLogFilePath();


// Communication


// ============================================================
/* SETUP */
// ============================================================

void setup() {
  Serial.begin(115200);
  initializeSystem();
  Serial.println("Harmful Gas Monitor Started\n");
}

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

void initializeSystem() {
  Serial.println("Initializing system..."); // for debug purposes
  initializeAlerts();
  initializeSensors();
  initializeDisplay();
  initializeStorage();
  initializeCommunication();
  Serial.println("Initialization complete."); // for debug purposes
}

void initializeSensors(){}
void initializeDisplay(){}

void initializeStorage(){
  Serial.println("Initializing storage..."); // for debug purposes
  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS mount failed!");
    faults.storageFault = true;
    return;
  }

  syncPrefs.begin("gas_sync", false);

  activeLogFileIndex = syncPrefs.getUChar("active_file", 1);
  if (activeLogFileIndex != 1 && activeLogFileIndex != 2) {
    activeLogFileIndex = 1;
    syncPrefs.putUChar("active_file", 1);
  }
  lastUploadedByteOffset = syncPrefs.getUInt("upload_offset", 0);

  const char* activePath = getActiveLogFilePath();

  if (!LittleFS.exists(activePath)) {
    File logFile = LittleFS.open(activePath, "w");
    if (logFile) {
      logFile.println(CSV_HEADER);
      lastUploadedByteOffset = logFile.size(); // Skip header row
      logFile.close();
      syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
      Serial.printf("Created %s and set offset past header.\n", activePath);
    } else {
      faults.storageFault = true;
      return;
    }
  } else {
    File logFile = LittleFS.open(activePath, "r");
    if (logFile) {
      size_t currentSize = logFile.size();
      logFile.close();
      if (lastUploadedByteOffset > currentSize) {
        lastUploadedByteOffset = 0;
        syncPrefs.putUInt("upload_offset", 0);
      }
    }
  }

  faults.storageFault = false;
}

void initializeCommunication(){
  Serial.println("Initializing communication...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true); // automatically set the modem to sleep to conserve battery life
  WiFi.setAutoReconnect(true); // make wi-fi auto reconnect if it drops off
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Configure background NTP time synchronization (UTC / Unix Epoch)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

void initializeAlerts(){}


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
// 5. DATA STORAGE
// ============================================================
// the general idea for data storage is that the esp32 saves the 60-second 
// average readings directly onto its internal flash memory (LittleFS) so no 
// data is lost if the power cuts out or the wi-fi drops. to keep the flash 
// memory from filling up, it alternates between two 100 KB csv files. before 
// writing a new row, it checks if the current file is full, and if it is, it 
// wipes the other file, writes the csv headers, and switches over to logging there.
//
// it also keeps track of which file is active and the exact byte position of the 
// last row uploaded to firebase using NVS preferences, so it knows where to pick 
// back up after a reboot. while the device is offline, it just logs the time in 
// seconds since boot, then once wi-fi connects and gets the real time from NTP, 
// it goes back through the active csv file and updates those boot timestamps to 
// real unix timestamps. there are also serial commands ('D' and 'R') so you can 
// dump the active csv file to the serial monitor or wipe the logs to reset everything.

// ============================================================
// 6. COMMUNICATION
// ============================================================
// the communication section handles everything to do with sending information to 
// the user and the cloud, so this includes updating the lcd, managing the wi-fi 
// connection, uploading logs to firebase, and printing debug info to the serial 
// monitor. every 5 seconds, it checks the wi-fi status and refreshes the lcd to 
// show whether wi-fi is connected or disconnected, any active gas alerts or sensor 
// faults, and the 5-second trimmed averages for CO and TVOC.
//
// instead of sending live sensor readings straight to the cloud, it reads unsent 
// rows directly from the local csv file every 2 seconds when wi-fi is connected. 
// it converts each csv row into a json format (turning any NAN values into null) 
// and sends it to firebase, only moving the saved upload bookmark forward once 
// firebase confirms it received the data. wi-fi modem-sleep and auto-reconnect are 
// also turned on here to help save battery life.



