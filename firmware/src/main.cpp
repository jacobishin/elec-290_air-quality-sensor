/* 
ELEC 290 Section 002-01- Harmful gas sensor firmware

Hardware:
ESP32-WROVER
SGP30 - eCO2 and TVOC sensor
MQ-7 - CO sensor
DHT11 - temp and humid sensor
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

//sensor pins
const int MQ7_PIN  = NULL;
const int DHT_PIN = NULL;
const int SGP30_PIN = NULL;
const int PM25_PIN = NULL;

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

const int allowedSensorFaults = NULL;
const int allowedInvalidReadings = NULL;
const int allowedStaleReadings = NULL;
const int allowedStorageFaults = NULL;
const int allowedWifiFaults = NULL;

typedef enum faultenum {NONE, SENSOR, STORAGE, INVALID, STALE, WIFI}faultenum;

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

// Contains SensorTotal structs for each read in value (raw data), wrapped in a SensorAccumulator struct
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
  struct SensorFault sensor;
  struct InvalidReading readingInvalid;
  struct StaleReading readingStale;
  struct StorageFault storage;
  struct WifiFault wifi;
};

struct SensorFault {
  bool status = false;
  int num = 0;
};

struct InvalidReading {
  bool status = false;
  int num = 0;
};

struct StaleReading {
  bool status = false;
  int num = 0;
};

struct StorageFault {
  bool status = false;
  int num = 0;
};

struct WifiFault {
  bool status = false;
  int num = 0;
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
    
    faultDetections(STORAGE, false);
    //!!!!!!!!!!!!!!!!!!!!!!call for fault checking (when function is made)
    return;
  }

  syncPrefs.begin("gas_sync", false);

  activeLogFileIndex = syncPrefs.getUChar("active_file", 1);
  if (activeLogFileIndex != 1 && activeLogFileIndex != 2) {
    activeLogFileIndex = 1;
    syncPrefs.putUChar("active_file", 1);
  }
  lastUploadedByteOffset = syncPrefs.getUInt("upload_offset", 0);

  // const char* activePath = getActiveLogFilePath();

  if (!LittleFS.exists(activePath)) {
    File logFile = LittleFS.open(activePath, "w");
    if (logFile) {
      logFile.println(CSV_HEADER);
      lastUploadedByteOffset = logFile.size(); // Skip header row
      logFile.close();
      syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
      Serial.printf("Created %s and set offset past header.\n", activePath);
    } else {

      //this is CAUSING a fault
      faultDetections(STORAGE, false);
      //!!!!!!!!!!!!!!!!!!!!!!call for fault checking (when function is made)
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
  //this is CAUSING a fault
  faultDetections(STORAGE, false);
  //!!!!!!!!!!!!!!!!!!!!!!call for fault checking (when function is made)
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

/// @brief sensors are read into the rawData struct, and then fed into accumulateData()
///        to obtain an average for the raw data. 
/// @param rawData 
void readSensors(){
  //rawSensorData struct containing individual sensor data, which is updated every sensor read
  //sensorAccumulator struct which that raw data is added in sum, and count++ to later be used
  //in data processing
  
  //read in all data
  try {
    rawData.mq7.coPPM.value = analogRead(MQ7_PIN);
    

    if (!sgp.IAQmeasure()){ //IAQmeasure return value is a bool
      faults.readingInvalid.num++;
      faultDetections(STORAGE, false);
      throw ("SGP30 reading returned false");
    }
    rawData.sgp30.eco2PPM.value = sgp.eCO2; //i think this is in ppm
    rawData.sgp30.tvocPPB.value = sgp.TVOC; //i think this is in ppb
    
    float tempHumid = dht.readHumidity(); //in % - water vapor in air as a percent of max the air can hold at temp
    float tempTemper = dht.readTemperature(); //in degree C
    if (isnan(tempHumid)){
      throw ("Humidity sensor nan");
    }
    
    rawData.tempHum.humidityPercent.value = tempHumid;
    rawData.tempHum.temperatureC.value = tempTemper;
    if (isnan(tempTemper)){
      throw ("Temperature sensor nan");
    }
    

  }
  catch (const char* msg){
    //!!!!!!!!!!!!!!!!!this should later do something with fault management
    Serial.print(msg);
  } 


  
}

/// @brief reads in data from each sensor, storing as a float, then adds the value to
///        its appropriate accumulator. If it is the highest that was seen, in the 
///        last period, also update the maximum value.

///        This is simply reading in values and accumulating it into categories.
///        processData will have the average values that get displayed and stored
void accumulateData(){
  //safetyAccumulators is for 5 second display update
  //accumulators is for 60 second file write
  float readInData = rawData.mq7.coPPM.value;

  safetyAccumulators.coPPM.sum += readInData;
  accumulators.coPPM.sum += readInData;
  if (isMax(readInData, safetyAccumulators.coPPM.sum)){
    safetyAccumulators.coPPM.maximum = readInData;
  }
  if (isMax(readInData, accumulators.coPPM.sum)){
    accumulators.coPPM.maximum = readInData;
  }

  readInData = rawData.sgp30.eco2PPM.value;

  safetyAccumulators.eco2PPM.sum+=readInData;
  accumulators.eco2PPM.sum+=readInData;
  if (isMax(readInData, safetyAccumulators.eco2PPM.sum)){
    safetyAccumulators.eco2PPM.maximum = readInData;
  }
  if (isMax(readInData, accumulators.eco2PPM.sum)){
    accumulators.eco2PPM.maximum=readInData;
  }

  readInData = rawData.sgp30.tvocPPB.value;

  safetyAccumulators.tvocPPB.sum+=readInData;
  accumulators.tvocPPB.sum+=readInData;
  if (isMax(readInData, safetyAccumulators.tvocPPB.sum)){
    safetyAccumulators.tvocPPB.maximum = readInData;
  }
  if (isMax(readInData, accumulators.tvocPPB.sum)){
    accumulators.tvocPPB.maximum=readInData;
  }

  readInData = rawData.pm25.concentration.value;

  safetyAccumulators.pm25UGM3.sum+=readInData;
  accumulators.pm25UGM3.sum+=readInData;
  if (isMax(readInData, safetyAccumulators.pm25UGM3.sum)){
    safetyAccumulators.pm25UGM3.maximum = readInData;
  }
  if (isMax(readInData, accumulators.pm25UGM3.sum)){
    accumulators.pm25UGM3.maximum=readInData;
  }

  readInData = rawData.tempHum.humidityPercent.value;

  safetyAccumulators.humidityPercent.sum+=readInData;
  accumulators.humidityPercent.sum+=readInData;
  if (isMax(readInData, safetyAccumulators.humidityPercent.sum)){
    safetyAccumulators.humidityPercent.maximum = readInData;
  }
  if (isMax(readInData, accumulators.humidityPercent.sum)){
    accumulators.humidityPercent.maximum=readInData;
  }

  readInData = rawData.tempHum.temperatureC.value;

  safetyAccumulators.temperatureC.sum+=readInData;
  accumulators.temperatureC.sum+=readInData;
  if (isMax(readInData, safetyAccumulators.temperatureC.sum)){
    safetyAccumulators.temperatureC.maximum = readInData;
  }
  if (isMax(readInData, accumulators.temperatureC.sum)){
    accumulators.temperatureC.maximum=readInData;
  }
}



/// @brief returns true if val1 > val2 
/// @param val1 
/// @param val2 
/// @return bool
bool isMax(int val1, int val2){
  return (val1 > val2) ? true: false;
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
    //add windowendtimestamp
    
    //maybe call storedata here?
  }
    //it will also update the screen data, so not in an else
    displayData.coPPM.average=safetyAccumulators.coPPM.sum/safetyAccumulators.coPPM.count;
    displayData.coPPM.maximum=safetyAccumulators.coPPM.maximum;
  
    displayData.eco2PPM.average=safetyAccumulators.eco2PPM.sum/safetyAccumulators.eco2PPM.count;
    displayData.eco2PPM.maximum=safetyAccumulators.eco2PPM.maximum;
  
    displayData.humidityPercent.average=safetyAccumulators.humidityPercent.sum/safetyAccumulators.humidityPercent.count;
    displayData.humidityPercent.maximum=safetyAccumulators.humidityPercent.maximum;
  
    displayData.pm25UGM3.average=safetyAccumulators.pm25UGM3.sum/safetyAccumulators.pm25UGM3.count;
    displayData.pm25UGM3.maximum=safetyAccumulators.pm25UGM3.maximum;
  
    displayData.temperatureC.average=safetyAccumulators.temperatureC.sum/safetyAccumulators.temperatureC.count;
    displayData.temperatureC.maximum=safetyAccumulators.temperatureC.maximum;

    displayData.tvocPPB.average=safetyAccumulators.tvocPPB.sum/safetyAccumulators.tvocPPB.count;
    displayData.tvocPPB.maximum=safetyAccumulators.tvocPPB.maximum;

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



