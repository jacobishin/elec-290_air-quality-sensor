#include "tests.h"
#include <LittleFS.h>
#include <Preferences.h>

// ============================================================
// FROM MAIN.CPP
// ============================================================
typedef enum faultenum {NONE, SENSOR, STORAGE, INVALID, STALE, WIFI} faultenum;

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

struct SensorFault     { bool status = false; int num = 0; };
struct InvalidReading  { bool status = false; int num = 0; };
struct StaleReading    { bool status = false; int num = 0; };
struct StorageFault    { bool status = false; int num = 0; };
struct WifiFault       { bool status = false; int num = 0; };

struct FaultStatus {
  struct SensorFault sensor;
  struct InvalidReading readingInvalid;
  struct StaleReading readingStale;
  struct StorageFault storage;
  struct WifiFault wifi;
};

extern const char* LOG_FILE_1_PATH;
extern const char* LOG_FILE_2_PATH;
extern uint8_t activeLogFileIndex;
extern size_t lastUploadedByteOffset;
extern size_t currentLogFileSize;
extern bool wifiConnected;
extern bool timeSyncedThisBoot;
extern String activeAlertMessage;
extern ProcessedData loggedData;
extern FaultStatus faults;

extern const char* getLogFilePath(bool active = true);
extern bool initNewLogFile(uint8_t fileIndex);
extern int faultDetections(faultenum fault, bool override);

// ============================================================
// TESTING & DEBUG FUNCTIONS
// ============================================================

// Prints every line of CSV file on flash to the Serial Monitor
void dumpActiveLogFile() {
  const char* path = getLogFilePath(true);
  Serial.printf("\n--- DUMPING ACTIVE LOG: %s (%u bytes) ---\n", path, (unsigned int)currentLogFileSize);
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("ERROR: Could not open active log file.");
    return;
  }
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
  Serial.println("--- END OF FILE ---\n");
}

// Wipes both CSV files and resets NVS bookmark back to file 1
void resetAllStorageLogs() {
  LittleFS.remove(LOG_FILE_1_PATH);
  LittleFS.remove(LOG_FILE_2_PATH);
  initNewLogFile(1);
  Serial.println("[TEST] Wiped both CSV log files and reset NVS state to File 1.");
}

// Prints current file system and Firebase upload bookmark
void printStorageSyncStatus() {
  Serial.println("\n--- STORAGE & SYNC STATUS ---");
  Serial.printf("Active File      : %s (Index %u)\n", getLogFilePath(true), activeLogFileIndex);
  Serial.printf("File Size        : %u bytes\n", (unsigned int)currentLogFileSize);
  Serial.printf("Uploaded Offset  : %u bytes (%u bytes unsent)\n",
                (unsigned int)lastUploadedByteOffset,
                (currentLogFileSize > lastUploadedByteOffset) ? (unsigned int)(currentLogFileSize - lastUploadedByteOffset) : 0);
  Serial.printf("Wi-Fi / NTP Sync : Wi-Fi %s | NTP %s\n",
                wifiConnected ? "CONNECTED" : "DISCONNECTED",
                timeSyncedThisBoot ? "SYNCED" : "UNSYNCED");
  Serial.println("-----------------------------\n");
}

// Prints a summary of the 60-second sensor averages, alert status, and fault counts to the serial monitor
void printDebugInformation() {
  Serial.println();
  Serial.printf("===== 60s WINDOW SUMMARY (Active File: %s) =====\n", getLogFilePath(true));
  Serial.printf("Alert Status: %s | Time Synced: %s\n",
                activeAlertMessage.c_str(), timeSyncedThisBoot ? "YES" : "NO");
  Serial.printf("CO Avg: %.2f ppm | TVOC Avg: %.2f ppb | eCO2 Avg: %.2f ppm\n",
                loggedData.coPPM.average, loggedData.tvocPPB.average, loggedData.eco2PPM.average);
  Serial.printf("Temp Avg: %.2f C | Hum Avg: %.2f %% | PM2.5 Avg: %.2f ug/m3\n",
                loggedData.temperatureC.average, loggedData.humidityPercent.average, loggedData.pm25UGM3.average);
  Serial.printf("Fault Counts -> Sensor: %d | Storage: %d | Wi-Fi: %d\n",
                faults.sensor.num, faults.storage.num, faults.wifi.num);
  Serial.println("============================================================");
}

// Listens for single-letter test commands over USB Serial ('d', 'r', 's', 'p', 'f', 'c')
void runTestCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read() | 0x20; // Convert uppercase to lowercase

  switch (cmd) {
    case 'd': dumpActiveLogFile();      break;
    case 'r': resetAllStorageLogs();    break;
    case 's': printStorageSyncStatus(); break;
    case 'p': printDebugInformation();  break;
    case 'f': faultDetections(STORAGE, false); break;
    case 'c':
      faultDetections(SENSOR, true);
      faultDetections(INVALID, true);
      faultDetections(STALE, true);
      faultDetections(STORAGE, true);
      faultDetections(WIFI, true);
      break;
    default: break;
  }
}