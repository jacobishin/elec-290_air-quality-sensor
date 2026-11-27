#include "tests.h"
#include "local_communication.h"
#include "storage.h"
#include <LittleFS.h>
#include <ctype.h>

// ============================================================
// EXTERNAL REFERENCES FROM MAIN.CPP
// ============================================================
extern bool dangerAlertActive;
extern String activeAlertMessage;
extern ProcessedData displayData;
extern ProcessedData loggedData;
extern FaultStatus faults;

extern int faultDetections(faultenum fault, bool override);

// ============================================================
// STORAGE & DIAGNOSTIC COMMANDS
// ============================================================

// Prints every line of active CSV file on flash to the Serial Monitor (RTM R5)
void dumpActiveLogFile() {
  const char* path = getLogFilePath(true);
  Serial.printf("\n--- DUMPING ACTIVE LOG: %s (%u bytes) ---\n", path, (unsigned int)currentLogFileSize);
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("ERROR: Could not open active log file.");
    return;
  }
  uint8_t buf[128];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n > 0) Serial.write(buf, n);
  }
  f.close();
  Serial.println("--- END OF FILE ---\n");
}

// Wipes both CSV files and resets NVS bookmark back to file 1 (RTM R5)
void resetAllStorageLogs() {
  LittleFS.remove(LOG_FILE_1_PATH);
  LittleFS.remove(LOG_FILE_2_PATH);
  initNewLogFile(1);
  Serial.println("[TEST R5] Wiped both CSV log files and reset NVS state to File 1.");
}

// Prints current file system and Firebase upload bookmark (RTM R5/R6)
void printStorageSyncStatus() {
  Serial.println("\n--- STORAGE & SYNC STATUS ---");
  Serial.printf("Active File      : %s (Index %u)\n", getLogFilePath(true), activeLogFileIndex);
  Serial.printf("Entries          : %u total (%u unsent)\n", totalLogEntries, unsentLogEntries);
  Serial.printf("File Size        : %u bytes (offset %u B)\n",
                (unsigned int)currentLogFileSize, (unsigned int)lastUploadedByteOffset);
  Serial.printf("Wi-Fi / NTP Sync : Wi-Fi %s | NTP %s\n",
                wifiConnected ? "CONNECTED" : "DISCONNECTED",
                timeSyncedThisBoot ? "SYNCED" : "UNSYNCED");
  Serial.println("-----------------------------\n");
}

// Prints a summary of the latest logged sensor averages, alert status, and fault counts (RTM R7)
void printDebugInformation() {
  Serial.println();
  Serial.printf("===== LOG WINDOW SUMMARY (Active File: %s) =====\n", getLogFilePath(true));
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

// ============================================================
// SERIAL DIAGNOSTIC RUNNER (Called in loop() when ENABLE_VERIFICATION_TESTS == 1)
// ============================================================
void runTestCommands() {
  if (!Serial.available()) return;
  int cmd = tolower((unsigned char)Serial.read());

  switch (cmd) {
    case 'd': dumpActiveLogFile();       break;
    case 'r': resetAllStorageLogs();     break;
    case 's': printStorageSyncStatus();  break;
    case 'p': printDebugInformation();   break;
    case 'f': faultDetections(STORAGE, false); Serial.println("[DIAG] Storage fault injected."); break;
    case 'c':
      faultDetections(SENSOR, true);
      faultDetections(INVALID, true);
      faultDetections(STALE, true);
      faultDetections(STORAGE, true);
      faultDetections(WIFI, true);
      dangerAlertActive  = false;
      activeAlertMessage = "Normal";
      renderDisplay(displayData);
      Serial.println("[DIAG] Cleared all faults and reset danger alert.");
      break;
    default: break;
  }
}