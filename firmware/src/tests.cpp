#include "tests.h"
#include "local_communication.h"
#include "storage.h"
#include <WiFi.h>
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
// MODULE TEST STATE
// ============================================================
static bool uptimeLogEnabled        = false;
static unsigned long lastUptimeLog  = 0;
static bool simulatedWifiDropped    = false;

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
// SIMULATION & VERIFICATION COMMANDS (RTM R1, R3, R4, R6, R7, R8)
// ============================================================

// [1] R1 & T1.1: Inject valid mock readings into displayData & loggedData
static void testInjectValidData() {
  unsigned long now = millis();
  displayData.windowEndTimestamp = now;
  displayData.coPPM              = { 12.5f, 15.0f, 5, true };
  displayData.tvocPPB            = { 85.0f, 95.0f, 5, true };
  displayData.eco2PPM            = { 480.0f, 510.0f, 5, true };
  displayData.temperatureC       = { 22.4f, 22.6f, 3, true };
  displayData.humidityPercent    = { 46.0f, 47.0f, 3, true };
  displayData.pm25UGM3           = { 9.2f, 10.0f, 5, true };
  loggedData = displayData;
  renderDisplay(displayData);
  Serial.println("[TEST R1/T1.1] Injected valid mock sensor data to LCD & loggedData.");
}

// [2] R8 & T1.2: Inject invalid/unreasonable readings (NaN & negative)
static void testInjectInvalidData() {
  displayData.coPPM    = { NAN, NAN, 0, false };
  displayData.pm25UGM3 = { -5.0f, -5.0f, 0, false };
  loggedData.coPPM     = displayData.coPPM;
  loggedData.pm25UGM3  = displayData.pm25UGM3;
  faultDetections(INVALID, false);
  renderDisplay(displayData);
  Serial.println("[TEST R8/T1.2] Injected invalid CO (NaN) & PM2.5 (-5.0) readings.");
}

// [3] R3: Toggle emergency Danger Alert (<= 2s response test)
static void testToggleDangerAlert() {
  dangerAlertActive  = !dangerAlertActive;
  activeAlertMessage = dangerAlertActive ? "HIGH CO (TEST)" : "Normal";
  renderDisplay(displayData);
  Serial.printf("[TEST R3] Danger Alert %s at %lu ms.\n",
                dangerAlertActive ? "ACTIVATED" : "CLEARED", millis());
}

// [4] R6 & T1.3: Toggle forced Wi-Fi disconnect / reconnect
static void testToggleWiFiDrop() {
  if (!simulatedWifiDropped) {
    simulatedWifiDropped = true;
    WiFi.disconnect();
    checkWiFiConnection();
    renderDisplay(displayData);
    Serial.println("[TEST R6/T1.3] Forced Wi-Fi DISCONNECT (check WiFi:DC & offline buffering).");
  } else {
    simulatedWifiDropped = false;
    WiFi.reconnect();
    Serial.println("[TEST R6/T1.3] Triggered Wi-Fi RECONNECT (watch unsent bytes drain).");
  }
}

// [5] R4 & R7: Toggle 60-second battery endurance & interval heartbeat log
static void testToggleUptimeHeartbeat() {
  uptimeLogEnabled = !uptimeLogEnabled;
  lastUptimeLog    = millis();
  Serial.printf("[TEST R4/R7] 60s Uptime Heartbeat %s.\n", uptimeLogEnabled ? "ENABLED" : "DISABLED");
}

// ============================================================
// MAIN TEST RUNNER (Called in loop() when ENABLE_VERIFICATION_TESTS == 1)
// ============================================================
void runTestCommands() {
  // Periodic 60-second uptime log for R4/R7 endurance testing (when toggled on with '5')
  if (uptimeLogEnabled && (millis() - lastUptimeLog >= 60000UL)) {
    lastUptimeLog = millis();
    Serial.printf("[TEST R4/R7] Uptime: %lu min | Entries: %u | Unsent: %u | WiFi: %s\n",
                  lastUptimeLog / 60000UL, totalLogEntries, unsentLogEntries,
                  wifiConnected ? "OK" : "DC");
  }

  if (!Serial.available()) return;
  int cmd = tolower((unsigned char)Serial.read());

  switch (cmd) {
    case 'd': dumpActiveLogFile();       break;
    case 'r': resetAllStorageLogs();     break;
    case 's': printStorageSyncStatus();  break;
    case 'p': printDebugInformation();   break;
    case 'f': faultDetections(STORAGE, false); Serial.println("[TEST R8] Storage fault injected."); break;
    case 'c':
      faultDetections(SENSOR, true);
      faultDetections(INVALID, true);
      faultDetections(STALE, true);
      faultDetections(STORAGE, true);
      faultDetections(WIFI, true);
      dangerAlertActive  = false;
      activeAlertMessage = "Normal";
      renderDisplay(displayData);
      Serial.println("[TEST R8] Cleared all faults and reset danger alert.");
      break;
    case '1': testInjectValidData();       break;
    case '2': testInjectInvalidData();     break;
    case '3': testToggleDangerAlert();     break;
    case '4': testToggleWiFiDrop();        break;
    case '5': testToggleUptimeHeartbeat(); break;
    default: break;
  }
}