#include "storage.h"
#include <math.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Preferences.h>

// ============================================================
// FROM MAIN.CPP
// ============================================================
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* FIREBASE_URL;
extern const char* FIREBASE_AUTH_TOKEN;
extern const unsigned long TRANSMIT_INTERVAL;

extern bool dangerAlertActive;
extern FaultStatus faults;

extern int faultDetections(faultenum fault, bool override);

// ============================================================
// STORAGE CONFIGURATION
// ============================================================
const char* LOG_FILE_1_PATH = "/sensor_log_1.csv";
const char* LOG_FILE_2_PATH = "/sensor_log_2.csv";

static const size_t MAX_LOG_FILE_SIZE = 65536; // 64 KB x 2 fits safely inside 192 KB min_spiffs.csv LittleFS

static const char* CSV_HEADER =
    "timestamp,alert_active,"
    "co_avg,co_count,co_valid,"
    "tvoc_avg,tvoc_count,tvoc_valid,"
    "eco2_avg,eco2_count,eco2_valid,"
    "temp_avg,temp_count,temp_valid,"
    "hum_avg,hum_count,hum_valid,"
    "pm25_avg,pm25_count,pm25_valid";

// FreeRTOS IPC payloads (Core 1 owns LittleFS/NVS, Core 0 owns HTTPS)
struct UploadJob {
  char json[320];
  size_t nextOffset;
  uint8_t fileIndex;
};

struct UploadResult {
  bool success;
  size_t nextOffset;
  uint8_t fileIndex;
};

// ============================================================
// STATES
// ============================================================
static Preferences syncPrefs;
uint8_t activeLogFileIndex    = 1;
size_t lastUploadedByteOffset = 0;
size_t currentLogFileSize     = 0;
unsigned int totalLogEntries  = 0;
unsigned int unsentLogEntries = 0;
bool wifiConnected            = false;
bool timeSyncedThisBoot       = false;
static unsigned long bootEpochSec = 0;

static QueueHandle_t firebaseUploadQueue = nullptr;
static QueueHandle_t firebaseResultQueue = nullptr;
static bool uploadInFlight               = false;
static unsigned long lastTransmission    = 0;

// ============================================================
// LITTLEFS STORAGE
// ============================================================
const char* getLogFilePath(bool active) {
  return ((activeLogFileIndex == 1) == active) ? LOG_FILE_1_PATH : LOG_FILE_2_PATH;
}

static void recountLogEntries() {
  totalLogEntries  = 0;
  unsentLogEntries = 0;

  File f = LittleFS.open(getLogFilePath(), "r");
  if (!f) return;

  uint8_t buf[128];
  size_t pos = 0;
  unsigned int lineIndex = 0;
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    for (size_t i = 0; i < n; i++, pos++) {
      if (buf[i] == '\n') {
        if (lineIndex >= 1) { // Skip line 0 (CSV header)
          totalLogEntries++;
          if (pos >= lastUploadedByteOffset) unsentLogEntries++;
        }
        lineIndex++;
      }
    }
  }
  f.close();
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

  totalLogEntries  = 0;
  unsentLogEntries = 0;

  syncPrefs.putUChar("active_file", activeLogFileIndex);
  syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
  faultDetections(STORAGE, true);
  return true;
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
      recountLogEntries();
    } else {
      faultDetections(STORAGE, false);
      return;
    }
  }

  faultDetections(STORAGE, true);
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
  } else {
    totalLogEntries++;
    unsentLogEntries++;
  }
}

// ============================================================
// CSV to JSON to CLOUD
// ============================================================
static bool csvLineToJson(char* csvLine, char* jsonOut, size_t jsonOutSize) {
  if (strncmp(csvLine, "timestamp", 9) == 0 || strncmp(csvLine, "seconds,", 8) == 0) return false;

  char* tok[20];
  int n = 0;
  for (char* p = strtok(csvLine, ","); p && n < 20; p = strtok(nullptr, ",")) tok[n++] = p;
  if (n < 20) return false;

  unsigned long ts = strtoul(tok[0], nullptr, 10);
  if (ts < 1700000000UL && bootEpochSec > 0) {
    ts += bootEpochSec;
  }

  static const char* keys[6] = {"co", "tvoc", "eco2", "temp", "hum", "pm25"};
  int pos = snprintf(jsonOut, jsonOutSize, "{\"timestamp\":%lu", ts);
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

static bool sendJsonToFirebase(const char* jsonPayload) {
  if (strlen(FIREBASE_URL) == 0) {
    Serial.printf("Sent to firebase: %s\n", jsonPayload);
    return true;
  }

  // Static client, HTTPClient & URL on Core 0 enable TLS keep-alive reuse
  static WiFiClientSecure client;
  static HTTPClient http;
  static char fullUrl[256];
  static bool configured = false;
  if (!configured) {
    client.setInsecure();
    http.setReuse(true);
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    if (strlen(FIREBASE_AUTH_TOKEN) > 0) {
      snprintf(fullUrl, sizeof(fullUrl), "%s?auth=%s", FIREBASE_URL, FIREBASE_AUTH_TOKEN);
    } else {
      snprintf(fullUrl, sizeof(fullUrl), "%s", FIREBASE_URL);
    }
    configured = true;
  }

  if (!http.begin(client, fullUrl)) return false;

  http.addHeader("Content-Type", "application/json");
  int code = http.POST(reinterpret_cast<uint8_t*>(const_cast<char*>(jsonPayload)), strlen(jsonPayload));
  http.end();
  return (code >= 200 && code < 300);
}

static void firebaseUploadTask(void* param) {
  (void)param;
  UploadJob job;
  for (;;) {
    if (xQueueReceive(firebaseUploadQueue, &job, portMAX_DELAY) == pdTRUE) {
      bool ok = sendJsonToFirebase(job.json);
      UploadResult res = { ok, job.nextOffset, job.fileIndex };
      xQueueSend(firebaseResultQueue, &res, portMAX_DELAY);
    }
  }
}

static void transmitData() {
  if (uploadInFlight || !firebaseUploadQueue || faults.storage.status ||
      !wifiConnected || lastUploadedByteOffset >= currentLogFileSize) {
    return;
  }

  File logFile = LittleFS.open(getLogFilePath(), "r");
  if (!logFile) return;

  logFile.seek(lastUploadedByteOffset);
  char line[256];
  size_t len = logFile.readBytesUntil('\n', line, sizeof(line) - 1);
  line[len] = '\0';
  size_t nextOffset = logFile.position();
  logFile.close();

  // Trim trailing CR/spaces
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[--len] = '\0';
  }

  UploadJob job  = {};
  job.nextOffset = nextOffset;
  job.fileIndex  = activeLogFileIndex;

  if (len == 0 || !csvLineToJson(line, job.json, sizeof(job.json))) {
    lastUploadedByteOffset = nextOffset;
    syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
    recountLogEntries();
    return;
  }

  if (xQueueSend(firebaseUploadQueue, &job, 0) == pdTRUE) {
    uploadInFlight = true;
  }
}

// ============================================================
// PUBLIC COMMUNICATION API
// ============================================================
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

void pollFirebaseSync(unsigned long currentTime) {
  if (uploadInFlight && firebaseResultQueue) {
    UploadResult res;
    if (xQueueReceive(firebaseResultQueue, &res, 0) == pdTRUE) {
      uploadInFlight   = false;
      lastTransmission = currentTime;
      if (res.success && res.fileIndex == activeLogFileIndex) {
        lastUploadedByteOffset = res.nextOffset;
        syncPrefs.putUInt("upload_offset", lastUploadedByteOffset);
        if (unsentLogEntries > 0) unsentLogEntries--;
      }
    }
  }

  if (!uploadInFlight && wifiConnected && (currentTime - lastTransmission >= TRANSMIT_INTERVAL)) {
    lastTransmission = currentTime;
    transmitData();
  }
}
