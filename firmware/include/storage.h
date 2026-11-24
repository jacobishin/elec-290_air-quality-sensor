#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include "local_communication.h"

// Shared storage & network state (referenced by local_communication.cpp & tests.cpp)
extern const char* LOG_FILE_1_PATH;
extern const char* LOG_FILE_2_PATH;
extern uint8_t activeLogFileIndex;
extern size_t lastUploadedByteOffset;
extern size_t currentLogFileSize;
extern bool wifiConnected;
extern bool timeSyncedThisBoot;

// Section 1: Init
void initializeStorage();
void initializeCommunication();

// Section 5: LittleFS CSV Ping-Pong Storage
const char* getLogFilePath(bool active = true);
bool initNewLogFile(uint8_t fileIndex);
void storeData(const ProcessedData &data);

// Section 6: Wi-Fi / NTP & Core 0 Async Firebase Sync
void checkWiFiConnection();
void pollFirebaseSync(unsigned long currentTime);

#endif
