#ifndef LOCAL_COMMUNICATION_H
#define LOCAL_COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

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

// ============================================================
// UI STATE
// ============================================================

enum DisplayScreen {
  PAGE_DASHBOARD = 0, // Airy 2x2 gas grid + borderless Temp/Hum + minimal header/banner
  SYSTEM_PAGE,        // Top-level Faults & LittleFS/NTP sync (L/R from Dashboard)
  BUZZER_PAGE         // Volume 0..10 (NVS saved) + Test Sound (Click from Dashboard)
};

struct UIState {
  DisplayScreen currentScreen = PAGE_DASHBOARD;
  int subRow                  = 0;     // Active row inside Buzzer page (0=Vol, 1=Test, 2=Back)
  bool alertMutedByJoystick   = false; // Silences buzzer until danger clears
};

// ============================================================
// PUBLIC MODULE API (Called by main.cpp)
// ============================================================

void initDisplayAndJoystickHardware();
void initBuzzerHardware();

void handleJoystickInput(unsigned long currentTime, const ProcessedData &data);
void renderDisplay(const ProcessedData &data);
void refreshBuzzerState(unsigned long currentTime);

#endif