#ifndef LOCAL_COMMUNICATION_H
#define LOCAL_COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ============================================================
// FROM MAIN.CPPs
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
// UI & JOYSTICK STATE
// ============================================================

enum DisplayPage {
  PAGE_OVERVIEW = 0, // 2x3 grid of all 6 sensors + top Wi-Fi/Alert bar
  PAGE_DETAIL   = 1, // Single sensor large readout + threshold info
  PAGE_SYSTEM   = 2, // Fault counters, storage file info, and sync status
  PAGE_COUNT    = 3
};

struct UIState {
  DisplayPage currentPage   = PAGE_OVERVIEW;
  uint8_t selectedSensor    = 0;     // 0=CO, 1=TVOC, 2=eCO2, 3=Temp, 4=Hum, 5=PM2.5
  bool needsFullRedraw      = true;  // Redraws static frames only on page change
  bool alertMutedByJoystick = false; // Pressing joystick button mutes buzzer for current alert
  unsigned long lastJoyPoll = 0;
  unsigned long lastJoyMove = 0;
};

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void initDisplayAndJoystickHardware();
void initBuzzerHardware();

void handleJoystickInput(unsigned long currentTime, const ProcessedData &data);
void renderDisplay(const ProcessedData &data);
void refreshBuzzerState(unsigned long currentTime);

#endif