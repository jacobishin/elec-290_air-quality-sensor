#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <math.h>
#include "local_communication.h"

// ============================================================
// MQ-7 OPERATING MODE & CALIBRATION SWITCHES
// ============================================================

// Single-line switch:
// 0 = Regular/Direct mode (constant 5V, no voltage divider, 0.5s polling, no warmup delay)
// 1 = Analog Heater-Cycle mode (S8050 PWM on GPIO 14 + 10k/20k divider on GPIO 36, 150s cycle)
#define MQ7_USE_HEATER_CIRCUIT 0

// ============================================================
// DATA STRUCTURES
// ============================================================

struct SensorReading {
  float value             = NAN;
  bool valid              = false;
  unsigned long timestamp = 0;
};

struct RawSensorData {
  SensorReading coPPM;
  SensorReading tvocPPB;
  SensorReading eco2PPM;
  SensorReading temperatureC;
  SensorReading humidityPercent;
  SensorReading pm25UGM3;
};

struct SensorTotal {
  float sum          = 0.0f;
  float maximum      = -INFINITY;
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

// ============================================================
// SHARED SENSOR STATE & MODULE API
// ============================================================

extern RawSensorData rawData;
extern SensorAccumulators safetyAccumulators; // Running 5-second accumulators (LCD & Buzzer)
extern SensorAccumulators accumulators;       // Running 5-minute accumulators (CSV & Firebase)
extern const unsigned long MQ7_STALE_TIMEOUT_MS;

void initializeSensors();
void readSensors(unsigned long currentTime);
void clearAccumulators(SensorAccumulators &acc);

bool isCOInWarmup();
bool isSGP30InWarmup();

#endif
