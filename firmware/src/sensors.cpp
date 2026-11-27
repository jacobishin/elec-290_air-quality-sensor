#include "sensors.h"
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include <DHT.h>

// ============================================================
// FROM MAIN.CPP
// ============================================================
extern const int I2C_SDA_SGP30_PIN;
extern const int I2C_SCL_SGP30_PIN;
extern const int PMS_TX_PIN;
extern const int PMS_RX_PIN;
extern const int MQ7_PIN;
extern const int DHT_PIN;

extern int faultDetections(faultenum fault, bool override);

// ============================================================
// MODE-PAIRED HARDWARE & SAMPLING CONSTANTS
// ============================================================
static const float MQ7_RL_OHMS = 10000.0f; // Onboard 10 kOhm load resistor on 4-pin module

#if MQ7_USE_HEATER_CIRCUIT
  static const int           MQ7_HEATER_PIN    = 14;       // 470 Ohm -> S8050 Base
  static const int           MQ7_PWM_CH        = 0;        // Safe: buzzer uses LEDC Channel 4
  static const float         MQ7_DIVIDER_SCALE = 1.5f;     // Undoes 10 kOhm / 20 kOhm divider
  static const float         MQ7_SUPPLY_VOLTS  = 5.0f;     // MQ-7 module supply voltage
  static const float         MQ7_R0_OHMS       = 10000.0f; // TODO: Calibrate from clean-air Rs at 149.5s
  static const unsigned long MQ7_CYCLE_MS      = 150000UL; // 60s purge + 90s sense
  extern const unsigned long MQ7_STALE_TIMEOUT_MS = 160000UL;
#else
  static const float         MQ7_DIVIDER_SCALE = 1.0f;     // No voltage divider in direct mode
  static const float         MQ7_SUPPLY_VOLTS  = 5.0f;     // MQ-7 module supply voltage
  static const float         MQ7_R0_OHMS       = 10000.0f; // TODO: Calibrate from clean-air Rs in direct mode
  extern const unsigned long MQ7_STALE_TIMEOUT_MS = 1500UL;
#endif

static const unsigned long MQ7_SAMPLE_INTERVAL     = 500;    // 0.5s ADC polling in direct mode
static const unsigned long SGP30_SAMPLE_INTERVAL   = 1000;   // SGP30 datasheet requires 1 Hz for baseline
static const unsigned int  SGP30_WARMUP_READINGS   = 20;     // Discard initial 20 null readings (0 ppb / 400 ppm)
static const unsigned long TEMPHUM_SAMPLE_INTERVAL = 2000;   // DHT22 hardware minimum is 2000 ms
static const unsigned long PM25_STALE_TIMEOUT_MS   = 6000UL; // PMS7003 quiet-air frame interval is up to ~2.3s

// ============================================================
// MODULE STATE
// ============================================================
RawSensorData rawData = {};
SensorAccumulators safetyAccumulators = {};
SensorAccumulators accumulators = {};

static Adafruit_SGP30 sgp;
static DHT* dhtPtr              = nullptr;
static bool sgp30Initialized    = false;
static unsigned int sgp30Count  = 0;

static unsigned long lastMQ7Read     = 0;
static unsigned long lastSGP30Read   = 0;
static unsigned long lastTempHumRead = 0;

static uint16_t eCO2_base = 40332;
static uint16_t TVOC_base = 43346;
static uint16_t sout = 0;

// ============================================================
// ACCUMULATOR HELPERS
// ============================================================
static void addSample(SensorTotal &safety, SensorTotal &logged, float val) {
  safety.sum += val;
  safety.count++;
  if (val > safety.maximum) safety.maximum = val;

  logged.sum += val;
  logged.count++;
  if (val > logged.maximum) logged.maximum = val;
}

static void resetTotal(SensorTotal &t) {
  t.sum     = 0.0f;
  t.maximum = -INFINITY;
  t.count   = 0;
}

void clearAccumulators(SensorAccumulators &acc) {
  resetTotal(acc.coPPM);
  resetTotal(acc.tvocPPB);
  resetTotal(acc.eco2PPM);
  resetTotal(acc.temperatureC);
  resetTotal(acc.humidityPercent);
  resetTotal(acc.pm25UGM3);
}

// ============================================================
// MQ-7 DRIVER (Direct and 150s Heater Cycle)
// ============================================================
#if MQ7_USE_HEATER_CIRCUIT
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    static void mq7HeaterPwmBegin()          { ledcAttachChannel(MQ7_HEATER_PIN, 1000, 8, MQ7_PWM_CH); }
    static void mq7HeaterPwmWrite(uint8_t d) { ledcWrite(MQ7_HEATER_PIN, d); }
  #else
    static void mq7HeaterPwmBegin() {
      ledcSetup(MQ7_PWM_CH, 1000, 8);
      ledcAttachPin(MQ7_HEATER_PIN, MQ7_PWM_CH);
    }
    static void mq7HeaterPwmWrite(uint8_t d) { ledcWrite(MQ7_PWM_CH, d); }
  #endif

  static bool mq7ShouldSampleNow(unsigned long now) {
    static bool initialized      = false;
    static bool sampledThisCycle = false;
    static int  lastDuty         = -1;

    if (!initialized) {
      mq7HeaterPwmBegin();
      initialized = true;
    }

    unsigned long cyclePos = now % MQ7_CYCLE_MS;
    if (cyclePos < 60000UL) {
      if (lastDuty != 255) { mq7HeaterPwmWrite(255); lastDuty = 255; } // 5.0V Purge
      sampledThisCycle = false;
      return false;
    } else if (cyclePos < 149500UL) {
      if (lastDuty != 22)  { mq7HeaterPwmWrite(22);  lastDuty = 22;  } // 1.4V RMS Sense
      return false;
    } else if (!sampledThisCycle) {
      mq7HeaterPwmWrite(255); // Solid GND for 5 ms so AO settles cleanly
      lastDuty = 255;
      delay(5);
      sampledThisCycle = true;
      return true;
    }
    return false;
  }
#endif

static bool readMQ7() {
  long adcSum = 0;
  for (int i = 0; i < 4; i++) adcSum += analogRead(MQ7_PIN);
  float rawADC = adcSum / 4.0f;

  float pinVoltage      = (rawADC / 4095.0f) * 3.3f;
  float actualAOVoltage = pinVoltage * MQ7_DIVIDER_SCALE;

  if (!isfinite(actualAOVoltage) || actualAOVoltage <= 0.01f || actualAOVoltage >= (MQ7_SUPPLY_VOLTS - 0.01f)) {
    rawData.coPPM.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  float H2PPM = pow(0.5, (15325.5-sout)/512);
  float rh2 = 323.636*(25.48617*pow(H2PPM, -0.75161));
  float rs    = MQ7_RL_OHMS * (MQ7_SUPPLY_VOLTS - actualAOVoltage) / actualAOVoltage;

  float rco = pow((1/rs)-(1/rh2)+(1/323.636),-1);
  float coPPM = 20.20766*pow(rco/323.636,-0.652758);

  if (!isfinite(coPPM) || coPPM < 0.0f) {
    rawData.coPPM.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.coPPM.value     = coPPM;
  rawData.coPPM.valid     = true;
  rawData.coPPM.timestamp = millis();
  addSample(safetyAccumulators.coPPM, accumulators.coPPM, coPPM);
  return true;
}

static void mq7Update(unsigned long currentTime) {
#if MQ7_USE_HEATER_CIRCUIT
  if (mq7ShouldSampleNow(currentTime)) {
    lastMQ7Read = currentTime;
    readMQ7();
  } else if (rawData.coPPM.timestamp > 0 &&
             (currentTime - rawData.coPPM.timestamp > MQ7_STALE_TIMEOUT_MS)) {
    if (rawData.coPPM.valid) {
      rawData.coPPM.valid = false;
      faultDetections(STALE, false);
    }
  }
#else
  if (currentTime - lastMQ7Read >= MQ7_SAMPLE_INTERVAL) {
    lastMQ7Read = currentTime;
    readMQ7();
  }
#endif
}

bool isCOInWarmup() {
#if MQ7_USE_HEATER_CIRCUIT
  return (rawData.coPPM.timestamp == 0 && millis() < MQ7_CYCLE_MS);
#else
  return false;
#endif
}

// ============================================================
// SGP30 DRIVER (1 Hz + 20-reading warm-up filter)
// ============================================================
static bool readSGP30() {
  if (!sgp30Initialized || !sgp.IAQmeasure()) {
    rawData.eco2PPM.valid = false;
    rawData.tvocPPB.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  // Keep calling IAQmeasure() at 1 Hz during warm-up, but discard the first 20 null readings
  if (sgp30Count < SGP30_WARMUP_READINGS) {
    sgp30Count++;
    rawData.eco2PPM.valid = false;
    rawData.tvocPPB.valid = false;
    return false;
  }

  sgp.IAQmeasureRaw();
  sout = sgp.rawH2;

  unsigned long now = millis();
  float eco2Val = static_cast<float>(sgp.eCO2);
  float tvocVal = static_cast<float>(sgp.TVOC);

  rawData.eco2PPM.value     = eco2Val;
  rawData.eco2PPM.valid     = true;
  rawData.eco2PPM.timestamp = now;
  addSample(safetyAccumulators.eco2PPM, accumulators.eco2PPM, eco2Val);

  rawData.tvocPPB.value     = tvocVal;
  rawData.tvocPPB.valid     = true;
  rawData.tvocPPB.timestamp = now;
  addSample(safetyAccumulators.tvocPPB, accumulators.tvocPPB, tvocVal);
  return true;
}

bool isSGP30InWarmup() {
  return sgp30Initialized &&
         (rawData.eco2PPM.timestamp == 0) &&
         (millis() < (SGP30_WARMUP_READINGS + 6UL) * SGP30_SAMPLE_INTERVAL);
}

// ============================================================
// PMS7003 NATIVE UART2 PARSER (32-byte 0x42 0x4D frame)
// ============================================================
static bool readPM25() {
  static uint8_t frame[32];
  static uint8_t idx = 0;

  bool gotValidFrame  = false;
  uint16_t latestPM25 = 0;

  while (Serial2.available() > 0) {
    uint8_t b = static_cast<uint8_t>(Serial2.read());

    if (idx == 0) {
      if (b == 0x42) frame[idx++] = b;
    } else if (idx == 1) {
      if (b == 0x4D)      frame[idx++] = b;
      else if (b == 0x42) { frame[0] = 0x42; idx = 1; }
      else                idx = 0;
    } else {
      frame[idx++] = b;
      if (idx == 4) {
        uint16_t frameLen = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
        if (frameLen != 28) idx = 0;
      } else if (idx == 32) {
        uint16_t sum = 0;
        for (int i = 0; i < 30; i++) sum += frame[i];
        uint16_t expected = (static_cast<uint16_t>(frame[30]) << 8) | frame[31];
        if (sum == expected) {
          latestPM25 = (static_cast<uint16_t>(frame[12]) << 8) | frame[13]; // Atmospheric PM2.5 (ug/m3)
          gotValidFrame = true;
        }
        idx = 0;
      }
    }
  }

  unsigned long now = millis();
  if (!gotValidFrame) {
    if (rawData.pm25UGM3.timestamp > 0 &&
        (now - rawData.pm25UGM3.timestamp > PM25_STALE_TIMEOUT_MS)) {
      if (rawData.pm25UGM3.valid) {
        rawData.pm25UGM3.valid = false;
        faultDetections(STALE, false);
      }
    }
    return false;
  }

  float pm25Val = static_cast<float>(latestPM25);
  if (!isfinite(pm25Val) || pm25Val < 0.0f || pm25Val > 1000.0f) {
    rawData.pm25UGM3.valid = false;
    faultDetections(INVALID, false);
    return false;
  }

  rawData.pm25UGM3.value     = pm25Val;
  rawData.pm25UGM3.valid     = true;
  rawData.pm25UGM3.timestamp = now;
  addSample(safetyAccumulators.pm25UGM3, accumulators.pm25UGM3, pm25Val);
  return true;
}

// ============================================================
// DHT22 DRIVER (2s interval)
// ============================================================
static void readTempHum() {
  if (!dhtPtr) return;
  unsigned long now = millis();

  float temp = dhtPtr->readTemperature();
  if (isnan(temp) || temp < 5.0f || temp > 80.0f) {
    rawData.temperatureC.valid = false;
    faultDetections(INVALID, false);
  } else {
    rawData.temperatureC.value     = temp;
    rawData.temperatureC.valid     = true;
    rawData.temperatureC.timestamp = now;
    addSample(safetyAccumulators.temperatureC, accumulators.temperatureC, temp);
  }

  float hum = dhtPtr->readHumidity();
  if (isnan(hum) || hum < 15.0f || hum > 100.0f) {
    rawData.humidityPercent.valid = false;
    faultDetections(INVALID, false);
  } else {
    rawData.humidityPercent.value     = hum;
    rawData.humidityPercent.valid     = true;
    rawData.humidityPercent.timestamp = now;
    addSample(safetyAccumulators.humidityPercent, accumulators.humidityPercent, hum);
  }
}

// ============================================================
// PUBLIC MODULE API
// ============================================================
void initializeSensors() {
  Serial.println("Initializing sensor pins and buses...");

  pinMode(MQ7_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ7_PIN, ADC_11db);
#if MQ7_USE_HEATER_CIRCUIT
  mq7ShouldSampleNow(millis()); // Start 60s 5V purge phase immediately at boot
#endif

  Wire.begin(I2C_SDA_SGP30_PIN, I2C_SCL_SGP30_PIN);

  static DHT dhtInstance(DHT_PIN, DHT22);
  dhtPtr = &dhtInstance;
  dhtPtr->begin();

  sgp30Initialized = sgp.begin();
  if (!sgp30Initialized) {
    Serial.println("WARNING: SGP30 sensor not found on I2C!");
    faultDetections(SENSOR, false);
  }
  sgp.setIAQBaseline(eCO2_base, TVOC_base);

  Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
}

void readSensors(unsigned long currentTime) {
  mq7Update(currentTime);

  if (currentTime - lastSGP30Read >= SGP30_SAMPLE_INTERVAL) {
    lastSGP30Read = currentTime;
    readSGP30();
  }
  if (currentTime - lastTempHumRead >= TEMPHUM_SAMPLE_INTERVAL) {
    lastTempHumRead = currentTime;
    readTempHum();
  }

  readPM25(); // Drain UART2 continuously
}
