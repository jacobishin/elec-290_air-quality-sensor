#include "local_communication.h"
#include "sensors.h"
#include <Preferences.h>
#include <math.h>

// ============================================================
// EXTERNAL REFERENCES FROM MAIN.CPP & STORAGE.CPP
// ============================================================
extern const int I2C_SDA_SCREEN_PIN; // SPI MOSI (GPIO 23)
extern const int I2C_SCL_SCREEN_PIN; // SPI SCK  (GPIO 18)
extern const int SCREEN_CS_PIN;      // GPIO 5
extern const int SCREEN_DC_PIN;      // GPIO 27
extern const int SCREEN_RST_PIN;     // GPIO 26
extern const int SCREEN_BL_PIN;      // ST7789 BLK PWM (GPIO 4, LEDC Ch 2)

extern const int JOY_X_PIN;          // GPIO 34
extern const int JOY_Y_PIN;          // GPIO 35
extern const int JOY_SW_PIN;         // GPIO 32
extern const int BUZZER_PIN;         // GPIO 25

extern const float C0_THRESHOLD1_PPM;
extern const float ECO2_THRESHOLD1_PPM;
extern const float C0_THRESHOLD2_PPM;
extern const float ECO2_THRESHOLD2_PPM;

extern bool wifiConnected;
extern bool dangerAlertActive;
extern String activeAlertMessage;
extern FaultStatus faults;

extern uint8_t activeLogFileIndex;
extern size_t lastUploadedByteOffset;
extern size_t currentLogFileSize;
extern bool timeSyncedThisBoot;
extern const char* getLogFilePath(bool active = true);

// ============================================================
// CONFIGURATION & CONSTANTS
// ============================================================
enum JoyDir { DIR_NONE = 0, DIR_LEFT, DIR_RIGHT, DIR_UP, DIR_DOWN };

// Physical stick -> logical menu action (rotated 90 deg)
#define MAP_PHYS_UP     DIR_RIGHT
#define MAP_PHYS_DOWN   DIR_LEFT
#define MAP_PHYS_LEFT   DIR_UP
#define MAP_PHYS_RIGHT  DIR_DOWN
static const int DIR_MAP[5] = { DIR_NONE, MAP_PHYS_LEFT, MAP_PHYS_RIGHT, MAP_PHYS_UP, MAP_PHYS_DOWN };

static const int SCR_W = 240;
static const int SCR_H = 320;
static const uint16_t COL_HEADER = 0x2104;
static const uint16_t COL_GREY   = 0x4208;
static const uint16_t COL_LIGHT  = 0x7BEF;
static const uint16_t COL_AMBER  = 0xFD20;
static const uint16_t COL_DARKGR = 0x0320;
static const uint16_t COL_WARMBG = 0x02B5;

static const float JOY_ENGAGE  = 0.50f;
static const float JOY_RELEASE = 0.25f;
static const unsigned long JOY_POLL_MS         = 30;
static const unsigned long FLASH_MS            = 350;
static const unsigned long SETTINGS_TIMEOUT_MS = 30000UL;

// LEDC PWM channels (Ch 0 = MQ-7 heater, Ch 2 = LCD backlight, Ch 4 = Buzzer)
static const int BUZZ_FREQ = 2500;
static const int BUZZER_CH = 4;
static const uint8_t VOL_DUTY[11] = {0, 12, 28, 44, 60, 76, 92, 106, 116, 124, 128};

static const int BL_FREQ = 5000;
static const int BL_CH   = 2;
// Perceptually spaced 8-bit PWM curve for brightness 1..10 (never 0 so UI stays visible)
static const uint8_t BL_DUTY[11] = {20, 20, 36, 56, 80, 108, 138, 170, 200, 228, 255};

// ============================================================
// MODULE STATE
// ============================================================
static Adafruit_ST7789 lcd(&SPI, SCREEN_CS_PIN, SCREEN_DC_PIN, SCREEN_RST_PIN);
static Preferences uiPrefs;
UIState uiState = {};

static int centerX = 2048, centerY = 2048;
static int buzzerVolume  = 5;  // 0..10 (NVS "gas_ui" / "vol")
static int lcdBrightness = 10; // 1..10 (NVS "gas_ui" / "bright")

// Non-blocking millis() buzzer state
static bool buzzerActive      = false;
static unsigned long buzzT0   = 0;
static int  oneShotMs         = 0;
static int  testBeepsLeft     = 0;
static bool testBeepOn        = false;

// Display & dirty-tracking cache
static bool warningActive      = false;
static bool anyGasValid        = false;
static bool anySensorWarming   = false;
static bool alertActiveView    = false;
static bool flashPhase         = false;
static bool needsFullRedraw    = true;
static bool lastWifiDrawn      = false;
static int  lastFaultDrawn     = -1;
static int  lastAgeDrawn       = -1;
static int  lastBannerKey      = -1;
static uint16_t lastRowBorder[5] = {0}; // 4 gas rows + 1 climate strip

static unsigned long lastJoyPollMs  = 0;
static unsigned long lastFlashMs    = 0;
static unsigned long lastInputMs    = 0;
static unsigned long lastAgeTickMs  = 0;
static unsigned long testBtnUntilMs = 0;

// ============================================================
// LEDC PWM HELPERS (ESP32 Arduino Core 2.x & 3.x compatible)
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static void buzzerPwmBegin()   { ledcAttachChannel(BUZZER_PIN, BUZZ_FREQ, 8, BUZZER_CH); }
  static void buzzerOn(int d)    { ledcWrite(BUZZER_PIN, d); }
  static void buzzerOff()        { ledcWrite(BUZZER_PIN, 0); }
  static void blPwmBegin()       { ledcAttachChannel(SCREEN_BL_PIN, BL_FREQ, 8, BL_CH); }
  static void blPwmWrite(int d)  { ledcWrite(SCREEN_BL_PIN, d); }
#else
  static void buzzerPwmBegin() {
    ledcSetup(BUZZER_CH, BUZZ_FREQ, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  }
  static void buzzerOn(int d)    { ledcWrite(BUZZER_CH, d); }
  static void buzzerOff()        { ledcWrite(BUZZER_CH, 0); }
  static void blPwmBegin() {
    ledcSetup(BL_CH, BL_FREQ, 8);
    ledcAttachPin(SCREEN_BL_PIN, BL_CH);
  }
  static void blPwmWrite(int d)  { ledcWrite(BL_CH, d); }
#endif

static int volumeDuty() { return VOL_DUTY[constrain(buzzerVolume, 0, 10)]; }

static void applyBrightness() {
  blPwmWrite(BL_DUTY[constrain(lcdBrightness, 1, 10)]);
}

static void saveUISettings() {
  uiPrefs.putInt("vol", buzzerVolume);
  uiPrefs.putInt("bright", lcdBrightness);
}

// ============================================================
// INITIALIZATION
// ============================================================
void initBuzzerHardware() {
  uiPrefs.begin("gas_ui", false);
  buzzerVolume  = constrain(uiPrefs.getInt("vol", 5), 0, 10);
  lcdBrightness = constrain(uiPrefs.getInt("bright", 10), 1, 10);
  buzzerPwmBegin();
  buzzerOff();
}

static void calibrateJoystick() {
  delay(60);
  long sx = 0, sy = 0;
  for (int i = 0; i < 32; i++) {
    sx += analogRead(JOY_X_PIN);
    sy += analogRead(JOY_Y_PIN);
    delay(2);
  }
  int cx = (int)(sx / 32);
  int cy = (int)(sy / 32);

  if (cx < 1400 || cx > 2700 || cy < 1400 || cy > 2700) {
    centerX = 2048;
    centerY = 2048;
    Serial.printf("[UI] WARNING: Joystick boot center out of range (X=%d Y=%d), using 2048.\n", cx, cy);
  } else {
    centerX = cx;
    centerY = cy;
    Serial.printf("[UI] Joystick center: X=%d Y=%d\n", centerX, centerY);
  }
}

void initDisplayAndJoystickHardware() {
  Serial.println("Initializing Portrait ST7789 (HW SPI), Backlight PWM & Joystick...");
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);
  calibrateJoystick();

  blPwmBegin();
  applyBrightness();

  SPI.begin(I2C_SCL_SCREEN_PIN, -1, I2C_SDA_SCREEN_PIN, SCREEN_CS_PIN);
  lcd.init(240, 320);
  lcd.setRotation(2); // Portrait 240x320 (change to 0 if upside down)
  lcd.fillScreen(ST77XX_BLACK);

  lcd.setTextColor(ST77XX_CYAN);
  lcd.setTextSize(2);
  lcd.setCursor(32, 136);
  lcd.print("HARMFUL GAS");
  lcd.setTextColor(ST77XX_WHITE);
  lcd.setCursor(62, 162);
  lcd.print("MONITOR");

  lastInputMs     = millis();
  needsFullRedraw = true;
}

// ============================================================
// NON-BLOCKING MILLIS() BUZZER ENGINE
// ============================================================
void refreshBuzzerState(unsigned long now) {
  if (!dangerAlertActive) {
    uiState.alertMutedByJoystick = false;
  }

  int duty = volumeDuty();
  if (duty == 0) {
    buzzerOff();
    buzzerActive  = false;
    oneShotMs     = 0;
    testBeepsLeft = 0;
    return;
  }

  // Priority 1: Life-safety Danger alarm (always overrides Settings & test beeps, RTM R3)
  if (dangerAlertActive && !uiState.alertMutedByJoystick) {
    testBeepsLeft = 0;
    oneShotMs     = 0;
    if (buzzerActive && (now - buzzT0 >= 125)) {
      buzzerOff(); buzzerActive = false; buzzT0 = now;
    } else if (!buzzerActive && (now - buzzT0 >= 125)) {
      buzzerOn(duty); buzzerActive = true; buzzT0 = now;
    }
    return;
  }

  // Priority 2: Non-blocking 3-beep TEST SOUND
  if (testBeepsLeft > 0) {
    if (!testBeepOn) {
      buzzerOn(duty); testBeepOn = true; buzzT0 = now;
    } else if (now - buzzT0 >= 200) {
      buzzerOff();
      if (now - buzzT0 >= 340) { testBeepOn = false; testBeepsLeft--; }
    }
    return;
  }

  // Priority 3: Non-blocking single sample beep (volume step)
  if (oneShotMs > 0) {
    if (!buzzerActive) {
      buzzerOn(duty); buzzerActive = true; buzzT0 = now;
    } else if (now - buzzT0 >= (unsigned long)oneShotMs) {
      buzzerOff(); buzzerActive = false; oneShotMs = 0;
    }
    return;
  }

  // Priority 4: Moderate T1 Warning beep (Dashboard only)
  if (uiState.currentScreen == PAGE_DASHBOARD && warningActive) {
    if (buzzerActive && (now - buzzT0 >= 180)) {
      buzzerOff(); buzzerActive = false; buzzT0 = now;
    } else if (!buzzerActive && (now - buzzT0 >= 720)) {
      buzzerOn(duty); buzzerActive = true; buzzT0 = now;
    }
    return;
  }

  if (buzzerActive) { buzzerOff(); buzzerActive = false; }
}

// ============================================================
// JOYSTICK INPUT (Throttled + Calibrated + Hysteresis + Remap)
// ============================================================
static float axisFrac(int pin, int center) {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  int d    = (int)(sum / 4) - center;
  int room = (d >= 0) ? (4095 - center) : center;
  return constrain(d / (float)max(room, 200), -1.0f, 1.0f);
}

static bool readButtonClick(unsigned long now) {
  static bool stable = false, lastRaw = false;
  static unsigned long changedAt = 0;
  bool raw = (digitalRead(JOY_SW_PIN) == LOW);
  if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
  if (now - changedAt > 30 && stable != raw) {
    stable = raw;
    if (stable) return true;
  }
  return false;
}

static int readDirection(unsigned long now, bool allowRepeat) {
  static int held = DIR_NONE;
  static unsigned long heldSince = 0, lastRepeat = 0, releasedAt = 0;

  float fx  = axisFrac(JOY_X_PIN, centerX);
  float fy  = axisFrac(JOY_Y_PIN, centerY);
  float ax  = fabsf(fx), ay = fabsf(fy);
  float big = fmaxf(ax, ay);

  if (held != DIR_NONE) {
    if (big < JOY_RELEASE) { held = DIR_NONE; releasedAt = now; return DIR_NONE; }
    if (allowRepeat && (held == DIR_LEFT || held == DIR_RIGHT) &&
        (now - heldSince > 450) && (now - lastRepeat > 140)) {
      lastRepeat = now;
      return held;
    }
    return DIR_NONE;
  }

  if (now - releasedAt < 150 || big < JOY_ENGAGE) return DIR_NONE;

  int dRaw;
  if (ax > ay * 1.5f)      dRaw = (fx > 0) ? DIR_RIGHT : DIR_LEFT;
  else if (ay > ax * 1.5f) dRaw = (fy > 0) ? DIR_DOWN  : DIR_UP;
  else return DIR_NONE;

  held      = DIR_MAP[dRaw];
  heldSince = lastRepeat = now;
  return held;
}

// ============================================================
// VISUAL STATE & DRAWING HELPERS
// ============================================================
static void updateVisualGasState(const ProcessedData &data) {
  const AverageData* s[4] = {&data.coPPM, &data.eco2PPM, &data.tvocPPB, &data.pm25UGM3};
  const float t1[4]       = {C0_THRESHOLD1_PPM, ECO2_THRESHOLD1_PPM, 0.0f, 0.0f};

  anyGasValid      = false;
  warningActive    = false;
  anySensorWarming = isCOInWarmup() || isSGP30InWarmup();

  for (int i = 0; i < 4; i++) {
    if (!s[i]->valid || !isfinite(s[i]->average)) continue;
    anyGasValid = true;
    if (t1[i] > 0.0f && s[i]->average >= t1[i]) warningActive = true;
  }
}

static int activeFaultCount() {
  return (faults.sensor.status ? 1 : 0) + (faults.readingInvalid.status ? 1 : 0) +
         (faults.readingStale.status ? 1 : 0) + (faults.storage.status ? 1 : 0) +
         (faults.wifi.status ? 1 : 0);
}

static void printCentered(const char* s, int cx, int y, uint8_t size, uint16_t fg, uint16_t bg, bool opaque = true) {
  int16_t x1, y1; uint16_t w, h;
  lcd.setTextSize(size);
  lcd.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  if (opaque) lcd.setTextColor(fg, bg);
  else        lcd.setTextColor(fg);
  lcd.setCursor(cx - w / 2, y);
  lcd.print(s);
}

static void drawFaultBadge(int fCount) {
  lastFaultDrawn  = fCount;
  uint16_t pillBg = fCount ? ST77XX_RED : COL_DARKGR;
  uint16_t pillFg = fCount ? ST77XX_WHITE : ST77XX_GREEN;
  char buf[8];
  if (fCount) snprintf(buf, sizeof(buf), "FLT:%d", fCount);
  else        snprintf(buf, sizeof(buf), "OK");
  lcd.fillRoundRect(98, 4, 54, 20, 4, pillBg);
  printCentered(buf, 125, 10, 1, pillFg, pillBg);
}

static void drawWifiBadge() {
  lastWifiDrawn   = wifiConnected;
  uint16_t pillBg = wifiConnected ? COL_DARKGR : ST77XX_RED;
  uint16_t pillFg = wifiConnected ? ST77XX_GREEN : ST77XX_YELLOW;
  lcd.fillRoundRect(156, 4, 44, 20, 4, pillBg);
  printCentered(wifiConnected ? "WiFi" : "NoWF", 178, 10, 1, pillFg, pillBg);
}

static void drawAgeBadge(const ProcessedData &data) {
  int ageSec = (data.windowEndTimestamp > 0 && millis() >= data.windowEndTimestamp)
               ? (int)((millis() - data.windowEndTimestamp) / 1000UL)
               : -1;
  if (ageSec > 99) ageSec = 99;
  if (ageSec == lastAgeDrawn) return;
  lastAgeDrawn = ageSec;

  char buf[6];
  if (ageSec < 0) snprintf(buf, sizeof(buf), " -- ");
  else            snprintf(buf, sizeof(buf), "%2ds ", ageSec);

  lcd.setTextSize(1);
  lcd.setTextColor(COL_LIGHT, COL_HEADER);
  lcd.setCursor(206, 10);
  lcd.print(buf);
}

static void drawHeader(const char* title, bool showBadges, const ProcessedData* data = nullptr) {
  lcd.fillRect(0, 0, SCR_W, 28, COL_HEADER);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_WHITE, COL_HEADER);
  lcd.setCursor(6, 6);
  lcd.print(title);
  if (showBadges) {
    drawFaultBadge(activeFaultCount());
    drawWifiBadge();
    lastAgeDrawn = -2;
    if (data) drawAgeBadge(*data);
  }
}

// Flicker-free slider bar helper (draws filled + unfilled halves directly)
static void drawSliderBar(int value, int maxVal, bool selected) {
  const int barX = 24, barY = 62, barW = 192, barH = 22;
  const int innerX = barX + 2, innerY = barY + 2, innerW = barW - 4, innerH = barH - 4;
  int fillW = (innerW * constrain(value, 0, maxVal)) / maxVal;

  lcd.drawRoundRect(barX, barY, barW, barH, 5, ST77XX_WHITE);
  if (fillW > 0) {
    lcd.fillRect(innerX, innerY, fillW, innerH, selected ? ST77XX_CYAN : ST77XX_WHITE);
  }
  if (fillW < innerW) {
    lcd.fillRect(innerX + fillW, innerY, innerW - fillW, innerH, ST77XX_BLACK);
  }
}

// ============================================================
// SCREEN 1: PLAN A DASHBOARD (4 Full-Width Gas Rows + Climate Strip)
// ============================================================
static void drawDashboard(const ProcessedData &data, bool full) {
  int fCount = activeFaultCount();

  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("GAS MON", true, &data);
    lastBannerKey = -1;
    for (int i = 0; i < 5; i++) lastRowBorder[i] = 0;
  } else {
    if (fCount != lastFaultDrawn)        drawFaultBadge(fCount);
    if (wifiConnected != lastWifiDrawn)  drawWifiBadge();
    drawAgeBadge(data);
  }

  // 4 Primary Gas Rows (CO, eCO2, TVOC, PM2.5)
  const AverageData* s[4] = {&data.coPPM, &data.eco2PPM, &data.tvocPPB, &data.pm25UGM3};
  const char* names[4]    = {"CO", "eCO2", "TVOC", "PM2.5"};
  const char* units[4]    = {"ppm", "ppm", "ppb", "ug/m3"};
  const bool  useDec[4]   = {true, false, false, false};
  const float t1[4]       = {C0_THRESHOLD1_PPM, ECO2_THRESHOLD1_PPM, 0.0f, 0.0f};
  const float t2[4]       = {C0_THRESHOLD2_PPM, ECO2_THRESHOLD2_PPM, 0.0f, 0.0f};

  char buf[16];
  for (int i = 0; i < 4; i++) {
    int y = 32 + i * 50;
    bool ok     = s[i]->valid && isfinite(s[i]->average);
    bool warmup = !ok && ((i == 0 && isCOInWarmup()) || ((i == 1 || i == 2) && isSGP30InWarmup()));

    uint16_t border = ST77XX_GREEN;
    uint16_t valCol = ST77XX_WHITE;
    if (warmup)                                       { border = ST77XX_YELLOW; valCol = ST77XX_YELLOW; }
    else if (!ok)                                     { border = ST77XX_RED;    valCol = ST77XX_RED;    }
    else if (t2[i] > 0.0f && s[i]->average >= t2[i])  { border = ST77XX_RED;    valCol = ST77XX_RED;    }
    else if (t1[i] > 0.0f && s[i]->average >= t1[i])  { border = ST77XX_YELLOW; valCol = ST77XX_YELLOW; }

    if (full || border != lastRowBorder[i]) {
      lastRowBorder[i] = border;
      lcd.drawRoundRect(6, y, 228, 46, 6, border);
    }

    if (full) {
      lcd.setTextSize(2);
      lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      lcd.setCursor(14, y + 7);
      lcd.print(names[i]);

      lcd.setTextSize(1);
      lcd.setTextColor(COL_LIGHT, ST77XX_BLACK);
      lcd.setCursor(14, y + 28);
      lcd.print(units[i]);
    }

    lcd.setTextSize(3);
    lcd.setTextColor(valCol, ST77XX_BLACK);
    lcd.setCursor(92, y + 11);
    if (ok) {
      if (useDec[i]) snprintf(buf, sizeof(buf), "%7.1f", s[i]->average);
      else           snprintf(buf, sizeof(buf), "%7.0f", s[i]->average);
    } else if (warmup) {
      snprintf(buf, sizeof(buf), " WARMUP");
    } else {
      snprintf(buf, sizeof(buf), "  FAULT");
    }
    lcd.print(buf);
  }

  // Compact Climate Strip (Temp & Humidity, size 2)
  bool tempOk = data.temperatureC.valid && isfinite(data.temperatureC.average);
  bool humOk  = data.humidityPercent.valid && isfinite(data.humidityPercent.average);
  uint16_t climBorder = (tempOk && humOk) ? COL_GREY : ST77XX_RED;

  if (full || climBorder != lastRowBorder[4]) {
    lastRowBorder[4] = climBorder;
    lcd.drawRoundRect(6, 234, 228, 34, 6, climBorder);
    lcd.drawFastVLine(120, 240, 22, COL_GREY);
  }

  lcd.setTextSize(2);
  lcd.setTextColor(tempOk ? ST77XX_CYAN : ST77XX_RED, ST77XX_BLACK);
  lcd.setCursor(14, 243);
  if (tempOk) snprintf(buf, sizeof(buf), "T:%4.1fC", data.temperatureC.average);
  else        snprintf(buf, sizeof(buf), "T: ERR ");
  lcd.print(buf);

  lcd.setTextColor(humOk ? ST77XX_CYAN : ST77XX_RED, ST77XX_BLACK);
  lcd.setCursor(130, 243);
  if (humOk) snprintf(buf, sizeof(buf), "H:%3.0f%% ", data.humidityPercent.average);
  else       snprintf(buf, sizeof(buf), "H: ERR ");
  lcd.print(buf);

  // Cached Bottom Status Banner
  int bKey = dangerAlertActive ? (uiState.alertMutedByJoystick ? 4 : 3)
           : (!anyGasValid ? (anySensorWarming ? 6 : 5)
                           : (warningActive ? 2 : (fCount > 0 ? 1 : 0)));

  if (full || bKey != lastBannerKey) {
    lastBannerKey = bKey;
    uint16_t banBg = (bKey >= 3 && bKey <= 4) ? ST77XX_RED
                   : (bKey == 2) ? ST77XX_YELLOW
                   : (bKey == 6) ? COL_WARMBG
                   : (bKey == 1 || bKey == 5) ? COL_AMBER : COL_DARKGR;
    uint16_t banFg = (bKey == 2) ? ST77XX_BLACK : ST77XX_WHITE;
    const char* txt = (bKey == 4) ? "ALARM SILENCED"
                    : (bKey == 3) ? "DANGER!"
                    : (bKey == 6) ? "WARMING UP"
                    : (bKey == 5) ? "NO GAS DATA"
                    : (bKey == 2) ? "WARNING"
                    : (bKey == 1) ? "SYSTEM FAULT" : "AIR NORMAL";

    lcd.fillRect(0, 274, SCR_W, 46, banBg);
    printCentered(txt, 120, 281, 2, banFg, banBg);
    printCentered("L/R:Faults  Click:Menu", 120, 304, 1, banFg, banBg);
  }
}

// ============================================================
// EMERGENCY ALERT OVERRIDE (Live rawData for <= 2s RTM R3)
// ============================================================
static void drawAlertScreen(bool phase, const ProcessedData &data) {
  uint16_t bg = phase ? ST77XX_RED : ST77XX_BLACK;
  uint16_t fg = phase ? ST77XX_WHITE : ST77XX_RED;
  char buf[28];

  lcd.fillScreen(bg);
  lcd.fillTriangle(120, 18, 80, 84, 160, 84, fg);
  printCentered("!", 120, 42, 4, bg, fg, false);
  printCentered("ALERT", 120, 96, 4, fg, bg);

  snprintf(buf, sizeof(buf), "%.20s", activeAlertMessage.c_str());
  printCentered(buf, 120, 138, 2, fg, bg);

  // Prefer live rawData so 1s spikes show immediately; fall back to 5s displayData
  float coVal = (rawData.coPPM.valid && isfinite(rawData.coPPM.value))
                ? rawData.coPPM.value : (data.coPPM.valid ? data.coPPM.average : NAN);
  float ecVal = (rawData.eco2PPM.valid && isfinite(rawData.eco2PPM.value))
                ? rawData.eco2PPM.value : (data.eco2PPM.valid ? data.eco2PPM.average : NAN);
  float pmVal = (rawData.pm25UGM3.valid && isfinite(rawData.pm25UGM3.value))
                ? rawData.pm25UGM3.value : (data.pm25UGM3.valid ? data.pm25UGM3.average : NAN);

  if (isfinite(coVal)) snprintf(buf, sizeof(buf), "CO: %.1f ppm", coVal);
  else                 snprintf(buf, sizeof(buf), "CO: --");
  printCentered(buf, 120, 174, 2, fg, bg);

  if (isfinite(ecVal)) snprintf(buf, sizeof(buf), "eCO2: %.0f ppm", ecVal);
  else                 snprintf(buf, sizeof(buf), "eCO2: --");
  printCentered(buf, 120, 198, 2, fg, bg);

  if (isfinite(pmVal)) snprintf(buf, sizeof(buf), "PM2.5: %.0f ug/m3", pmVal);
  else                 snprintf(buf, sizeof(buf), "PM2.5: --");
  printCentered(buf, 120, 222, 2, fg, bg);

  printCentered("Ventilate / leave area", 120, 258, 1, fg, bg);
  printCentered("CLICK: SILENCE", 120, 284, 2, fg, bg);
}

// ============================================================
// SCREEN 2: TOP-LEVEL FAULTS & SYSTEM STATUS (Large Size 2 Faults)
// ============================================================
static void drawSystemPage(bool full) {
  int fCount = activeFaultCount();

  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("FAULTS & SYS", false);

    lcd.drawRoundRect(6, 34, 228, 158, 6, fCount ? ST77XX_RED : COL_GREY);
    lcd.setTextSize(1);
    lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    lcd.setCursor(14, 41);
    lcd.print("ACTIVE FAULTS");

    lcd.drawRoundRect(6, 198, 228, 92, 6, faults.storage.status ? ST77XX_RED : COL_GREY);
    lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    lcd.setCursor(14, 206);
    lcd.print("STORAGE & SYNC");

    printCentered("L/R:Dashboard  Click:Menu", 120, 302, 1, COL_LIGHT, ST77XX_BLACK);
  } else {
    lcd.drawRoundRect(6, 34, 228, 158, 6, fCount ? ST77XX_RED : COL_GREY);
    lcd.drawRoundRect(6, 198, 228, 92, 6, faults.storage.status ? ST77XX_RED : COL_GREY);
  }

  const char* fNames[5] = {"Sensor", "Inval ", "Stale ", "Store ", "Wi-Fi "};
  const bool  fStat[5]  = {faults.sensor.status, faults.readingInvalid.status,
                           faults.readingStale.status, faults.storage.status, faults.wifi.status};
  const int   fNum[5]   = {faults.sensor.num, faults.readingInvalid.num,
                           faults.readingStale.num, faults.storage.num, faults.wifi.num};

  char buf[42];
  lcd.setTextSize(2);
  for (int i = 0; i < 5; i++) {
    lcd.setCursor(14, 56 + i * 26);
    lcd.setTextColor(fStat[i] ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "%s:%-3s %3d", fNames[i], fStat[i] ? "FLT" : "OK", fNum[i]);
    lcd.print(buf);
  }

  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd.setCursor(14, 224);
  snprintf(buf, sizeof(buf), "File: %-18s (#%u)", getLogFilePath(true), activeLogFileIndex);
  lcd.print(buf);

  unsigned int unsent = (currentLogFileSize > lastUploadedByteOffset)
                        ? (unsigned int)(currentLogFileSize - lastUploadedByteOffset) : 0;
  lcd.setCursor(14, 244);
  snprintf(buf, sizeof(buf), "Size: %-6uB  Unsent: %-6uB", (unsigned int)currentLogFileSize, unsent);
  lcd.print(buf);

  lcd.setCursor(14, 264);
  lcd.setTextColor(wifiConnected ? ST77XX_GREEN : ST77XX_YELLOW, ST77XX_BLACK);
  snprintf(buf, sizeof(buf), "WiFi: %-3s  |  NTP: %-10s",
           wifiConnected ? "OK" : "DC", timeSyncedThisBoot ? "SYNCED" : "BOOT SEC");
  lcd.print(buf);
}

// ============================================================
// SCREEN 3: SETTINGS MENU (Flicker-Free Partial Redraw)
// ============================================================
static void drawSettingsMenu(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("SETTINGS", false);
    printCentered("U/D: Select  Click: Open", 120, 298, 1, COL_LIGHT, ST77XX_BLACK);
  }

  const char* titles[3] = {"Buzzer Volume", "Brightness", "BACK"};
  char sub[22];

  for (int i = 0; i < 3; i++) {
    int y           = 46 + i * 78;
    bool sel        = (uiState.menuIndex == i);
    uint16_t border = sel ? ST77XX_CYAN : COL_GREY;

    // Draw borders only (no fillRoundRect) to eliminate flicker
    lcd.drawRoundRect(12, y, 216, 66, 8, border);
    lcd.drawRoundRect(13, y + 1, 214, 64, 7, sel ? border : ST77XX_BLACK);

    printCentered(titles[i], 120, y + (i == 2 ? 25 : 14), 2,
                  sel ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);

    if (i == 0) {
      if (buzzerVolume == 0) snprintf(sub, sizeof(sub), "    MUTED    ");
      else                   snprintf(sub, sizeof(sub), "Volume: %2d/10", buzzerVolume);
      printCentered(sub, 120, y + 40, 1, COL_LIGHT, ST77XX_BLACK);
    } else if (i == 1) {
      snprintf(sub, sizeof(sub), "Level: %3d%%  ", lcdBrightness * 10);
      printCentered(sub, 120, y + 40, 1, COL_LIGHT, ST77XX_BLACK);
    }
  }
}

// ============================================================
// SCREEN 4: BUZZER PAGE (Flicker-Free)
// ============================================================
static void drawBuzzerPage(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("BUZZER", false);
    printCentered("U/D: Row  L/R: Vol  Click: OK", 120, 302, 1, COL_LIGHT, ST77XX_BLACK);
  }

  bool s0 = (uiState.subRow == 0);
  lcd.drawRoundRect(12, 48, 216, 86, 8, s0 ? ST77XX_CYAN : COL_GREY);
  drawSliderBar(buzzerVolume, 10, s0);

  char buf[16];
  if (buzzerVolume == 0) snprintf(buf, sizeof(buf), " MUTED ");
  else                   snprintf(buf, sizeof(buf), " %2d/10 ", buzzerVolume);
  printCentered(buf, 120, 96, 3, ST77XX_WHITE, ST77XX_BLACK);

  bool s1 = (uiState.subRow == 1);
  bool hi = (testBtnUntilMs > 0);
  if (full || hi) {
    lcd.fillRoundRect(14, 154, 212, 50, 6, hi ? ST77XX_CYAN : ST77XX_BLACK);
  }
  lcd.drawRoundRect(12, 152, 216, 54, 8, s1 ? ST77XX_CYAN : COL_GREY);
  printCentered(hi ? (buzzerVolume ? "BEEPING..." : "  MUTED   ") : "TEST SOUND",
                120, 171, 2, hi ? ST77XX_BLACK : (s1 ? ST77XX_CYAN : ST77XX_WHITE),
                hi ? ST77XX_CYAN : ST77XX_BLACK);

  bool s2 = (uiState.subRow == 2);
  lcd.drawRoundRect(12, 222, 216, 54, 8, s2 ? ST77XX_CYAN : COL_GREY);
  printCentered("SAVE & BACK", 120, 241, 2, s2 ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);
}

// ============================================================
// SCREEN 5: BRIGHTNESS PAGE (Flicker-Free PWM Control)
// ============================================================
static void drawBrightnessPage(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("BRIGHTNESS", false);
    printCentered("L/R: Adjust  Click: Save", 120, 302, 1, COL_LIGHT, ST77XX_BLACK);
  }

  bool s0 = (uiState.subRow == 0);
  lcd.drawRoundRect(12, 48, 216, 104, 8, s0 ? ST77XX_CYAN : COL_GREY);
  drawSliderBar(lcdBrightness, 10, s0);

  char buf[16];
  snprintf(buf, sizeof(buf), " %3d%% ", lcdBrightness * 10);
  printCentered(buf, 120, 100, 3, ST77XX_WHITE, ST77XX_BLACK);

  bool s1 = (uiState.subRow == 1);
  lcd.drawRoundRect(12, 176, 216, 54, 8, s1 ? ST77XX_CYAN : COL_GREY);
  printCentered("SAVE & BACK", 120, 195, 2, s1 ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);
}

// ============================================================
// PUBLIC DISPLAY & INPUT HANDLERS
// ============================================================
void renderDisplay(const ProcessedData &data) {
  updateVisualGasState(data);

  // Life-safety override: immediately exit any page on Danger (RTM R3)
  if (dangerAlertActive && !uiState.alertMutedByJoystick) {
    if (uiState.currentScreen != PAGE_DASHBOARD) {
      saveUISettings();
      uiState.currentScreen = PAGE_DASHBOARD;
    }
    if (!alertActiveView) {
      alertActiveView = true;
      flashPhase      = true;
      lastFlashMs     = millis();
      drawAlertScreen(flashPhase, data);
    }
    return;
  }

  if (alertActiveView) {
    alertActiveView = false;
    needsFullRedraw = true;
  }

  switch (uiState.currentScreen) {
    case PAGE_DASHBOARD:  drawDashboard(data, needsFullRedraw);     break;
    case SYSTEM_PAGE:     drawSystemPage(needsFullRedraw);          break;
    case SETTINGS_MENU:   drawSettingsMenu(needsFullRedraw);        break;
    case BUZZER_PAGE:     drawBuzzerPage(needsFullRedraw);          break;
    case BRIGHTNESS_PAGE: drawBrightnessPage(needsFullRedraw);      break;
  }
  needsFullRedraw = false;
}

void handleJoystickInput(unsigned long now, const ProcessedData &data) {
  // 1. Immediate life-safety Alert override & 350 ms flashing (RTM R3)
  bool wantAlert = dangerAlertActive && !uiState.alertMutedByJoystick;
  if (wantAlert != alertActiveView) {
    renderDisplay(data);
  } else if (alertActiveView && (now - lastFlashMs >= FLASH_MS)) {
    lastFlashMs = now;
    flashPhase  = !flashPhase;
    drawAlertScreen(flashPhase, data);
  }

  // 2. Refresh 1s dashboard age indicator without redrawing full screen
  if (!alertActiveView && uiState.currentScreen == PAGE_DASHBOARD && (now - lastAgeTickMs >= 1000UL)) {
    lastAgeTickMs = now;
    drawAgeBadge(data);
  }

  // 3. Reset TEST SOUND button highlight when done
  if (testBtnUntilMs > 0 && now >= testBtnUntilMs) {
    testBtnUntilMs = 0;
    if (uiState.currentScreen == BUZZER_PAGE) {
      lcd.fillRoundRect(14, 154, 212, 50, 6, ST77XX_BLACK);
      drawBuzzerPage(false);
    }
  }

  // 4. Auto-exit Settings screens after 30 s inactivity (top-level Dashboard & System stay open)
  bool inSettings = (uiState.currentScreen == SETTINGS_MENU ||
                     uiState.currentScreen == BUZZER_PAGE ||
                     uiState.currentScreen == BRIGHTNESS_PAGE);
  if (inSettings && (now - lastInputMs > SETTINGS_TIMEOUT_MS)) {
    saveUISettings();
    uiState.currentScreen = PAGE_DASHBOARD;
    needsFullRedraw       = true;
    renderDisplay(data);
    return;
  }

  // 5. Throttle ADC/button polling to 30 ms (~33 Hz)
  if (now - lastJoyPollMs < JOY_POLL_MS) return;
  lastJoyPollMs = now;

  bool allowRepeat = (uiState.subRow == 0) &&
                     (uiState.currentScreen == BUZZER_PAGE || uiState.currentScreen == BRIGHTNESS_PAGE);
  bool click = readButtonClick(now);
  int  dir   = readDirection(now, allowRepeat);
  if (!click && dir == DIR_NONE) return;
  lastInputMs = now;

  // --- Screen-specific navigation ---
  if (uiState.currentScreen == PAGE_DASHBOARD) {
    if (alertActiveView) {
      if (click) {
        uiState.alertMutedByJoystick = true;
        renderDisplay(data);
      }
      return;
    }
    if (dir == DIR_LEFT || dir == DIR_RIGHT) {
      uiState.currentScreen = SYSTEM_PAGE;
      needsFullRedraw       = true;
      renderDisplay(data);
    } else if (click) {
      uiState.currentScreen = SETTINGS_MENU;
      uiState.menuIndex     = 0;
      needsFullRedraw       = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == SYSTEM_PAGE) {
    if (dir == DIR_LEFT || dir == DIR_RIGHT) {
      uiState.currentScreen = PAGE_DASHBOARD;
      needsFullRedraw       = true;
      renderDisplay(data);
    } else if (click) {
      uiState.currentScreen = SETTINGS_MENU;
      uiState.menuIndex     = 0;
      needsFullRedraw       = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == SETTINGS_MENU) {
    if (dir == DIR_UP || dir == DIR_DOWN) {
      uiState.menuIndex = (uiState.menuIndex + (dir == DIR_DOWN ? 1 : 2)) % 3;
      drawSettingsMenu(false);
    } else if (dir == DIR_LEFT) {
      uiState.currentScreen = PAGE_DASHBOARD;
      needsFullRedraw       = true;
      renderDisplay(data);
    } else if (click) {
      uiState.currentScreen = (uiState.menuIndex == 0) ? BUZZER_PAGE
                            : (uiState.menuIndex == 1) ? BRIGHTNESS_PAGE : PAGE_DASHBOARD;
      uiState.subRow        = 0;
      needsFullRedraw       = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == BUZZER_PAGE) {
    if (dir == DIR_UP || dir == DIR_DOWN) {
      uiState.subRow = (uiState.subRow + (dir == DIR_DOWN ? 1 : 2)) % 3;
      drawBuzzerPage(false);
    } else if ((dir == DIR_LEFT || dir == DIR_RIGHT) && uiState.subRow == 0) {
      int old = buzzerVolume;
      buzzerVolume = constrain(buzzerVolume + (dir == DIR_RIGHT ? 1 : -1), 0, 10);
      if (buzzerVolume != old) {
        oneShotMs = 70;
        drawBuzzerPage(false);
      }
    } else if (dir == DIR_LEFT && uiState.subRow != 0) {
      saveUISettings();
      uiState.currentScreen = SETTINGS_MENU;
      needsFullRedraw       = true;
      renderDisplay(data);
    } else if (click) {
      if (uiState.subRow == 0) {
        oneShotMs = 140;
      } else if (uiState.subRow == 1) {
        if (buzzerVolume > 0) testBeepsLeft = 3;
        testBtnUntilMs = now + 1000;
        drawBuzzerPage(false);
      } else {
        saveUISettings();
        uiState.currentScreen = SETTINGS_MENU;
        needsFullRedraw       = true;
        renderDisplay(data);
      }
    }
  } else if (uiState.currentScreen == BRIGHTNESS_PAGE) {
    if (dir == DIR_UP || dir == DIR_DOWN) {
      uiState.subRow = (uiState.subRow + 1) % 2;
      drawBrightnessPage(false);
    } else if ((dir == DIR_LEFT || dir == DIR_RIGHT) && uiState.subRow == 0) {
      int old = lcdBrightness;
      lcdBrightness = constrain(lcdBrightness + (dir == DIR_RIGHT ? 1 : -1), 1, 10);
      if (lcdBrightness != old) {
        applyBrightness();
        drawBrightnessPage(false);
      }
    } else if (dir == DIR_LEFT && uiState.subRow != 0) {
      saveUISettings();
      uiState.currentScreen = SETTINGS_MENU;
      needsFullRedraw       = true;
      renderDisplay(data);
    } else if (click) {
      saveUISettings();
      uiState.currentScreen = SETTINGS_MENU;
      needsFullRedraw       = true;
      renderDisplay(data);
    }
  }
}
