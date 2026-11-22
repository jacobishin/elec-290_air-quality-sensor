#include "local_communication.h"
#include <Preferences.h>
#include <math.h>

// ============================================================
// EXTERNAL REFERENCES FROM MAIN.CPP
// ============================================================
extern const int I2C_SDA_SCREEN_PIN; // SPI MOSI (GPIO 23)
extern const int I2C_SCL_SCREEN_PIN; // SPI SCK  (GPIO 18)
extern const int SCREEN_CS_PIN;      // GPIO 5
extern const int SCREEN_DC_PIN;      // GPIO 27
extern const int SCREEN_RST_PIN;     // GPIO 26

extern const int JOY_X_PIN;          // GPIO 34
extern const int JOY_Y_PIN;          // GPIO 35
extern const int JOY_SW_PIN;         // GPIO 32
extern const int BUZZER_PIN;         // GPIO 25

extern const float C0_THRESHOLD1_PPM;
extern const float TVOC_THRESHOLD1_PPB;
extern const float ECO2_THRESHOLD1_PPM;
extern const float PM25_THRESHOLD1_UGM3;

extern const float C0_THRESHOLD2_PPM;
extern const float TVOC_THRESHOLD2_PPB;
extern const float ECO2_THRESHOLD2_PPM;
extern const float PM25_THRESHOLD2_UGM3;

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

// Physical stick -> logical menu action (currently rotated 90 deg)
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

static const float JOY_ENGAGE  = 0.50f;
static const float JOY_RELEASE = 0.25f;
static const unsigned long JOY_POLL_MS         = 30;
static const unsigned long FLASH_MS            = 350;
static const unsigned long SETTINGS_TIMEOUT_MS = 30000UL;

static const int BUZZ_FREQ = 2500;
static const int BUZZER_CH = 4; // Avoids LEDC ch 0-3 collisions (e.g., MQ-7 PWM)
static const uint8_t VOL_DUTY[11] = {0, 12, 28, 44, 60, 76, 92, 106, 116, 124, 128};

// ============================================================
// MODULE STATE
// ============================================================
static Adafruit_ST7789 lcd(&SPI, SCREEN_CS_PIN, SCREEN_DC_PIN, SCREEN_RST_PIN);
static Preferences uiPrefs;
UIState uiState = {};

static int centerX = 2048, centerY = 2048;
static int buzzerVolume = 5; // 0..10 (saved in NVS "gas_ui")

// Non-blocking millis() buzzer state
static bool buzzerActive      = false;
static unsigned long buzzT0   = 0;
static int  oneShotMs         = 0;
static int  testBeepsLeft     = 0;
static bool testBeepOn        = false;

// Display & cache state
static bool warningActive     = false; // Visual T1 warning only (Danger comes from main.cpp)
static bool anyGasValid       = false;
static bool alertActiveView   = false;
static bool flashPhase        = false;
static bool needsFullRedraw   = true;
static bool lastWifiDrawn     = false;
static int  lastBannerKey     = -1;
static uint16_t lastCardBorder[6] = {0};

static unsigned long lastJoyPollMs  = 0;
static unsigned long lastFlashMs    = 0;
static unsigned long lastInputMs    = 0;
static unsigned long testBtnUntilMs = 0;

// ============================================================
// LEDC PWM HELPERS (ESP32 Arduino Core 2.x & 3.x compatible)
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static void pwmBegin()      { ledcAttachChannel(BUZZER_PIN, BUZZ_FREQ, 8, BUZZER_CH); }
  static void buzzerOn(int d) { ledcWrite(BUZZER_PIN, d); }
  static void buzzerOff()     { ledcWrite(BUZZER_PIN, 0); }
#else
  static void pwmBegin() {
    ledcSetup(BUZZER_CH, BUZZ_FREQ, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  }
  static void buzzerOn(int d) { ledcWrite(BUZZER_PIN, d); }
  static void buzzerOff()     { ledcWrite(BUZZER_PIN, 0); }
#endif

static int volumeDuty() { return VOL_DUTY[constrain(buzzerVolume, 0, 10)]; }

// ============================================================
// INITIALIZATION
// ============================================================
void initBuzzerHardware() {
  uiPrefs.begin("gas_ui", false);
  buzzerVolume = constrain(uiPrefs.getInt("vol", 5), 0, 10);
  pwmBegin();
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
  Serial.println("Initializing Portrait ST7789 (HW SPI) & Joystick...");
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);
  calibrateJoystick();

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

  lastInputMs = millis();
  needsFullRedraw = true;
}

// ============================================================
// NON-BLOCKING MILLIS() BUZZER ENGINE
// ============================================================
void refreshBuzzerState(unsigned long now) {
  if (!dangerAlertActive) {
    uiState.alertMutedByJoystick = false; // Auto-reset mute when Section 4 clears danger
  }

  int duty = volumeDuty();
  if (duty == 0) {
    buzzerOff();
    buzzerActive = false;
    oneShotMs = 0;
    testBeepsLeft = 0;
    return;
  }

  // Priority 1: Life-safety Danger alarm (always overrides Settings & test beeps, RTM R3)
  if (dangerAlertActive && !uiState.alertMutedByJoystick) {
    testBeepsLeft = 0;
    oneShotMs = 0;
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
  int d = (int)(sum / 4) - center;
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

  float fx = axisFrac(JOY_X_PIN, centerX);
  float fy = axisFrac(JOY_Y_PIN, centerY);
  float ax = fabsf(fx), ay = fabsf(fy);
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

  held = DIR_MAP[dRaw];
  heldSince = lastRepeat = now;
  return held;
}

// ============================================================
// VISUAL STATE & DRAWING HELPERS
// ============================================================
static void updateVisualGasState(const ProcessedData &data) {
  const AverageData* s[4] = {&data.coPPM, &data.tvocPPB, &data.eco2PPM, &data.pm25UGM3};
  const float t1[4] = {C0_THRESHOLD1_PPM, TVOC_THRESHOLD1_PPB, ECO2_THRESHOLD1_PPM, PM25_THRESHOLD1_UGM3};

  anyGasValid = false;
  warningActive = false;
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

static void drawWifiPill() {
  lastWifiDrawn = wifiConnected;
  uint16_t pillBg = wifiConnected ? 0x0320 : ST77XX_RED;
  uint16_t pillFg = wifiConnected ? ST77XX_GREEN : ST77XX_YELLOW;
  lcd.fillRoundRect(174, 4, 60, 22, 4, pillBg);
  printCentered(wifiConnected ? "WiFi:OK" : "WiFi:DC", 204, 11, 1, pillFg, pillBg);
}

static void drawHeader(const char* title, bool showWifi) {
  lcd.fillRect(0, 0, SCR_W, 30, COL_HEADER);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_WHITE, COL_HEADER);
  lcd.setCursor(8, 7);
  lcd.print(title);
  if (showWifi) drawWifiPill();
}

// ============================================================
// SCREENS: DASHBOARD, ALERT, SETTINGS, SYSTEM, BUZZER
// ============================================================
static void drawDashboard(const ProcessedData &data, bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("AIR MONITOR", true);
    lastBannerKey = -1;
    for (int i = 0; i < 6; i++) lastCardBorder[i] = 0;
  } else if (wifiConnected != lastWifiDrawn) {
    drawWifiPill();
  }

  const AverageData* s[6] = {
    &data.coPPM, &data.tvocPPB, &data.eco2PPM,
    &data.temperatureC, &data.humidityPercent, &data.pm25UGM3
  };
  const char* names[6] = {"CO", "TVOC", "eCO2", "Temp", "Hum", "PM2.5"};
  const char* units[6] = {"ppm", "ppb", "ppm", "C", "%", "ug/m3"};
  const float t1[6]    = {C0_THRESHOLD1_PPM, TVOC_THRESHOLD1_PPB, ECO2_THRESHOLD1_PPM, 0, 0, PM25_THRESHOLD1_UGM3};
  const float t2[6]    = {C0_THRESHOLD2_PPM, TVOC_THRESHOLD2_PPB, ECO2_THRESHOLD2_PPM, 0, 0, PM25_THRESHOLD2_UGM3};

  char buf[18];
  for (int i = 0; i < 6; i++) {
    int x = 6 + (i % 2) * 116;
    int y = 36 + (i / 2) * 78;
    bool ok = s[i]->valid && isfinite(s[i]->average);

    uint16_t border = ST77XX_GREEN;
    uint16_t valCol = ST77XX_WHITE;
    if (!ok) { border = ST77XX_RED; valCol = ST77XX_RED; }
    else if (t2[i] > 0.0f && s[i]->average >= t2[i]) { border = ST77XX_RED; valCol = ST77XX_RED; }
    else if (t1[i] > 0.0f && s[i]->average >= t1[i]) { border = ST77XX_YELLOW; valCol = ST77XX_YELLOW; }

    if (full || border != lastCardBorder[i]) {
      lastCardBorder[i] = border;
      lcd.drawRoundRect(x, y, 112, 72, 6, border);
    }

    if (full) {
      lcd.setTextSize(1);
      lcd.setTextColor(COL_LIGHT, ST77XX_BLACK);
      lcd.setCursor(x + 8, y + 8);
      snprintf(buf, sizeof(buf), "%s (%s)", names[i], units[i]);
      lcd.print(buf);
    }

    lcd.setTextSize(2);
    lcd.setTextColor(valCol, ST77XX_BLACK);
    lcd.setCursor(x + 8, y + 32);
    if (ok) snprintf(buf, sizeof(buf), "%-7.1f", s[i]->average);
    else    snprintf(buf, sizeof(buf), "%-7s", "FAULT");
    lcd.print(buf);
  }

  // Cached Bottom Status Banner (redraws only when state changes)
  int bKey = dangerAlertActive ? (uiState.alertMutedByJoystick ? 4 : 3)
           : (!anyGasValid ? 5 : (warningActive ? 2 : (activeFaultCount() > 0 ? 1 : 0)));
  if (full || bKey != lastBannerKey) {
    lastBannerKey = bKey;
    uint16_t banBg = (bKey >= 3 && bKey <= 4) ? ST77XX_RED
                   : (bKey == 2) ? ST77XX_YELLOW
                   : (bKey == 1 || bKey == 5) ? 0xFD20 : 0x0320;
    uint16_t banFg = (bKey == 2) ? ST77XX_BLACK : ST77XX_WHITE;
    const char* txt = (bKey == 4) ? "ALARM SILENCED"
                    : (bKey == 3) ? "DANGER!"
                    : (bKey == 5) ? "NO GAS DATA"
                    : (bKey == 2) ? "WARNING"
                    : (bKey == 1) ? "SYSTEM FAULT" : "AIR NORMAL";

    lcd.fillRect(0, 274, SCR_W, 46, banBg);
    printCentered(txt, 120, 282, 2, banFg, banBg);
    printCentered("Click: Settings / Menu", 120, 304, 1, banFg, banBg);
  }
}

static void drawAlertScreen(bool phase, const ProcessedData &data) {
  uint16_t bg = phase ? ST77XX_RED : ST77XX_BLACK;
  uint16_t fg = phase ? ST77XX_WHITE : ST77XX_RED;
  char buf[28];

  lcd.fillScreen(bg);
  lcd.fillTriangle(120, 18, 80, 84, 160, 84, fg);
  printCentered("!", 120, 42, 4, bg, fg, false); // Transparent over triangle
  printCentered("ALERT", 120, 96, 4, fg, bg);

  snprintf(buf, sizeof(buf), "%.20s", activeAlertMessage.c_str());
  printCentered(buf, 120, 138, 2, fg, bg);

  if (data.coPPM.valid && isfinite(data.coPPM.average))
    snprintf(buf, sizeof(buf), "CO: %.1f ppm", data.coPPM.average);
  else snprintf(buf, sizeof(buf), "CO: --");
  printCentered(buf, 120, 174, 2, fg, bg);

  if (data.eco2PPM.valid && isfinite(data.eco2PPM.average))
    snprintf(buf, sizeof(buf), "eCO2: %.0f ppm", data.eco2PPM.average);
  else snprintf(buf, sizeof(buf), "eCO2: --");
  printCentered(buf, 120, 198, 2, fg, bg);

  if (data.pm25UGM3.valid && isfinite(data.pm25UGM3.average))
    snprintf(buf, sizeof(buf), "PM2.5: %.1f ug/m3", data.pm25UGM3.average);
  else snprintf(buf, sizeof(buf), "PM2.5: --");
  printCentered(buf, 120, 222, 2, fg, bg);

  printCentered("Ventilate / leave area", 120, 258, 1, fg, bg);
  printCentered("CLICK: SILENCE", 120, 284, 2, fg, bg);
}

static void drawSettingsMenu(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("SETTINGS", false);
    printCentered("U/D: Select  Click: Open", 120, 298, 1, COL_LIGHT, ST77XX_BLACK);
  }
  const char* titles[3] = {"System & Faults", "Buzzer Volume", "BACK"};
  char sub[20];
  for (int i = 0; i < 3; i++) {
    int y = 48 + i * 78;
    bool sel = (uiState.menuIndex == i);
    uint16_t border = sel ? ST77XX_CYAN : COL_GREY;
    lcd.fillRoundRect(12, y, 216, 66, 8, ST77XX_BLACK);
    lcd.drawRoundRect(12, y, 216, 66, 8, border);
    if (sel) lcd.drawRoundRect(13, y + 1, 214, 64, 7, border);

    printCentered(titles[i], 120, y + (i == 2 ? 25 : 14), 2, sel ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);
    if (i == 0) {
      int f = activeFaultCount();
      snprintf(sub, sizeof(sub), f ? "%d FAULT(S)" : "ALL OK", f);
      printCentered(sub, 120, y + 40, 1, f ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
    } else if (i == 1) {
      if (buzzerVolume == 0) snprintf(sub, sizeof(sub), "MUTED");
      else snprintf(sub, sizeof(sub), "Volume: %d/10", buzzerVolume);
      printCentered(sub, 120, y + 40, 1, COL_LIGHT, ST77XX_BLACK);
    }
  }
}

static void drawSystemPage(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("SYSTEM STATUS", false);
    printCentered("Click or Left: Back", 120, 302, 1, COL_LIGHT, ST77XX_BLACK);
  }
  char buf[42];
  lcd.drawRoundRect(8, 38, 224, 90, 6, faults.storage.status ? ST77XX_RED : COL_GREY);
  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.setCursor(16, 46); lcd.print("STORAGE & SYNC");

  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd.setCursor(16, 64);
  snprintf(buf, sizeof(buf), "File: %-18s (#%u)", getLogFilePath(true), activeLogFileIndex);
  lcd.print(buf);

  unsigned int unsent = (currentLogFileSize > lastUploadedByteOffset)
                        ? (unsigned int)(currentLogFileSize - lastUploadedByteOffset) : 0;
  lcd.setCursor(16, 82);
  snprintf(buf, sizeof(buf), "Size: %-6uB Unsent: %-6uB", (unsigned int)currentLogFileSize, unsent);
  lcd.print(buf);

  lcd.setCursor(16, 100);
  snprintf(buf, sizeof(buf), "WiFi: %-3s | NTP: %-10s",
           wifiConnected ? "OK" : "DC", timeSyncedThisBoot ? "SYNCED" : "BOOT SEC");
  lcd.setTextColor(wifiConnected ? ST77XX_GREEN : ST77XX_YELLOW, ST77XX_BLACK);
  lcd.print(buf);

  lcd.drawRoundRect(8, 138, 224, 150, 6, activeFaultCount() ? ST77XX_RED : COL_GREY);
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  lcd.setCursor(16, 146); lcd.print("FAULT STATUS & COUNTS");

  const char* fNames[5] = {"Sensor ", "Invalid", "Stale  ", "Storage", "Wi-Fi  "};
  const bool  fStat[5]  = {faults.sensor.status, faults.readingInvalid.status,
                           faults.readingStale.status, faults.storage.status, faults.wifi.status};
  const int   fNum[5]   = {faults.sensor.num, faults.readingInvalid.num,
                           faults.readingStale.num, faults.storage.num, faults.wifi.num};

  for (int i = 0; i < 5; i++) {
    lcd.setCursor(16, 166 + i * 22);
    lcd.setTextColor(fStat[i] ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "%s : %-5s (%-3d)", fNames[i], fStat[i] ? "FAULT" : "OK", fNum[i]);
    lcd.print(buf);
  }
}

static void drawBuzzerPage(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("BUZZER", false);
    printCentered("U/D: Row  L/R: Vol  Click: OK", 120, 302, 1, COL_LIGHT, ST77XX_BLACK);
  }

  bool s0 = (uiState.buzzRow == 0);
  lcd.fillRoundRect(12, 48, 216, 86, 8, ST77XX_BLACK);
  lcd.drawRoundRect(12, 48, 216, 86, 8, s0 ? ST77XX_CYAN : COL_GREY);
  lcd.drawRoundRect(24, 62, 192, 22, 5, ST77XX_WHITE);
  int w = (188 * buzzerVolume) / 10;
  if (w > 0) lcd.fillRoundRect(26, 64, w, 18, 4, s0 ? ST77XX_CYAN : ST77XX_WHITE);
  char buf[16];
  if (buzzerVolume == 0) snprintf(buf, sizeof(buf), "MUTED");
  else snprintf(buf, sizeof(buf), "%d / 10", buzzerVolume);
  printCentered(buf, 120, 96, 3, ST77XX_WHITE, ST77XX_BLACK);

  bool s1 = (uiState.buzzRow == 1);
  bool hi = (testBtnUntilMs > 0);
  lcd.fillRoundRect(12, 152, 216, 54, 8, hi ? ST77XX_CYAN : ST77XX_BLACK);
  lcd.drawRoundRect(12, 152, 216, 54, 8, s1 ? ST77XX_CYAN : COL_GREY);
  printCentered(hi ? (buzzerVolume ? "BEEPING..." : "MUTED") : "TEST SOUND",
                120, 171, 2, hi ? ST77XX_BLACK : (s1 ? ST77XX_CYAN : ST77XX_WHITE),
                hi ? ST77XX_CYAN : ST77XX_BLACK);

  bool s2 = (uiState.buzzRow == 2);
  lcd.fillRoundRect(12, 222, 216, 54, 8, ST77XX_BLACK);
  lcd.drawRoundRect(12, 222, 216, 54, 8, s2 ? ST77XX_CYAN : COL_GREY);
  printCentered("SAVE & BACK", 120, 241, 2, s2 ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);
}

// ============================================================
// PUBLIC DISPLAY & INPUT HANDLERS
// ============================================================
void renderDisplay(const ProcessedData &data) {
  updateVisualGasState(data);

  // Life-safety override: immediately exit Settings on Danger (RTM R3)
  if (dangerAlertActive && !uiState.alertMutedByJoystick) {
    if (uiState.currentScreen != PAGE_DASHBOARD) {
      uiPrefs.putInt("vol", buzzerVolume);
      uiState.currentScreen = PAGE_DASHBOARD;
    }
    if (!alertActiveView) {
      alertActiveView = true;
      flashPhase = true;
      lastFlashMs = millis();
      drawAlertScreen(flashPhase, data);
    }
    return;
  }

  if (alertActiveView) {
    alertActiveView = false;
    needsFullRedraw = true;
  }

  switch (uiState.currentScreen) {
    case PAGE_DASHBOARD: drawDashboard(data, needsFullRedraw); break;
    case SETTINGS_MENU:  drawSettingsMenu(needsFullRedraw);    break;
    case SYSTEM_PAGE:    drawSystemPage(needsFullRedraw);      break;
    case BUZZER_PAGE:    drawBuzzerPage(needsFullRedraw);      break;
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
    flashPhase = !flashPhase;
    drawAlertScreen(flashPhase, data);
  }

  // 2. Reset TEST SOUND button highlight when done
  if (testBtnUntilMs > 0 && now >= testBtnUntilMs) {
    testBtnUntilMs = 0;
    if (uiState.currentScreen == BUZZER_PAGE) drawBuzzerPage(false);
  }

  // 3. Auto-exit Settings after 30 s inactivity
  if (uiState.currentScreen != PAGE_DASHBOARD && (now - lastInputMs > SETTINGS_TIMEOUT_MS)) {
    uiPrefs.putInt("vol", buzzerVolume);
    uiState.currentScreen = PAGE_DASHBOARD;
    needsFullRedraw = true;
    renderDisplay(data);
    return;
  }

  // 4. Throttle ADC/button polling to 30 ms (~33 Hz) to save CPU & battery
  if (now - lastJoyPollMs < JOY_POLL_MS) return;
  lastJoyPollMs = now;

  bool click = readButtonClick(now);
  int dir    = readDirection(now, uiState.currentScreen == BUZZER_PAGE && uiState.buzzRow == 0);
  if (!click && dir == DIR_NONE) return;
  lastInputMs = now;

  if (uiState.currentScreen == PAGE_DASHBOARD) {
    if (alertActiveView) {
      if (click) {
        uiState.alertMutedByJoystick = true;
        renderDisplay(data);
      }
      return;
    }
    if (click) {
      uiState.currentScreen = SETTINGS_MENU;
      uiState.menuIndex = 0;
      needsFullRedraw = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == SETTINGS_MENU) {
    if (dir == DIR_UP || dir == DIR_DOWN) {
      uiState.menuIndex = (uiState.menuIndex + (dir == DIR_DOWN ? 1 : 2)) % 3;
      drawSettingsMenu(false);
    } else if (dir == DIR_LEFT) {
      uiState.currentScreen = PAGE_DASHBOARD;
      needsFullRedraw = true;
      renderDisplay(data);
    } else if (click) {
      uiState.currentScreen = (uiState.menuIndex == 0) ? SYSTEM_PAGE
                            : (uiState.menuIndex == 1) ? BUZZER_PAGE : PAGE_DASHBOARD;
      uiState.buzzRow = 0;
      needsFullRedraw = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == SYSTEM_PAGE) {
    if (click || dir == DIR_LEFT) {
      uiState.currentScreen = SETTINGS_MENU;
      needsFullRedraw = true;
      renderDisplay(data);
    }
  } else if (uiState.currentScreen == BUZZER_PAGE) {
    if (dir == DIR_UP || dir == DIR_DOWN) {
      uiState.buzzRow = (uiState.buzzRow + (dir == DIR_DOWN ? 1 : 2)) % 3;
      drawBuzzerPage(false);
    } else if ((dir == DIR_LEFT || dir == DIR_RIGHT) && uiState.buzzRow == 0) {
      int old = buzzerVolume;
      buzzerVolume = constrain(buzzerVolume + (dir == DIR_RIGHT ? 1 : -1), 0, 10);
      if (buzzerVolume != old) {
        oneShotMs = 70;
        drawBuzzerPage(false);
      }
    } else if (dir == DIR_LEFT && uiState.buzzRow != 0) {
      uiPrefs.putInt("vol", buzzerVolume);
      uiState.currentScreen = SETTINGS_MENU;
      needsFullRedraw = true;
      renderDisplay(data);
    } else if (click) {
      if (uiState.buzzRow == 0) {
        oneShotMs = 140;
      } else if (uiState.buzzRow == 1) {
        if (buzzerVolume > 0) testBeepsLeft = 3;
        testBtnUntilMs = now + 1000;
        drawBuzzerPage(false);
      } else {
        uiPrefs.putInt("vol", buzzerVolume);
        uiState.currentScreen = SETTINGS_MENU;
        needsFullRedraw = true;
        renderDisplay(data);
      }
    }
  }
}