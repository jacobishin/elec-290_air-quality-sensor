#include "local_communication.h"
#include "sensors.h"
#include "storage.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

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
extern const float ECO2_THRESHOLD1_PPM;
extern const float C0_THRESHOLD2_PPM;
extern const float ECO2_THRESHOLD2_PPM;

extern bool dangerAlertActive;
extern String activeAlertMessage;
extern FaultStatus faults;

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

// Theme 1: Medical / Lab Instrument (Teal & Crisp White on Pure Black)
static const uint16_t COL_HEADER = 0x0147; // Deep teal-navy header bar
static const uint16_t COL_BORDER = 0x2B4E; // Calm steel-teal normal borders
static const uint16_t COL_LABEL  = 0x87FF; // Soft ice-cyan sensor names
static const uint16_t COL_UNIT   = 0x6B6D; // Muted slate units & hints
static const uint16_t COL_OK_BG  = 0x0264; // Deep teal-green OK pill
static const uint16_t COL_OK_FG  = 0x57EA; // Soft mint-emerald OK text
static const uint16_t COL_WARN   = 0xFD20; // Warm amber (warmup / T1 warning)

static const float JOY_ENGAGE  = 0.50f;
static const float JOY_RELEASE = 0.25f;
static const unsigned long JOY_POLL_MS         = 30;
static const unsigned long FLASH_MS            = 350;
static const unsigned long SETTINGS_TIMEOUT_MS = 30000UL;
static const unsigned long BOOT_REFRESH_MS     = 250UL;
static const unsigned long BOOT_MIN_MS         = 3000UL;
static const unsigned long BOOT_MAX_MS         = 26000UL; // Covers 20s SGP30 warm-up + first 5s display window

static const int BUZZ_FREQ = 2500;
static const int BUZZER_CH = 4; // Avoids LEDC ch 0 (MQ-7 heater)
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
static bool warningActive      = false;
static bool alertActiveView    = false;
static bool flashPhase         = false;
static bool needsFullRedraw    = true;
static bool lastWifiDrawn      = false;
static int  lastFaultDrawn     = -1;
static bool lastMutedDrawn     = false;
static uint16_t lastCardBorder[6] = {0};
static uint16_t lastCardCol[6]    = {0};
static char lastCardText[6][12]   = {{0}};

static unsigned long lastJoyPollMs  = 0;
static unsigned long lastFlashMs    = 0;
static unsigned long lastInputMs    = 0;
static unsigned long lastBootTickMs = 0;
static unsigned long testBtnUntilMs = 0;

// Boot checklist dirty cache (zero-flicker 4 Hz updates)
static char     lastBootStat[5][8] = {{0}};
static uint16_t lastBootCol[5]     = {0};
static int      lastBootBarPx      = -1;
static int      lastBootMsgKey     = -1;

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
  static void buzzerOn(int d) { ledcWrite(BUZZER_CH, d); }
  static void buzzerOff()     { ledcWrite(BUZZER_CH, 0); }
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

  // Priority 1: Life-safety Danger alarm (always overrides all pages, RTM R3)
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
  const AverageData* s[2] = {&data.coPPM, &data.eco2PPM};
  const float t1[2]       = {C0_THRESHOLD1_PPM, ECO2_THRESHOLD1_PPM};

  warningActive = false;
  for (int i = 0; i < 2; i++) {
    if (!s[i]->valid || !isfinite(s[i]->average)) continue;
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
  lcd.setCursor(cx - (int)w / 2, y);
  lcd.print(s);
}

static void drawHeaderBadges(int fCount) {
  lastFaultDrawn = fCount;
  lastWifiDrawn  = wifiConnected;

  // Show red FLT:N badge only when a fault is active
  lcd.fillRect(128, 4, 46, 22, COL_HEADER);
  if (fCount > 0) {
    char buf[16];
    snprintf(buf, sizeof(buf), "FLT:%d", fCount);
    lcd.fillRoundRect(130, 5, 42, 20, 4, ST77XX_RED);
    printCentered(buf, 151, 11, 1, ST77XX_WHITE, ST77XX_RED);
  }

  uint16_t pillBg = wifiConnected ? COL_OK_BG : ST77XX_RED;
  uint16_t pillFg = wifiConnected ? COL_OK_FG : ST77XX_YELLOW;
  lcd.fillRoundRect(178, 5, 56, 20, 4, pillBg);
  printCentered(wifiConnected ? "WiFi:OK" : "WiFi:DC", 206, 11, 1, pillFg, pillBg);
}

static void drawHeader(const char* title, bool showBadges) {
  lcd.fillRect(0, 0, SCR_W, 30, COL_HEADER);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_WHITE, COL_HEADER);
  lcd.setCursor(8, 7);
  lcd.print(title);
  if (showBadges) drawHeaderBadges(activeFaultCount());
}

// ============================================================
// SCREEN 0: NON-BLOCKING LIVE SELF-TEST BOOT SCREEN
// ============================================================
static bool allBootSensorsReady(const ProcessedData &data) {
  bool coReady  = (data.coPPM.valid && isfinite(data.coPPM.average)) || isCOInWarmup();
  bool sgpReady = data.eco2PPM.valid && isfinite(data.eco2PPM.average) &&
                  data.tvocPPB.valid && isfinite(data.tvocPPB.average);
  bool pmReady  = data.pm25UGM3.valid && isfinite(data.pm25UGM3.average);
  bool thReady  = data.temperatureC.valid && isfinite(data.temperatureC.average) &&
                  data.humidityPercent.valid && isfinite(data.humidityPercent.average);
  return coReady && sgpReady && pmReady && thReady;
}

static void drawBootPage(bool full, const ProcessedData &data) {
  unsigned long now = millis();

  if (full) {
    lcd.fillScreen(ST77XX_BLACK);

    // Brand header band
    lcd.fillRect(0, 0, SCR_W, 62, COL_HEADER);
    lcd.fillRect(0, 62, SCR_W, 2, COL_LABEL);
    printCentered("HARMFUL GAS", 120, 10, 2, ST77XX_WHITE, COL_HEADER);
    printCentered("MONITOR",     120, 30, 2, COL_LABEL,    COL_HEADER);
    printCentered("ELEC 290  -  TEAM 01", 120, 50, 1, COL_UNIT, COL_HEADER);

    // Checklist frame & static sensor names
    lcd.drawRoundRect(10, 74, 220, 154, 6, COL_BORDER);
    lcd.setTextSize(1);
    lcd.setTextColor(COL_LABEL, ST77XX_BLACK);
    lcd.setCursor(18, 82);
    lcd.print("SYSTEM SELF-TEST");

    static const char* labels[5] = {"MQ-7 CO   ", "SGP30 Gas ", "PMS7003   ", "DHT22 T/H ", "Wi-Fi     "};
    lcd.setTextSize(2);
    lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    for (int i = 0; i < 5; i++) {
      lcd.setCursor(18, 100 + i * 24);
      lcd.print(labels[i]);
      lastBootStat[i][0] = '\0';
      lastBootCol[i]     = 0;
    }

    // Progress bar outline & footer
    lcd.drawRoundRect(16, 262, 208, 18, 4, COL_BORDER);
    printCentered("Click joystick to skip", 120, 298, 1, COL_UNIT, ST77XX_BLACK);

    lastBootBarPx  = -1;
    lastBootMsgKey = -1;
  }

  // Compute live status badge for each subsystem
  char     stat[5][8];
  uint16_t col[5];

  // 0: MQ-7 (CO)
  if (rawData.coPPM.valid) {
    snprintf(stat[0], sizeof(stat[0]), "[ OK ]"); col[0] = COL_OK_FG;
  } else if (isCOInWarmup()) {
    snprintf(stat[0], sizeof(stat[0]), "[WARM]"); col[0] = COL_WARN;
  } else {
    snprintf(stat[0], sizeof(stat[0]), "[WAIT]"); col[0] = COL_WARN;
  }

  // 1: SGP30 (eCO2 / TVOC)
  if (rawData.eco2PPM.valid && rawData.tvocPPB.valid) {
    snprintf(stat[1], sizeof(stat[1]), "[ OK ]"); col[1] = COL_OK_FG;
  } else if (faults.sensor.status) {
    snprintf(stat[1], sizeof(stat[1]), "[FAIL]"); col[1] = ST77XX_RED;
  } else if (isSGP30InWarmup()) {
    int rem = 20 - (int)(now / 1000UL);
    if (rem < 1) rem = 1;
    snprintf(stat[1], sizeof(stat[1]), "[%2ds ]", rem); col[1] = COL_WARN;
  } else {
    snprintf(stat[1], sizeof(stat[1]), "[WAIT]"); col[1] = COL_WARN;
  }

  // 2: PMS7003 (PM2.5)
  if (rawData.pm25UGM3.valid) {
    snprintf(stat[2], sizeof(stat[2]), "[ OK ]"); col[2] = COL_OK_FG;
  } else {
    snprintf(stat[2], sizeof(stat[2]), "[WAIT]"); col[2] = COL_WARN;
  }

  // 3: DHT22 (Temp / Hum)
  if (rawData.temperatureC.valid && rawData.humidityPercent.valid) {
    snprintf(stat[3], sizeof(stat[3]), "[ OK ]"); col[3] = COL_OK_FG;
  } else {
    snprintf(stat[3], sizeof(stat[3]), "[WAIT]"); col[3] = COL_WARN;
  }

  // 4: Wi-Fi
  if (wifiConnected) {
    snprintf(stat[4], sizeof(stat[4]), "[ OK ]"); col[4] = COL_OK_FG;
  } else {
    snprintf(stat[4], sizeof(stat[4]), "[WAIT]"); col[4] = COL_WARN;
  }

  // Redraw only changed status tokens (zero flicker)
  lcd.setTextSize(2);
  for (int i = 0; i < 5; i++) {
    if (full || col[i] != lastBootCol[i] || strcmp(stat[i], lastBootStat[i]) != 0) {
      lastBootCol[i] = col[i];
      strncpy(lastBootStat[i], stat[i], sizeof(lastBootStat[i]) - 1);
      lastBootStat[i][sizeof(lastBootStat[i]) - 1] = '\0';
      lcd.setTextColor(col[i], ST77XX_BLACK);
      lcd.setCursor(138, 100 + i * 24);
      lcd.print(stat[i]);
    }
  }

  // Status caption above progress bar
  bool rawAllOk = (rawData.coPPM.valid || isCOInWarmup()) &&
                  rawData.eco2PPM.valid && rawData.tvocPPB.valid &&
                  rawData.pm25UGM3.valid &&
                  rawData.temperatureC.valid && rawData.humidityPercent.valid;
  int msgKey = allBootSensorsReady(data) ? 2 : (rawAllOk ? 1 : 0);
  if (full || msgKey != lastBootMsgKey) {
    lastBootMsgKey = msgKey;
    const char* msg = (msgKey == 2) ? "   All sensors ready!   "
                    : (msgKey == 1) ? " Finalizing readings... "
                                    : " Warming up sensors...  ";
    printCentered(msg, 120, 244, 1, (msgKey == 2) ? COL_OK_FG : COL_LABEL, ST77XX_BLACK);
  }

  // Smooth flicker-free progress bar (inner 204 x 14 px)
  const int barMax = 204;
  int fillPx = allBootSensorsReady(data)
               ? barMax
               : (int)min((unsigned long)barMax, (now * (unsigned long)barMax) / BOOT_MAX_MS);
  if (fillPx != lastBootBarPx) {
    if (fillPx > 0)      lcd.fillRect(18, 264, fillPx, 14, COL_LABEL);
    if (fillPx < barMax) lcd.fillRect(18 + fillPx, 264, barMax - fillPx, 14, ST77XX_BLACK);
    lastBootBarPx = fillPx;
  }
}

// ============================================================
// SCREEN 1: CLASSIC 2x3 CARD GRID (Expanded, Pure Black, No Banner)
// ============================================================
static void drawDashboard(const ProcessedData &data, bool full) {
  int fCount = activeFaultCount();
  bool muted = dangerAlertActive && uiState.alertMutedByJoystick;

  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("AIR MON", true);
    for (int i = 0; i < 6; i++) {
      lastCardBorder[i]  = 0;
      lastCardCol[i]     = 0;
      lastCardText[i][0] = '\0';
    }
    lastMutedDrawn = !muted;
  } else if (fCount != lastFaultDrawn || wifiConnected != lastWifiDrawn) {
    drawHeaderBadges(fCount);
  }

  const AverageData* s[6] = {
    &data.coPPM, &data.tvocPPB, &data.eco2PPM,
    &data.temperatureC, &data.humidityPercent, &data.pm25UGM3
  };
  const char* names[6]  = {"CO", "TVOC", "eCO2", "Temp", "Hum", "PM2.5"};
  const char* units[6]  = {"ppm", "ppb", "ppm", "C", "%", "ug/m3"};
  const bool  useDec[6] = {true, false, false, true, false, false};
  const float t1[6]     = {C0_THRESHOLD1_PPM, 0.0f, ECO2_THRESHOLD1_PPM, 0.0f, 0.0f, 0.0f};
  const float t2[6]     = {C0_THRESHOLD2_PPM, 0.0f, ECO2_THRESHOLD2_PPM, 0.0f, 0.0f, 0.0f};

  // 2x3 cards: 112 x 84 px each (uses space freed by removing the bottom banner)
  char buf[16];
  for (int i = 0; i < 6; i++) {
    int x = 6 + (i % 2) * 116;
    int y = 36 + (i / 2) * 90;

    bool ok     = s[i]->valid && isfinite(s[i]->average);
    bool warmup = !ok && ((i == 0 && isCOInWarmup()) || ((i == 1 || i == 2) && isSGP30InWarmup()));

    uint16_t border = COL_BORDER;
    uint16_t valCol = ST77XX_WHITE;
    if (warmup)                                      { border = COL_WARN;   valCol = COL_WARN;   }
    else if (!ok)                                    { border = ST77XX_RED; valCol = ST77XX_RED; }
    else if (t2[i] > 0.0f && s[i]->average >= t2[i]) { border = ST77XX_RED; valCol = ST77XX_RED; }
    else if (t1[i] > 0.0f && s[i]->average >= t1[i]) { border = COL_WARN;   valCol = COL_WARN;   }

    if (full || border != lastCardBorder[i]) {
      lastCardBorder[i] = border;
      lcd.drawRoundRect(x, y, 112, 84, 6, border);
    }

    if (full) {
      lcd.setTextSize(2);
      lcd.setTextColor(COL_LABEL, ST77XX_BLACK);
      lcd.setCursor(x + 8, y + 8);
      lcd.print(names[i]);

      lcd.setTextSize(1);
      lcd.setTextColor(COL_UNIT, ST77XX_BLACK);
      lcd.setCursor(x + 8, y + 28);
      lcd.print(units[i]);
    }

    if (ok) {
      if (useDec[i]) snprintf(buf, sizeof(buf), "%-7.1f", s[i]->average);
      else           snprintf(buf, sizeof(buf), "%-7.0f", s[i]->average);
    } else if (warmup) {
      snprintf(buf, sizeof(buf), "%-7s", "WARMUP");
    } else {
      snprintf(buf, sizeof(buf), "%-7s", "FAULT");
    }

    if (full || valCol != lastCardCol[i] || strcmp(buf, lastCardText[i]) != 0) {
      lastCardCol[i] = valCol;
      strncpy(lastCardText[i], buf, sizeof(lastCardText[i]) - 1);
      lastCardText[i][sizeof(lastCardText[i]) - 1] = '\0';
      lcd.setTextSize(2);
      lcd.setTextColor(valCol, ST77XX_BLACK);
      lcd.setCursor(x + 8, y + 50);
      lcd.print(buf);
    }
  }

  // Slim 1-line footer hint (pure black, no bulky colored banner)
  if (full || muted != lastMutedDrawn) {
    lastMutedDrawn = muted;
    lcd.fillRect(0, 306, SCR_W, 14, ST77XX_BLACK);
    if (muted) {
      printCentered("ALARM SILENCED (DANGER ACTIVE)", 120, 308, 1, ST77XX_RED, ST77XX_BLACK);
    } else {
      printCentered("L/R: Faults   Click: Buzzer", 120, 308, 1, COL_UNIT, ST77XX_BLACK);
    }
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
// SCREEN 2: TOP-LEVEL FAULTS & SYSTEM STATUS (Pure Black, Size 2 Faults)
// ============================================================
static void drawSystemPage(bool full) {
  int fCount = activeFaultCount();

  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("FAULTS & SYS", false);

    lcd.setTextSize(1);
    lcd.setTextColor(COL_LABEL, ST77XX_BLACK);
    lcd.setCursor(16, 44);
    lcd.print("ACTIVE FAULTS");

    lcd.setTextColor(COL_LABEL, ST77XX_BLACK);
    lcd.setCursor(16, 208);
    lcd.print("STORAGE & SYNC");

    printCentered("L/R: Dashboard   Click: Buzzer", 120, 304, 1, COL_UNIT, ST77XX_BLACK);
  }

  lcd.drawRoundRect(8, 36, 224, 156, 6, fCount ? ST77XX_RED : COL_BORDER);
  lcd.drawRoundRect(8, 200, 224, 92, 6, faults.storage.status ? ST77XX_RED : COL_BORDER);

  const char* fNames[5] = {"Sensor", "Inval ", "Stale ", "Store ", "Wi-Fi "};
  const bool  fStat[5]  = {faults.sensor.status, faults.readingInvalid.status,
                           faults.readingStale.status, faults.storage.status, faults.wifi.status};
  const int   fNum[5]   = {faults.sensor.num, faults.readingInvalid.num,
                           faults.readingStale.num, faults.storage.num, faults.wifi.num};

  char buf[42];
  lcd.setTextSize(2);
  for (int i = 0; i < 5; i++) {
    lcd.setCursor(16, 60 + i * 25);
    lcd.setTextColor(fStat[i] ? ST77XX_RED : COL_OK_FG, ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "%s:%-3s %3d", fNames[i], fStat[i] ? "FLT" : "OK", fNum[i]);
    lcd.print(buf);
  }

  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd.setCursor(16, 226);
  snprintf(buf, sizeof(buf), "File: %-18s (#%u)", getLogFilePath(true), activeLogFileIndex);
  lcd.print(buf);

  lcd.setCursor(16, 246);
  snprintf(buf, sizeof(buf), "Entries: %-5u Unsent: %-5u", totalLogEntries, unsentLogEntries);
  lcd.print(buf);

  lcd.setCursor(16, 266);
  lcd.setTextColor(wifiConnected ? COL_OK_FG : COL_WARN, ST77XX_BLACK);
  snprintf(buf, sizeof(buf), "WiFi: %-3s  |  NTP: %-10s",
           wifiConnected ? "OK" : "DC", timeSyncedThisBoot ? "SYNCED" : "BOOT SEC");
  lcd.print(buf);
}

// ============================================================
// SCREEN 3: BUZZER VOLUME PAGE (Pure Black, Flicker-Free)
// ============================================================
static void drawSliderBar(int value, int maxVal, bool selected) {
  const int barX = 24, barY = 62, barW = 192, barH = 22;
  const int innerX = barX + 2, innerY = barY + 2, innerW = barW - 4, innerH = barH - 4;
  int fillW = (innerW * constrain(value, 0, maxVal)) / maxVal;

  lcd.drawRoundRect(barX, barY, barW, barH, 5, ST77XX_WHITE);
  if (fillW > 0)      lcd.fillRect(innerX, innerY, fillW, innerH, selected ? COL_LABEL : ST77XX_WHITE);
  if (fillW < innerW) lcd.fillRect(innerX + fillW, innerY, innerW - fillW, innerH, ST77XX_BLACK);
}

static void drawBuzzerPage(bool full) {
  if (full) {
    lcd.fillScreen(ST77XX_BLACK);
    drawHeader("BUZZER", false);
    printCentered("U/D: Row  L/R: Vol  Click: OK", 120, 302, 1, COL_UNIT, ST77XX_BLACK);
  }

  bool s0 = (uiState.subRow == 0);
  lcd.drawRoundRect(12, 48, 216, 86, 8, s0 ? COL_LABEL : COL_BORDER);
  drawSliderBar(buzzerVolume, 10, s0);

  char buf[16];
  if (buzzerVolume == 0) snprintf(buf, sizeof(buf), " MUTED ");
  else                   snprintf(buf, sizeof(buf), " %2d/10 ", buzzerVolume);
  printCentered(buf, 120, 96, 3, ST77XX_WHITE, ST77XX_BLACK);

  bool s1 = (uiState.subRow == 1);
  bool hi = (testBtnUntilMs > 0);
  if (full || hi) {
    lcd.fillRoundRect(14, 154, 212, 50, 6, hi ? COL_LABEL : ST77XX_BLACK);
  }
  lcd.drawRoundRect(12, 152, 216, 54, 8, s1 ? COL_LABEL : COL_BORDER);
  printCentered(hi ? (buzzerVolume ? "BEEPING..." : "  MUTED   ") : "TEST SOUND",
                120, 171, 2, hi ? ST77XX_BLACK : (s1 ? COL_LABEL : ST77XX_WHITE),
                hi ? COL_LABEL : ST77XX_BLACK);

  bool s2 = (uiState.subRow == 2);
  lcd.drawRoundRect(12, 222, 216, 54, 8, s2 ? COL_LABEL : COL_BORDER);
  printCentered("SAVE & BACK", 120, 241, 2, s2 ? COL_LABEL : ST77XX_WHITE, ST77XX_BLACK);
}

// ============================================================
// PUBLIC DISPLAY & INPUT HANDLERS
// ============================================================
void renderDisplay(const ProcessedData &data) {
  updateVisualGasState(data);

  // Life-safety override: immediately exit any page on Danger (RTM R3)
  if (dangerAlertActive && !uiState.alertMutedByJoystick) {
    if (uiState.currentScreen != PAGE_DASHBOARD) {
      uiPrefs.putInt("vol", buzzerVolume);
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
    case BOOT_PAGE:      drawBootPage(needsFullRedraw, data);  break;
    case PAGE_DASHBOARD: drawDashboard(data, needsFullRedraw); break;
    case SYSTEM_PAGE:    drawSystemPage(needsFullRedraw);      break;
    case BUZZER_PAGE:    drawBuzzerPage(needsFullRedraw);      break;
  }
  needsFullRedraw = false;
}

static void switchScreen(DisplayScreen next, const ProcessedData &data) {
  uiState.currentScreen = next;
  if (next == BUZZER_PAGE) uiState.subRow = 0;
  needsFullRedraw = true;
  renderDisplay(data);
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

  // 2. Non-blocking live boot screen refresh & smart auto-exit
  if (!alertActiveView && uiState.currentScreen == BOOT_PAGE) {
    if ((allBootSensorsReady(data) && now >= BOOT_MIN_MS) || (now >= BOOT_MAX_MS)) {
      switchScreen(PAGE_DASHBOARD, data);
      return;
    }
    if (now - lastBootTickMs >= BOOT_REFRESH_MS) {
      lastBootTickMs = now;
      drawBootPage(false, data);
    }
  }

  // 3. Reset TEST SOUND button highlight when done
  if (testBtnUntilMs > 0 && now >= testBtnUntilMs) {
    testBtnUntilMs = 0;
    if (uiState.currentScreen == BUZZER_PAGE) {
      lcd.fillRoundRect(14, 154, 212, 50, 6, ST77XX_BLACK);
      drawBuzzerPage(false);
    }
  }

  // 4. Auto-exit Buzzer page after 30 s inactivity
  if (uiState.currentScreen == BUZZER_PAGE && (now - lastInputMs > SETTINGS_TIMEOUT_MS)) {
    uiPrefs.putInt("vol", buzzerVolume);
    switchScreen(PAGE_DASHBOARD, data);
    return;
  }

  // 5. Throttle ADC/button polling to 30 ms (~33 Hz)
  if (now - lastJoyPollMs < JOY_POLL_MS) return;
  lastJoyPollMs = now;

  bool allowRepeat = (uiState.currentScreen == BUZZER_PAGE && uiState.subRow == 0);
  bool click = readButtonClick(now);
  int  dir   = readDirection(now, allowRepeat);
  if (!click && dir == DIR_NONE) return;
  lastInputMs = now;

  if (uiState.currentScreen == BOOT_PAGE) {
    if (click && now >= 500UL) {
      switchScreen(PAGE_DASHBOARD, data);
    }
  } else if (uiState.currentScreen == PAGE_DASHBOARD) {
    if (alertActiveView) {
      if (click) {
        uiState.alertMutedByJoystick = true;
        renderDisplay(data);
      }
      return;
    }
    if (dir == DIR_LEFT || dir == DIR_RIGHT) switchScreen(SYSTEM_PAGE, data);
    else if (click)                          switchScreen(BUZZER_PAGE, data);
  } else if (uiState.currentScreen == SYSTEM_PAGE) {
    if (dir == DIR_LEFT || dir == DIR_RIGHT) switchScreen(PAGE_DASHBOARD, data);
    else if (click)                          switchScreen(BUZZER_PAGE, data);
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
      uiPrefs.putInt("vol", buzzerVolume);
      switchScreen(PAGE_DASHBOARD, data);
    } else if (click) {
      if (uiState.subRow == 0) {
        oneShotMs = 140;
      } else if (uiState.subRow == 1) {
        if (buzzerVolume > 0) testBeepsLeft = 3;
        testBtnUntilMs = now + 1000;
        drawBuzzerPage(false);
      } else {
        uiPrefs.putInt("vol", buzzerVolume);
        switchScreen(PAGE_DASHBOARD, data);
      }
    }
  }
}
