#include "local_communication.h"

// ============================================================
// EXTERNAL REFERENCES FROM MAIN.CPP
// ============================================================
extern const int I2C_SDA_SCREEN_PIN;
extern const int I2C_SCL_SCREEN_PIN;
extern const int SCREEN_CS_PIN;
extern const int SCREEN_DC_PIN;
extern const int SCREEN_RST_PIN;

extern const int JOY_X_PIN;
extern const int JOY_Y_PIN;
extern const int JOY_SW_PIN;
extern const int BUZZER_PIN;

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
// MODULE OBJECTS & CONSTANTS
// ============================================================
Adafruit_ST7789 lcd = Adafruit_ST7789(SCREEN_CS_PIN, SCREEN_DC_PIN, I2C_SDA_SCREEN_PIN, I2C_SCL_SCREEN_PIN, SCREEN_RST_PIN);
UIState uiState = {};

const unsigned long JOY_POLL_INTERVAL = 50;  // Check joystick every 50 ms
const unsigned long JOY_MOVE_COOLDOWN = 250; // 250 ms between menu steps
const int JOY_LOW_THRESHOLD           = 1000;
const int JOY_HIGH_THRESHOLD          = 3000;

const unsigned long BUZZER_BEEP_INTERVAL = 400; // Non-blocking 400 ms pulsing alarm
unsigned long lastBuzzerToggle           = 0;
bool buzzerPinState                      = false;
bool lastButtonState                     = HIGH;

const char* SENSOR_NAMES[6] = {"CO", "TVOC", "eCO2", "Temp", "Hum", "PM2.5"};
const char* SENSOR_UNITS[6] = {"ppm", "ppb", "ppm", "C", "%", "ug/m3"};

// ============================================================
// INITIALIZATION FUNCTIONS
// ============================================================

// Initializes the joystick pins and the 2.0" 240x320 ST7789 SPI display
void initDisplayAndJoystickHardware() {
  Serial.println("Initializing 2.0\" ST7789 LCD and Joystick...");
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  lcd.init(240, 320);
  lcd.setRotation(1); // Landscape: 320px wide x 240px tall
  lcd.fillScreen(ST77XX_BLACK);

  lcd.setTextColor(ST77XX_WHITE);
  lcd.setTextSize(2);
  lcd.setCursor(55, 110);
  lcd.print("Gas Monitor Init");
  uiState.needsFullRedraw = true;
}

// Initializes the buzzer pin and ensures it starts LOW
void initBuzzerHardware() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

// ============================================================
// BUZZER ALERT LOGIC
// ============================================================

// Drives the buzzer with a non-blocking pulsing pattern when dangerAlertActive is true
void refreshBuzzerState(unsigned long currentTime) {
  if (!dangerAlertActive) {
    uiState.alertMutedByJoystick = false; // Reset mute once air quality returns to normal
    buzzerPinState = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // If the user pressed the joystick button to acknowledge/mute the alarm, keep buzzer off
  if (uiState.alertMutedByJoystick) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // Toggle the buzzer every 400 ms for a noticeable pulsing alarm
  if (currentTime - lastBuzzerToggle >= BUZZER_BEEP_INTERVAL) {
    lastBuzzerToggle = currentTime;
    buzzerPinState = !buzzerPinState;
    digitalWrite(BUZZER_PIN, buzzerPinState ? HIGH : LOW);
  }
}

// ============================================================
// INTERNAL SCREEN DRAWING HELPERS
// ============================================================

// Draws the top 28px bar showing Wi-Fi status (R6) and active alert/fault banner
static void drawTopStatusBar() {
  uint16_t barColor = dangerAlertActive ? ST77XX_RED : (faults.sensor.status ? 0xFD20 : 0x18E3);
  lcd.fillRect(0, 0, 320, 28, barColor);

  lcd.setTextSize(2);
  lcd.setCursor(6, 6);
  lcd.setTextColor(wifiConnected ? ST77XX_GREEN : ST77XX_YELLOW, barColor);
  lcd.print(wifiConnected ? "WiFi:OK" : "WiFi:DC");

  lcd.setCursor(115, 6);
  lcd.setTextColor(ST77XX_WHITE, barColor);
  char alertBuf[18];
  snprintf(alertBuf, sizeof(alertBuf), "%-16s", activeAlertMessage.c_str());
  lcd.print(alertBuf);
}

// Draws the bottom 20px navigation bar
static void drawBottomNavBar() {
  lcd.fillRect(0, 220, 320, 20, 0x18E3);
  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_CYAN, 0x18E3);
  lcd.setCursor(8, 226);

  if (uiState.currentPage == PAGE_OVERVIEW) {
    lcd.print("< [1/3 OVERVIEW] >   U/D:Select  Click:Detail");
  } else if (uiState.currentPage == PAGE_DETAIL) {
    lcd.print("< [2/3 DETAIL] >     U/D:Sensor  Click:Back  ");
  } else {
    lcd.print("< [3/3 SYSTEM] >     L/R:Switch Page         ");
  }
}

// Page 0: Draws a 2-column x 3-row grid of all 6 sensors
static void drawOverviewPage(const ProcessedData &data, bool fullRedraw) {
  const AverageData* sensors[6] = {
    &data.coPPM, &data.tvocPPB, &data.eco2PPM,
    &data.temperatureC, &data.humidityPercent, &data.pm25UGM3
  };

  for (uint8_t i = 0; i < 6; i++) {
    int col = i % 2;
    int row = i / 2;
    int x = 6 + col * 156;
    int y = 34 + row * 60;

    bool selected = (i == uiState.selectedSensor);
    uint16_t borderColor = selected ? ST77XX_CYAN : 0x4208;

    if (fullRedraw) {
      lcd.fillRect(x, y, 150, 54, ST77XX_BLACK);
    }
    lcd.drawRect(x, y, 150, 54, borderColor);

    // Sensor title and unit
    lcd.setTextSize(1);
    lcd.setTextColor(selected ? ST77XX_CYAN : ST77XX_WHITE, ST77XX_BLACK);
    lcd.setCursor(x + 6, y + 6);
    char labelBuf[20];
    snprintf(labelBuf, sizeof(labelBuf), "%s (%s)", SENSOR_NAMES[i], SENSOR_UNITS[i]);
    lcd.print(labelBuf);

    // Sensor 5-second average value (or "--" if invalid)
    lcd.setTextSize(2);
    lcd.setCursor(x + 6, y + 24);
    char valBuf[12];
    if (sensors[i]->valid && isfinite(sensors[i]->average)) {
      lcd.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      snprintf(valBuf, sizeof(valBuf), "%-8.1f", sensors[i]->average);
    } else {
      lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      snprintf(valBuf, sizeof(valBuf), "%-8s", "--");
    }
    lcd.print(valBuf);
  }
}

// Page 1: Zoomed-in view of a single selected sensor
static void drawDetailPage(const ProcessedData &data, bool fullRedraw) {
  const AverageData* sensors[6] = {
    &data.coPPM, &data.tvocPPB, &data.eco2PPM,
    &data.temperatureC, &data.humidityPercent, &data.pm25UGM3
  };
  const float thresh1[6] = {C0_THRESHOLD1_PPM, TVOC_THRESHOLD1_PPB, ECO2_THRESHOLD1_PPM, 0, 0, PM25_THRESHOLD1_UGM3};
  const float thresh2[6] = {C0_THRESHOLD2_PPM, TVOC_THRESHOLD2_PPB, ECO2_THRESHOLD2_PPM, 0, 0, PM25_THRESHOLD2_UGM3};

  uint8_t idx = uiState.selectedSensor;
  const AverageData* s = sensors[idx];

  if (fullRedraw) {
    lcd.fillRect(0, 28, 320, 192, ST77XX_BLACK);
    lcd.drawRect(8, 36, 304, 176, ST77XX_CYAN);
  }

  // Sensor Title
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.setCursor(20, 48);
  char titleBuf[24];
  snprintf(titleBuf, sizeof(titleBuf), "%s (%s)      ", SENSOR_NAMES[idx], SENSOR_UNITS[idx]);
  lcd.print(titleBuf);

  // Large Value Readout
  lcd.setTextSize(4);
  lcd.setCursor(20, 82);
  char valBuf[14];
  if (s->valid && isfinite(s->average)) {
    lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    snprintf(valBuf, sizeof(valBuf), "%-8.2f", s->average);
  } else {
    lcd.setTextColor(ST77XX_RED, ST77XX_BLACK);
    snprintf(valBuf, sizeof(valBuf), "%-8s", "INVALID");
  }
  lcd.print(valBuf);

  // Sample count, validity, and threshold info
  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd.setCursor(20, 140);
  char infoLine1[42];
  snprintf(infoLine1, sizeof(infoLine1), "5s Window Samples: %-4u | Status: %-7s",
           s->sampleCount, s->valid ? "VALID" : "FAULT");
  lcd.print(infoLine1);

  lcd.setCursor(20, 160);
  char infoLine2[42];
  if (thresh1[idx] > 0 || thresh2[idx] > 0) {
    snprintf(infoLine2, sizeof(infoLine2), "Thresholds -> T1: %.1f | T2: %.1f %-6s",
             thresh1[idx], thresh2[idx], SENSOR_UNITS[idx]);
  } else {
    snprintf(infoLine2, sizeof(infoLine2), "Thresholds -> Informational Indicator   ");
  }
  lcd.print(infoLine2);
}

// Page 2: System Diagnostics, Fault Counters, and Storage Sync Status
static void drawSystemPage(bool fullRedraw) {
  if (fullRedraw) {
    lcd.fillRect(0, 28, 320, 192, ST77XX_BLACK);
    lcd.setTextSize(2);
    lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    lcd.setCursor(12, 36);
    lcd.print("System & Fault Status");
  }

  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  char buf[48];
  lcd.setCursor(12, 66);
  snprintf(buf, sizeof(buf), "Active CSV : %-18s (#%u)", getLogFilePath(true), activeLogFileIndex);
  lcd.print(buf);

  lcd.setCursor(12, 84);
  snprintf(buf, sizeof(buf), "File Size  : %-7u B | Unsent: %-7u B",
           (unsigned int)currentLogFileSize,
           (currentLogFileSize > lastUploadedByteOffset) ? (unsigned int)(currentLogFileSize - lastUploadedByteOffset) : 0);
  lcd.print(buf);

  lcd.setCursor(12, 102);
  snprintf(buf, sizeof(buf), "NTP Clock  : %-12s", timeSyncedThisBoot ? "SYNCED (UTC)" : "OFFLINE (Boot s)");
  lcd.print(buf);

  lcd.setCursor(12, 126);
  lcd.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  lcd.print("--- FAULT STATUS & COUNTS ---");

  lcd.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  lcd.setCursor(12, 144);
  snprintf(buf, sizeof(buf), "Sensor : %s (%-3d) | Invalid: %s (%-3d)",
           faults.sensor.status ? "FAULT" : "OK   ", faults.sensor.num,
           faults.readingInvalid.status ? "FAULT" : "OK   ", faults.readingInvalid.num);
  lcd.print(buf);

  lcd.setCursor(12, 162);
  snprintf(buf, sizeof(buf), "Stale  : %s (%-3d) | Storage: %s (%-3d)",
           faults.readingStale.status ? "FAULT" : "OK   ", faults.readingStale.num,
           faults.storage.status ? "FAULT" : "OK   ", faults.storage.num);
  lcd.print(buf);

  lcd.setCursor(12, 180);
  snprintf(buf, sizeof(buf), "Wi-Fi  : %s (%-3d) | Buzzer : %-8s",
           faults.wifi.status ? "FAULT" : "OK   ", faults.wifi.num,
           dangerAlertActive ? (uiState.alertMutedByJoystick ? "MUTED" : "ACTIVE") : "OFF");
  lcd.print(buf);
}

// ============================================================
// PUBLIC DISPLAY & JOYSTICK FUNCTIONS
// ============================================================

// Renders the current UI page on the ST7789 screen (flicker-free in-place updates)
void renderDisplay(const ProcessedData &data) {
  if (uiState.needsFullRedraw) {
    lcd.fillRect(0, 28, 320, 192, ST77XX_BLACK);
    drawBottomNavBar();
  }

  drawTopStatusBar();

  switch (uiState.currentPage) {
    case PAGE_OVERVIEW:
      drawOverviewPage(data, uiState.needsFullRedraw);
      break;
    case PAGE_DETAIL:
      drawDetailPage(data, uiState.needsFullRedraw);
      break;
    case PAGE_SYSTEM:
      drawSystemPage(uiState.needsFullRedraw);
      break;
    default:
      break;
  }

  uiState.needsFullRedraw = false;
}

// Polls the analog joystick and pushbutton without blocking
void handleJoystickInput(unsigned long currentTime, const ProcessedData &data) {
  if (currentTime - uiState.lastJoyPoll < JOY_POLL_INTERVAL) return;
  uiState.lastJoyPoll = currentTime;

  bool screenChanged = false;

  // 1. Check Joystick Pushbutton (SW is active-LOW)
  bool btnState = digitalRead(JOY_SW_PIN);
  if (btnState == LOW && lastButtonState == HIGH && (currentTime - uiState.lastJoyMove >= JOY_MOVE_COOLDOWN)) {
    uiState.lastJoyMove = currentTime;

    if (dangerAlertActive && !uiState.alertMutedByJoystick) {
      // Pressing the button during an active alarm acknowledges/mutes the buzzer
      uiState.alertMutedByJoystick = true;
      digitalWrite(BUZZER_PIN, LOW);
      screenChanged = true;
    } else if (uiState.currentPage == PAGE_OVERVIEW) {
      uiState.currentPage     = PAGE_DETAIL;
      uiState.needsFullRedraw = true;
      screenChanged           = true;
    } else if (uiState.currentPage == PAGE_DETAIL) {
      uiState.currentPage     = PAGE_OVERVIEW;
      uiState.needsFullRedraw = true;
      screenChanged           = true;
    }
  }
  lastButtonState = btnState;

  // 2. Check Joystick X/Y Axes (with cooldown so one push moves one item)
  if (currentTime - uiState.lastJoyMove >= JOY_MOVE_COOLDOWN) {
    int xVal = analogRead(JOY_X_PIN);
    int yVal = analogRead(JOY_Y_PIN);

    // Left / Right switches pages
    if (xVal < JOY_LOW_THRESHOLD) {
      uiState.currentPage     = static_cast<DisplayPage>((uiState.currentPage + PAGE_COUNT - 1) % PAGE_COUNT);
      uiState.needsFullRedraw = true;
      uiState.lastJoyMove     = currentTime;
      screenChanged           = true;
    } else if (xVal > JOY_HIGH_THRESHOLD) {
      uiState.currentPage     = static_cast<DisplayPage>((uiState.currentPage + 1) % PAGE_COUNT);
      uiState.needsFullRedraw = true;
      uiState.lastJoyMove     = currentTime;
      screenChanged           = true;
    }
    // Up / Down selects sensors on Overview or Detail pages
    else if (yVal < JOY_LOW_THRESHOLD) {
      uiState.selectedSensor = (uiState.selectedSensor + 5) % 6;
      if (uiState.currentPage == PAGE_DETAIL) uiState.needsFullRedraw = true;
      uiState.lastJoyMove = currentTime;
      screenChanged       = true;
    } else if (yVal > JOY_HIGH_THRESHOLD) {
      uiState.selectedSensor = (uiState.selectedSensor + 1) % 6;
      if (uiState.currentPage == PAGE_DETAIL) uiState.needsFullRedraw = true;
      uiState.lastJoyMove = currentTime;
      screenChanged       = true;
    }
  }

  // Immediately redraw the screen when the user moves the joystick
  if (screenChanged) {
    renderDisplay(data);
  }
}