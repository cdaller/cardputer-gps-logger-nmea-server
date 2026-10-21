// GPS Logger for M5Stack Cardputer / Cardputer ADV
//
// Logs GPS positions to CSV and/or GPX files on the SD card. While logging
// with deep sleep enabled, the ESP32 sleeps between points: on each timer
// wake-up it only reads the GPS, writes one point and goes back to sleep
// without touching the display. Pressing G0 (BtnA) wakes up the full UI.

#include <M5Cardputer.h>
#include <SD.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <time.h>

#include "config.h"
#include "gps.h"
#include "logger.h"

static const gpio_num_t PIN_BACKLIGHT = GPIO_NUM_38;
static const gpio_num_t PIN_WAKE_BTN = GPIO_NUM_0;  // G0 / BtnA on Cardputer and ADV
static const int PIN_BATTERY = 10;
static const int PIN_ADV_LORA_CS = 5;               // SX1262 on CAP LoRa-1262 shares the SD SPI bus
static const uint8_t TCA8418_ADDR = 0x34;           // ADV keyboard controller

static const uint32_t MIN_SLEEP_INTERVAL_SEC = 15;  // below this, deep sleep is not worth it
static const uint32_t MIN_SLEEP_MS = 3000;
static const uint32_t RTC_MAGIC = 0x47504C31;

// Survives deep sleep.
struct RtcState {
  uint32_t magic;
  bool logging;
  bool isAdv;
  char baseName[48];
  uint32_t points;
  uint32_t detectedBaud;
};
RTC_DATA_ATTR RtcState rtc;

enum Page { PAGE_MAIN, PAGE_DETAILS, PAGE_MENU, PAGE_SETTINGS, PAGE_HELP };
static const int PAGE_CYCLE_COUNT = 3;  // MAIN, DETAILS, MENU are switched with Tab / G0

static Config cfg;
static GpsReader gps;
static M5Canvas canvas(&M5.Display);
static bool sdOk = false;
static bool hasKeyboard = false;
static Page page = PAGE_MAIN;
static Page returnPage = PAGE_MAIN;  // where settings / help go back to
static int menuSel = 0;
static uint32_t lastLogMs = 0;       // 0 = nothing logged since start/wake
static uint32_t lastInputMs = 0;
static uint32_t lastDrawMs = 0;
static bool screenOn = true;
static char statusMsg[40] = "";
static uint32_t statusMsgUntil = 0;
static int settingsSel = 0;
static Config cfgBeforeEdit;

// ---------------------------------------------------------------- colors

static const uint16_t COL_BG = TFT_BLACK;
static const uint16_t COL_BAR = 0x18E3;     // dark grey
static const uint16_t COL_LABEL = 0x8410;   // grey
static const uint16_t COL_POS = TFT_CYAN;
static const uint16_t COL_ALT = TFT_YELLOW;
static const uint16_t COL_TIME = TFT_WHITE;
static const uint16_t COL_NOFIX = 0xFB2C;   // soft red
static const uint16_t COL_SEL = 0x2A69;     // blue-ish selection

// ---------------------------------------------------------------- helpers

static int batteryPctFromMv(int mv) {
  return constrain((mv - 3300) * 100 / (4150 - 3300), 0, 100);
}

// Smoothed battery voltage: one ADC sample per second into an exponential moving
// average (~30 s time constant), since single readings jitter by tens of mV.
// The weight grows with the time since the last sample, so after a long gap
// (screen off, long log interval) the average jumps to the current reading.
static int batteryMvUi() {
  static uint32_t lastSampleMs = 0;
  static float avgMv = 0;
  uint32_t now = millis();
  uint32_t dt = now - lastSampleMs;
  if (avgMv == 0 || dt >= 1000) {
    lastSampleMs = now;
    int mv = M5.Power.getBatteryVoltage();
    avgMv = avgMv == 0 ? mv : avgMv + (mv - avgMv) * min(1.0f, dt / 30000.0f);
  }
  return (int)(avgMv + 0.5f);
}

static void showStatus(const char* msg, uint32_t ms = 2500) {
  strlcpy(statusMsg, msg, sizeof(statusMsg));
  statusMsgUntil = millis() + ms;
}

static bool canDeepSleep() {
  return rtc.logging && cfg.deepSleep && cfg.logIntervalSec >= MIN_SLEEP_INTERVAL_SEC && sdOk;
}

static uint32_t effectiveBaud() {
  return cfg.gpsBaud ? cfg.gpsBaud : rtc.detectedBaud;  // reuse auto-detected baud after wake
}

static void startGps() { gps.begin(cfg.gpsRx, cfg.gpsTx, effectiveBaud()); }

// UTC date/time from the GPS shifted by the configured offset.
static bool localTime(struct tm& out) {
  TinyGPSPlus& g = gps.gps();
  if (!g.time.isValid() || !g.date.isValid() || g.date.year() < 2020) return false;
  struct tm t = {};
  t.tm_year = g.date.year() - 1900;
  t.tm_mon = g.date.month() - 1;
  t.tm_mday = g.date.day();
  t.tm_hour = g.time.hour();
  t.tm_min = g.time.minute();
  t.tm_sec = g.time.second();
  time_t epoch = mktime(&t);  // TZ is UTC on the ESP32 unless set
  epoch += (time_t)cfg.tzOffsetHours * 3600;
  gmtime_r(&epoch, &out);
  return true;
}

static bool buildPoint(LogPoint& p, int batteryPct) {
  TinyGPSPlus& g = gps.gps();
  if (!gps.hasFix() || !g.date.isValid() || !g.time.isValid() || g.date.year() < 2020) return false;
  p.year = g.date.year();
  p.month = g.date.month();
  p.day = g.date.day();
  p.hour = g.time.hour();
  p.minute = g.time.minute();
  p.second = g.time.second();
  p.lat = g.location.lat();
  p.lon = g.location.lng();
  p.altM = g.altitude.isValid() ? g.altitude.meters() : 0;
  p.speedKmh = g.speed.isValid() ? g.speed.kmph() : 0;
  p.courseDeg = g.course.isValid() ? g.course.deg() : 0;
  p.hdop = g.hdop.isValid() ? g.hdop.hdop() : 0;
  p.sats = g.satellites.isValid() ? g.satellites.value() : 0;
  p.batteryPct = batteryPct;
  return true;
}

static bool logPoint(int batteryPct) {
  LogPoint p;
  if (!buildPoint(p, batteryPct)) return false;
  if (!logger::write(p, cfg.logFormat, rtc.baseName, sizeof(rtc.baseName))) {
    showStatus("SD write error!");
    return false;
  }
  rtc.points++;
  return true;
}

static void startLogging() {
  rtc.logging = true;
  rtc.baseName[0] = '\0';
  rtc.points = 0;
  lastLogMs = 0;
  showStatus(sdOk ? "Logging started" : "No SD card!");
}

static void stopLogging() {
  rtc.logging = false;
  showStatus("Logging stopped");
}

// ---------------------------------------------------------------- deep sleep

[[noreturn]] static void enterDeepSleep(uint32_t sleepMs, bool displayActive) {
  gps.end();
  SD.end();
  if (displayActive) {
    M5.Display.setBrightness(0);
    M5.Display.sleep();
  }
  // Keep the backlight off while sleeping and during quiet timer wake-ups.
  gpio_reset_pin(PIN_BACKLIGHT);
  gpio_set_direction(PIN_BACKLIGHT, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_BACKLIGHT, 0);
  gpio_hold_en(PIN_BACKLIGHT);
  gpio_deep_sleep_hold_en();

  rtc_gpio_pullup_en(PIN_WAKE_BTN);
  esp_sleep_enable_ext0_wakeup(PIN_WAKE_BTN, 0);
  esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  log_i("Deep sleep for %lu ms", (unsigned long)sleepMs);
  esp_deep_sleep_start();
}

static uint32_t remainingIntervalMs(uint32_t sinceLogMs) {
  uint32_t interval = cfg.logIntervalSec * 1000;
  return sinceLogMs >= interval ? 0 : interval - sinceLogMs;
}

// Timer wake-up while logging: read GPS, log one point, sleep again.
// Returns only if the user pressed G0 and wants the UI.
static void quietWakeCycle() {
  uint32_t start = millis();
  pinMode(PIN_WAKE_BTN, INPUT_PULLUP);
  if (rtc.isAdv) {
    pinMode(PIN_ADV_LORA_CS, OUTPUT);
    digitalWrite(PIN_ADV_LORA_CS, HIGH);
  }
  configDefaults(cfg, rtc.isAdv);
  sdOk = logger::sdBegin();
  if (!sdOk) enterDeepSleep(cfg.logIntervalSec * 1000, false);
  configLoad(cfg);
  if (!canDeepSleep()) return;  // config changed on the card, show UI
  startGps();

  bool logged = false;
  while (millis() - start < cfg.fixTimeoutSec * 1000UL) {
    gps.update();
    if (gps.baudLocked() && cfg.gpsBaud == 0) rtc.detectedBaud = gps.baud();
    if (digitalRead(PIN_WAKE_BTN) == LOW) return;
    if (gps.gps().location.isUpdated() && logPoint(batteryPctFromMv(analogReadMilliVolts(PIN_BATTERY) * 2))) {
      logged = true;
      break;
    }
    delay(5);
  }
  log_i("Quiet wake: %s after %lu ms", logged ? "logged" : "no fix", millis() - start);
  uint32_t sleepMs = remainingIntervalMs(millis() - start);
  enterDeepSleep(max(sleepMs, MIN_SLEEP_MS), false);
}

// ---------------------------------------------------------------- drawing

static void drawBattery(int x, int y) {
  int mv = batteryMvUi();
  int pct = batteryPctFromMv(mv);
  uint16_t col = pct > 50 ? TFT_GREEN : pct > 20 ? TFT_YELLOW : TFT_RED;
  canvas.drawRect(x, y + 3, 22, 11, TFT_WHITE);
  canvas.fillRect(x + 22, y + 6, 2, 5, TFT_WHITE);
  canvas.fillRect(x + 2, y + 5, 18 * pct / 100, 7, col);
  canvas.setTextColor(col);
  canvas.setTextDatum(top_right);
  canvas.drawString(String(pct) + "%", x - 3, y + 1);
  canvas.setTextDatum(top_left);
}

static uint16_t satColor(int sats) {
  return sats >= 7 ? TFT_GREEN : sats >= 4 ? TFT_YELLOW : TFT_RED;
}

static void drawStatusBar() {
  canvas.fillRect(0, 0, 240, 18, COL_BAR);
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(top_left);
  if (rtc.logging) {
    bool blink = (millis() / 500) % 2;
    canvas.fillCircle(7, 9, 5, blink ? TFT_RED : 0x7800);
    canvas.setTextColor(TFT_RED);
    canvas.drawString("REC " + String(rtc.points), 15, 1);
  } else {
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("IDLE", 3, 1);
  }

  bool gpsAlive = gps.lastDataMs() && millis() - gps.lastDataMs() < 5000;
  if (!gps.baudLocked()) {
    canvas.setTextColor(TFT_ORANGE);
    canvas.drawString("BAUD " + String(gps.baud()), 82, 1);
  } else if (!gpsAlive) {
    canvas.setTextColor(TFT_RED);
    canvas.drawString("NO GPS", 88, 1);
  } else {
    int used = gps.gps().satellites.value();
    canvas.setTextColor(satColor(used));
    canvas.drawString("SAT " + String(used) + "/" + String(gps.satsInView()), 82, 1);
  }

  canvas.setTextColor(sdOk ? TFT_GREEN : TFT_RED);
  canvas.drawString(sdOk ? "SD" : "NOSD", 150, 1);
  drawBattery(214, 0);
}

// Small label on the left, large value right of it.
static void drawRow(int y, const char* label, const String& value, uint16_t color) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString(label, 3, y + 9);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(color);
  canvas.drawString(value, 30, y);
}

static void drawMainPage() {
  TinyGPSPlus& g = gps.gps();
  bool fix = gps.hasFix();
  char buf[32];

  if (fix) {
    double lat = g.location.lat(), lon = g.location.lng();
    snprintf(buf, sizeof(buf), "%.6f %c", fabs(lat), lat >= 0 ? 'N' : 'S');
    drawRow(22, "LAT", buf, COL_POS);
    snprintf(buf, sizeof(buf), "%.6f %c", fabs(lon), lon >= 0 ? 'E' : 'W');
    drawRow(50, "LON", buf, COL_POS);
  } else {
    drawRow(22, "LAT", "-- no fix --", COL_NOFIX);
    drawRow(50, "LON", "--", COL_NOFIX);
  }

  if (fix && g.altitude.isValid()) snprintf(buf, sizeof(buf), "%.0f m", g.altitude.meters());
  else strcpy(buf, "--");
  drawRow(78, "ALT", buf, fix ? COL_ALT : COL_NOFIX);

  // Satellites used in fix, large, right side of the altitude row
  int used = g.satellites.value();
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("SAT", 150, 87);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(satColor(used));
  canvas.drawString(String(used), 172, 78);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("/" + String(gps.satsInView()), 172 + canvas.textWidth("00", &fonts::Font4), 84);

  struct tm t;
  if (localTime(t)) {
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    drawRow(106, "TIME", buf, COL_TIME);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_LABEL);
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    canvas.drawString(buf, 160, 110);
    snprintf(buf, sizeof(buf), "UTC%+d", cfg.tzOffsetHours);
    canvas.drawString(buf, 160, 121);
  } else {
    drawRow(106, "TIME", "--:--:--", COL_NOFIX);
  }
}

static void drawDetailsPage() {
  TinyGPSPlus& g = gps.gps();
  canvas.setFont(&fonts::Font2);
  int y = 21;
  auto line = [&](uint16_t col, const char* fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    canvas.setTextColor(col);
    canvas.drawString(buf, 3, y);
    y += 16;
  };
  line(TFT_CYAN, "Speed %.1f km/h  Course %.0f", g.speed.kmph(), g.course.deg());
  line(TFT_GREEN, "HDOP %.2f  Sats %lu/%d  Age %lus", g.hdop.hdop(),
       (unsigned long)g.satellites.value(), gps.satsInView(),
       g.location.isValid() ? (unsigned long)(g.location.age() / 1000) : 0UL);
  line(TFT_YELLOW, "%s RX%d TX%d %lu%s", GPS_PRESETS[cfg.gpsModule].name, cfg.gpsRx, cfg.gpsTx,
       (unsigned long)gps.baud(), cfg.gpsBaud == 0 ? "(A)" : "");
  line(TFT_WHITE, "NMEA ok %lu  err %lu", (unsigned long)g.passedChecksum(),
       (unsigned long)g.failedChecksum());
  line(TFT_ORANGE, "Log %lus %s sleep:%s", (unsigned long)cfg.logIntervalSec,
       LOG_FORMAT_NAMES[cfg.logFormat], canDeepSleep() ? "on" : cfg.deepSleep ? "n/a" : "off");
  const char* name = strrchr(rtc.baseName, '/');
  line(TFT_MAGENTA, "File %s (%lu)", name ? name + 1 : "-", (unsigned long)rtc.points);
  int mv = batteryMvUi();
  line(TFT_GREEN, "Battery %.2fV %d%%  %s", mv / 1000.0f, batteryPctFromMv(mv),
       hasKeyboard ? "KB" : "no KB");
}

// ---------------------------------------------------------------- settings

enum SettingId {
  S_INTERVAL, S_FORMAT, S_SLEEP, S_SCREEN, S_FIXTO, S_MODULE, S_RX, S_TX, S_BAUD,
  S_TZ, S_AUTOSTART, S_BRIGHT, S_COUNT
};
static const char* const SETTING_NAMES[S_COUNT] = {
  "Log interval", "Log format", "Deep sleep", "Screen timeout", "Fix timeout", "GPS module",
  "GPS RX pin", "GPS TX pin", "GPS baud", "Time zone", "Auto start", "Brightness",
};
static const uint32_t INTERVALS[] = {1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600};
static const uint32_t SCREEN_TOS[] = {0, 15, 30, 60, 120, 300};
static const uint32_t FIX_TOS[] = {15, 30, 60, 90, 120, 180, 300};
static const uint32_t BAUDS[] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
static const uint32_t BRIGHTS[] = {16, 48, 96, 128, 192, 255};

template <size_t N>
static uint32_t cycle(const uint32_t (&arr)[N], uint32_t cur, int dir) {
  size_t i = 0;
  while (i < N && arr[i] < cur) i++;  // nearest entry >= cur
  if (i == N) i = N - 1;
  else if (arr[i] != cur && dir > 0) i--;  // so that +1 lands on the next larger value
  return arr[(i + N + dir) % N];
}

static String secondsText(uint32_t s) {
  if (s == 0) return "never";
  if (s < 60) return String(s) + " s";
  if (s < 3600) return String(s / 60) + " min";
  return String(s / 3600) + " h";
}

static String settingValue(int id) {
  switch (id) {
    case S_INTERVAL: return secondsText(cfg.logIntervalSec);
    case S_FORMAT: return LOG_FORMAT_NAMES[cfg.logFormat];
    case S_SLEEP:
      if (!cfg.deepSleep) return "off";
      return cfg.logIntervalSec >= MIN_SLEEP_INTERVAL_SEC ? "on" : "on (int<15s)";
    case S_SCREEN: return secondsText(cfg.screenTimeoutSec);
    case S_FIXTO: return secondsText(cfg.fixTimeoutSec);
    case S_MODULE: {
      String s = GPS_PRESETS[cfg.gpsModule].name;
      if (cfg.gpsModule == GPS_CAP_LORA1262 && !rtc.isAdv) s += " (ADV!)";
      return s;
    }
    case S_RX: return String(cfg.gpsRx);
    case S_TX: return String(cfg.gpsTx);
    case S_BAUD: return cfg.gpsBaud ? String(cfg.gpsBaud) : String("auto");
    case S_TZ: return "UTC" + String(cfg.tzOffsetHours >= 0 ? "+" : "") + String(cfg.tzOffsetHours);
    case S_AUTOSTART: return cfg.autoStart ? "on" : "off";
    case S_BRIGHT: return String(cfg.brightness * 100 / 255) + " %";
  }
  return "";
}

static void changeSetting(int id, int dir) {
  switch (id) {
    case S_INTERVAL: cfg.logIntervalSec = cycle(INTERVALS, cfg.logIntervalSec, dir); break;
    case S_FORMAT: cfg.logFormat = (cfg.logFormat + LOG_FORMAT_COUNT + dir) % LOG_FORMAT_COUNT; break;
    case S_SLEEP: cfg.deepSleep = !cfg.deepSleep; break;
    case S_SCREEN: cfg.screenTimeoutSec = cycle(SCREEN_TOS, cfg.screenTimeoutSec, dir); break;
    case S_FIXTO: cfg.fixTimeoutSec = cycle(FIX_TOS, cfg.fixTimeoutSec, dir); break;
    case S_MODULE: cfg.applyPreset((cfg.gpsModule + GPS_MODULE_COUNT + dir) % GPS_MODULE_COUNT); break;
    case S_RX: cfg.gpsRx = constrain(cfg.gpsRx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_TX: cfg.gpsTx = constrain(cfg.gpsTx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_BAUD: cfg.gpsBaud = cycle(BAUDS, cfg.gpsBaud, dir); break;
    case S_TZ: cfg.tzOffsetHours = constrain(cfg.tzOffsetHours + dir, -12, 14); break;
    case S_AUTOSTART: cfg.autoStart = !cfg.autoStart; break;
    case S_BRIGHT:
      cfg.brightness = cycle(BRIGHTS, cfg.brightness, dir);
      M5.Display.setBrightness(cfg.brightness);
      break;
  }
}

static void openSettings() {
  cfgBeforeEdit = cfg;
  settingsSel = 0;
  returnPage = page;
  page = PAGE_SETTINGS;
}

static void closeSettings() {
  bool gpsChanged = cfg.gpsRx != cfgBeforeEdit.gpsRx || cfg.gpsTx != cfgBeforeEdit.gpsTx ||
                    cfg.gpsBaud != cfgBeforeEdit.gpsBaud;
  if (gpsChanged) {
    rtc.detectedBaud = 0;
    startGps();
  }
  if (sdOk && configSave(cfg)) showStatus("Settings saved to SD");
  else showStatus("No SD: settings not saved");
  page = returnPage;
}

static void drawSettingsPage() {
  canvas.fillRect(0, 0, 240, 18, COL_SEL);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("SETTINGS", 3, 1);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextDatum(top_right);
  canvas.drawString(";. select ,/ change `save", 237, 1);
  canvas.setTextDatum(top_left);

  const int rows = 7, rowH = 16;
  int first = constrain(settingsSel - rows / 2, 0, S_COUNT - rows);
  for (int i = 0; i < rows; i++) {
    int id = first + i;
    int y = 20 + i * rowH;
    bool sel = id == settingsSel;
    if (sel) canvas.fillRoundRect(0, y, 240, rowH, 3, COL_SEL);
    canvas.setTextColor(sel ? TFT_WHITE : COL_LABEL);
    canvas.drawString(SETTING_NAMES[id], 6, y);
    canvas.setTextColor(sel ? TFT_YELLOW : TFT_CYAN);
    canvas.setTextDatum(top_right);
    canvas.drawString((sel ? "< " : "") + settingValue(id) + (sel ? " >" : ""), 236, y);
    canvas.setTextDatum(top_left);
  }
}

static void drawHelpPage() {
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_YELLOW);
  canvas.drawString("KEYS", 3, 20);
  const char* lines[] = {
    "Enter/R  start / stop logging",
    "Tab / ,/  page (pos/info/menu)",
    "S  settings   H  this help",
    "Z  screen off / sleep now",
    "G0 click: page  hold: rec",
    "G0 wakes from deep sleep",
  };
  canvas.setTextColor(TFT_WHITE);
  for (int i = 0; i < 6; i++) canvas.drawString(lines[i], 3, 38 + i * 16);
}

// ---------------------------------------------------------------- menu

enum MenuId { M_LOGGING, M_SETTINGS, M_HELP, M_SLEEP, M_COUNT };

static void drawMenuPage() {
  const int rowH = 26;
  for (int id = 0; id < M_COUNT; id++) {
    int y = 20 + id * rowH;
    bool sel = id == menuSel;
    String label;
    uint16_t col = TFT_WHITE;
    switch (id) {
      case M_LOGGING:
        label = rtc.logging ? "Stop logging" : "Start logging";
        col = rtc.logging ? TFT_RED : TFT_GREEN;
        break;
      case M_SETTINGS: label = "Settings"; col = TFT_CYAN; break;
      case M_HELP: label = "Help / keys"; col = TFT_YELLOW; break;
      case M_SLEEP:
        label = canDeepSleep() && lastLogMs ? "Deep sleep now" : "Screen off";
        col = TFT_MAGENTA;
        break;
    }
    if (sel) canvas.fillRoundRect(2, y, 236, rowH - 2, 5, COL_SEL);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(sel ? col : COL_LABEL);
    canvas.drawString(label, 14, y);
  }
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("Up/Down: select   Enter: OK   Tab: page", 3, 126);
}

static void draw() {
  canvas.fillScreen(COL_BG);
  canvas.setTextDatum(top_left);
  if (page == PAGE_SETTINGS) {
    drawSettingsPage();
  } else {
    drawStatusBar();
    if (page == PAGE_MAIN) drawMainPage();
    else if (page == PAGE_DETAILS) drawDetailsPage();
    else if (page == PAGE_MENU) drawMenuPage();
    else drawHelpPage();
  }
  if (millis() < statusMsgUntil) {
    canvas.setFont(&fonts::Font2);
    int w = canvas.textWidth(statusMsg) + 16;
    int x = (240 - w) / 2;
    canvas.fillRoundRect(x, 56, w, 24, 5, TFT_NAVY);
    canvas.drawRoundRect(x, 56, w, 24, 5, TFT_WHITE);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.drawString(statusMsg, 120, 68);
    canvas.setTextDatum(top_left);
  }
  canvas.pushSprite(0, 0);
}

// ---------------------------------------------------------------- screen / input

static void setScreen(bool on) {
  if (on == screenOn) return;
  screenOn = on;
  if (on) {
    M5.Display.wakeup();
    M5.Display.setBrightness(cfg.brightness);
  } else {
    M5.Display.setBrightness(0);
    M5.Display.sleep();
  }
}

static void sleepNowIfPossible() {
  if (canDeepSleep() && lastLogMs) {
    uint32_t ms = remainingIntervalMs(millis() - lastLogMs);
    draw();
    enterDeepSleep(max(ms, MIN_SLEEP_MS), true);
  }
  setScreen(false);
}

static void nextPage(int dir) {
  page = (Page)((page + PAGE_CYCLE_COUNT + dir) % PAGE_CYCLE_COUNT);
}

static void openHelp() {
  returnPage = page;
  page = PAGE_HELP;
}

static void menuSelect(int id) {
  switch (id) {
    case M_LOGGING: rtc.logging ? stopLogging() : startLogging(); break;
    case M_SETTINGS: openSettings(); break;
    case M_HELP: openHelp(); break;
    case M_SLEEP: sleepNowIfPossible(); break;
  }
}

static void handleKey(char c, bool enter, bool del, bool tab) {
  if (page == PAGE_SETTINGS) {
    if (c == ';') settingsSel = (settingsSel + S_COUNT - 1) % S_COUNT;
    else if (c == '.') settingsSel = (settingsSel + 1) % S_COUNT;
    else if (c == ',') changeSetting(settingsSel, -1);
    else if (c == '/' || enter) changeSetting(settingsSel, +1);
    else if (c == '`' || del || c == 's') closeSettings();
    return;
  }
  if (page == PAGE_HELP) {
    page = returnPage;
    return;
  }
  if (page == PAGE_MENU) {
    if (c == ';') { menuSel = (menuSel + M_COUNT - 1) % M_COUNT; return; }
    if (c == '.') { menuSel = (menuSel + 1) % M_COUNT; return; }
    if (enter) { menuSelect(menuSel); return; }
  }
  if (enter || c == 'r') rtc.logging ? stopLogging() : startLogging();
  else if (tab || c == '/' || c == ' ') nextPage(+1);
  else if (c == ',') nextPage(-1);
  else if (c == 's') openSettings();
  else if (c == 'h') openHelp();
  else if (c == 'z') sleepNowIfPossible();
}

static void handleInput() {
  bool activity = false;

  if (hasKeyboard) {
    M5Cardputer.Keyboard.updateKeyList();
    M5Cardputer.Keyboard.updateKeysState();
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
      activity = true;
      if (screenOn) {
        auto& ks = M5Cardputer.Keyboard.keysState();
        char c = ks.word.empty() ? 0 : tolower(ks.word[0]);
        handleKey(c, ks.enter, ks.del, ks.tab);
      }
    }
  }

  if (M5.BtnA.wasHold()) {
    activity = true;
    if (screenOn) rtc.logging ? stopLogging() : startLogging();
  } else if (M5.BtnA.wasClicked()) {
    activity = true;
    if (screenOn) {
      if (page == PAGE_SETTINGS) closeSettings();
      else if (page == PAGE_HELP) page = returnPage;
      else nextPage(+1);
    }
  }

  if (activity) {
    lastInputMs = millis();
    setScreen(true);
  }
}

static bool detectKeyboard() {
  auto board = M5.getBoard();
  if (board == m5::board_t::board_M5Cardputer) return true;  // GPIO matrix keyboard
  if (board == m5::board_t::board_M5CardputerADV) return M5.In_I2C.scanID(TCA8418_ADDR);
  return false;
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool fromSleep = cause == ESP_SLEEP_WAKEUP_TIMER || cause == ESP_SLEEP_WAKEUP_EXT0;
  if (!fromSleep || rtc.magic != RTC_MAGIC) {
    memset(&rtc, 0, sizeof(rtc));
    rtc.magic = RTC_MAGIC;
  }

  if (cause == ESP_SLEEP_WAKEUP_TIMER && rtc.logging) quietWakeCycle();  // returns only for UI

  gpio_hold_dis(PIN_BACKLIGHT);
  gpio_deep_sleep_hold_dis();

  auto m5cfg = M5.config();
  M5Cardputer.begin(m5cfg, false);
  rtc.isAdv = M5.getBoard() == m5::board_t::board_M5CardputerADV;
  hasKeyboard = detectKeyboard();
  if (hasKeyboard) M5Cardputer.Keyboard.begin();
  if (rtc.isAdv) {
    pinMode(PIN_ADV_LORA_CS, OUTPUT);
    digitalWrite(PIN_ADV_LORA_CS, HIGH);
  }

  M5.Display.setRotation(1);
  M5.Display.wakeup();
  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  configDefaults(cfg, rtc.isAdv);
  sdOk = logger::sdBegin();
  if (sdOk && !configLoad(cfg)) configSave(cfg);  // first start: write defaults to SD
  M5.Display.setBrightness(cfg.brightness);
  startGps();

  if (!sdOk) showStatus("No SD card!", 4000);
  else if (!fromSleep && cfg.autoStart) startLogging();
  else if (fromSleep && rtc.logging) showStatus("Woke up - still logging");

  lastInputMs = millis();
}

void loop() {
  M5.update();
  gps.update();
  if (gps.baudLocked() && cfg.gpsBaud == 0) rtc.detectedBaud = gps.baud();
  handleInput();

  uint32_t now = millis();
  if (rtc.logging && sdOk && (lastLogMs == 0 || now - lastLogMs >= cfg.logIntervalSec * 1000UL)) {
    if (logPoint(batteryPctFromMv(batteryMvUi()))) lastLogMs = now;
  }

  // Screen timeout: deep sleep between points if possible, otherwise screen off
  if (cfg.screenTimeoutSec && page != PAGE_SETTINGS &&
      now - lastInputMs > cfg.screenTimeoutSec * 1000UL) {
    if (canDeepSleep() && lastLogMs) {
      uint32_t ms = remainingIntervalMs(now - lastLogMs);
      // Point overdue without fix: sleep and let the timer wake retry with fix_timeout
      if (ms == 0 && !gps.hasFix()) ms = MIN_SLEEP_MS;
      if (ms >= MIN_SLEEP_MS) {
        showStatus("Deep sleep - G0 wakes", 10000);
        draw();
        delay(800);
        enterDeepSleep(ms, true);
      }
    } else {
      setScreen(false);
    }
  }

  if (screenOn && now - lastDrawMs > 200) {
    lastDrawMs = now;
    draw();
  }
  delay(5);
}
