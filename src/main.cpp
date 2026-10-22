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
#include "nmea_server.h"
#include "version.h"

static const gpio_num_t PIN_BACKLIGHT = GPIO_NUM_38;
static const gpio_num_t PIN_WAKE_BTN = GPIO_NUM_0;  // G0 / BtnA on Cardputer and ADV
static const int PIN_BATTERY = 10;
static const int PIN_ADV_LORA_CS = 5;               // SX1262 on CAP LoRa-1262 shares the SD SPI bus
static const uint8_t TCA8418_ADDR = 0x34;           // ADV keyboard controller

static const uint32_t MIN_SLEEP_INTERVAL_SEC = 15;  // below this, deep sleep is not worth it
static const uint32_t MIN_SLEEP_MS = 3000;
static const int MAX_SD_ERRORS = 3;                 // failed writes in a row until logging stops
static const float HDOP_UERE_M = 5.0f;              // for the accuracy estimate
static const uint32_t RTC_MAGIC = 0x47504C31;

// Survives deep sleep.
struct RtcState {
  uint32_t magic;
  bool logging;
  bool isAdv;
  char baseName[48];
  uint32_t points;
  uint32_t detectedBaud;
  uint64_t wakeAtMs;  // planned timer wake-up, on the RTC clock (rtcNowMs)
};
RTC_DATA_ATTR RtcState rtc;

enum Page { PAGE_MAIN, PAGE_BIG, PAGE_DETAILS, PAGE_SATS, PAGE_SIGNAL, PAGE_WIFI, PAGE_MENU, PAGE_SETTINGS, PAGE_HELP };
// MAIN .. MENU are switched with Tab / G0; WIFI only with the NMEA server on
static const int PAGE_CYCLE_COUNT = PAGE_MENU + 1;

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
static bool editing = false;   // text input for the selected setting
static String editBuf;
static int lastDeviceCount = 0;
static Config cfgBeforeEdit;
static int sdErrors = 0;       // failed point writes in a row

// ---------------------------------------------------------------- colors

static const uint16_t COL_BG = TFT_BLACK;
static const uint16_t COL_BAR = 0x18E3;     // dark grey
static const uint16_t COL_LABEL = 0x8410;   // grey
static const uint16_t COL_POS = TFT_CYAN;
static const uint16_t COL_ALT = TFT_YELLOW;
static const uint16_t COL_TIME = TFT_WHITE;
static const uint16_t COL_NOFIX = 0xFB2C;   // soft red
static const uint16_t COL_SEL = 0x2A69;     // blue-ish selection
static const uint16_t COL_SAT_USED = 0x2BC9;  // green: satellite used in fix
static const uint16_t COL_SAT_SEEN = 0x4A8B;  // grey: visible, not used

// ---------------------------------------------------------------- helpers

// Same reading as M5.Power.getBatteryVoltage(), but also usable in quiet
// wake-ups before M5 is initialized, so all logged points are comparable.
static int batteryMv() {
  return analogReadMilliVolts(PIN_BATTERY) * 2;  // 1:2 voltage divider
}

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
    int mv = batteryMv();
    avgMv = avgMv == 0 ? mv : avgMv + (mv - avgMv) * min(1.0f, dt / 30000.0f);
  }
  return (int)(avgMv + 0.5f);
}

static void showStatus(const char* msg, uint32_t ms = 2500) {
  strlcpy(statusMsg, msg, sizeof(statusMsg));
  statusMsgUntil = millis() + ms;
}

// Milliseconds on the RTC clock, which keeps running in deep sleep (millis() restarts at boot).
static uint64_t rtcNowMs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static bool canDeepSleep() {
  return rtc.logging && cfg.deepSleep && cfg.logIntervalSec >= MIN_SLEEP_INTERVAL_SEC && sdOk &&
         !cfg.nmeaServer;  // WiFi would be off while sleeping
}

static uint32_t effectiveBaud() {
  return cfg.gpsBaud ? cfg.gpsBaud : rtc.detectedBaud;  // reuse auto-detected baud after wake
}

// Raw NMEA goes to the track's .nmea file; sentences before the first point
// (no file name yet) are dropped.
static void nmeaSink(char c) {
  if (rtc.logging && cfg.logNmea && sdOk && rtc.baseName[0]) logger::nmeaPut(c, rtc.baseName);
  nmeaserver::put(c);
}

static void startGps() {
  gps.begin(cfg.gpsRx, cfg.gpsTx, effectiveBaud());
  gps.setRawSink(nmeaSink);
}

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
  epoch += (time_t)cfg.tzOffsetMin * 60;
  gmtime_r(&epoch, &out);
  return true;
}

// False without a fix, and for a fix that was already used: the loop may come
// back before the GPS has sent the next one.
static bool buildPoint(LogPoint& p, int batteryPct) {
  static uint32_t lastDate = 0, lastTime = 0;
  TinyGPSPlus& g = gps.gps();
  if (!gps.hasFix() || !g.date.isValid() || !g.time.isValid() || g.date.year() < 2020) return false;
  if (g.date.value() == lastDate && g.time.value() == lastTime) return false;
  lastDate = g.date.value();
  lastTime = g.time.value();
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

enum LogResult { LOG_OK, LOG_NO_FIX, LOG_SD_ERROR };

static LogResult logPoint(int batteryPct) {
  LogPoint p;
  if (!buildPoint(p, batteryPct)) return LOG_NO_FIX;
  if (!logger::write(p, cfg.logFormat, rtc.baseName, sizeof(rtc.baseName))) return LOG_SD_ERROR;
  rtc.points++;
  return LOG_OK;
}

static void startLogging() {
  if (!sdOk) {  // card inserted later, or replaced after write errors
    SD.end();
    sdOk = logger::sdBegin();
    if (!sdOk) {
      showStatus("No SD card!");
      return;
    }
  }
  sdErrors = 0;
  rtc.logging = true;
  rtc.baseName[0] = '\0';
  rtc.points = 0;
  lastLogMs = 0;
  showStatus("Logging started");
}

static void stopLogging() {
  if (rtc.baseName[0]) logger::nmeaFlush(rtc.baseName);  // also empties the buffer on SD errors
  rtc.logging = false;
  showStatus("Logging stopped");
}

// ---------------------------------------------------------------- deep sleep

[[noreturn]] static void enterDeepSleep(uint32_t sleepMs, bool displayActive) {
  gps.end();
  if (sdOk && rtc.baseName[0]) logger::nmeaFlush(rtc.baseName);
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
  rtc.wakeAtMs = rtcNowMs() + sleepMs;
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
  // Measured from the planned wake-up, so boot time doesn't add up to a drift.
  auto awakeMs = [] {
    uint64_t now = rtcNowMs();
    return now > rtc.wakeAtMs ? (uint32_t)(now - rtc.wakeAtMs) : 0;
  };
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

  LogResult result = LOG_NO_FIX;
  while (awakeMs() < cfg.fixTimeoutSec * 1000UL) {
    gps.update();
    if (gps.baudLocked() && cfg.gpsBaud == 0) rtc.detectedBaud = gps.baud();
    if (digitalRead(PIN_WAKE_BTN) == LOW) return;
    if (gps.gps().location.isUpdated()) {
      result = logPoint(batteryPctFromMv(batteryMv()));
      if (result != LOG_NO_FIX) break;  // SD error: try again at the next wake-up
    }
    delay(5);
  }
  log_i("Quiet wake: %s after %lu ms",
        result == LOG_OK ? "logged" : result == LOG_SD_ERROR ? "SD error" : "no fix",
        (unsigned long)awakeMs());
  uint32_t sleepMs = remainingIntervalMs(awakeMs());
  enterDeepSleep(max(sleepMs, MIN_SLEEP_MS), false);
}

// ---------------------------------------------------------------- drawing

static uint16_t batteryColor(int pct) {
  return pct > 50 ? TFT_GREEN : pct > 20 ? TFT_YELLOW : TFT_RED;
}

// Battery symbol ending at x (right edge), with the percentage left of it if
// withPct. Returns the left edge.
static int drawBattery(int x, int pct, bool withPct) {
  uint16_t col = batteryColor(pct);
  x -= 24;
  canvas.drawRect(x, 3, 22, 11, TFT_WHITE);
  canvas.fillRect(x + 22, 6, 2, 5, TFT_WHITE);
  canvas.fillRect(x + 2, 5, max(18 * pct / 100, 1), 7, col);
  if (!withPct) return x;
  String s = String(pct) + "%";
  canvas.setTextColor(col);
  canvas.drawString(s, x - 3 - canvas.textWidth(s), 1);
  return x - 3 - canvas.textWidth(s);
}

// Up arrow (U+2191 is not in the built-in fonts) followed by "NMEA".
static const int NMEA_ARROW_W = 9;
static void drawNmeaUpload(int x, uint16_t col) {
  canvas.fillTriangle(x + 3, 3, x, 8, x + 6, 8, col);
  canvas.fillRect(x + 2, 8, 3, 7, col);
  canvas.setTextColor(col);
  canvas.drawString("NMEA", x + NMEA_ARROW_W, 1);
}

static uint16_t satColor(int sats) {
  return sats >= 7 ? TFT_GREEN : sats >= 4 ? TFT_YELLOW : TFT_RED;
}

// Left: recording state. Right to left: battery, NMEA server, SD (only if
// missing), satellite reception. Without room the satellite count (or the
// baud rate being tried) and then the battery percentage are left out.
static void drawStatusBar() {
  canvas.fillRect(0, 0, 240, 18, COL_BAR);
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(top_left);
  int left;
  if (rtc.logging) {
    bool blink = (millis() / 500) % 2;
    canvas.fillCircle(7, 9, 5, blink ? TFT_RED : 0x7800);
    canvas.setTextColor(TFT_RED);
    String rec = "REC " + String(rtc.points);
    canvas.drawString(rec, 15, 1);
    left = 15 + canvas.textWidth(rec);
  } else {
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("IDLE", 3, 1);
    left = 3 + canvas.textWidth("IDLE");
  }

  // Satellite reception
  bool gpsAlive = gps.lastDataMs() && millis() - gps.lastDataMs() < 5000;
  int used = gps.gps().satellites.value();
  String sat, satCount;
  uint16_t satCol;
  if (!gps.baudLocked()) {
    sat = "BAUD";
    satCount = " " + String(gps.baud());
    satCol = TFT_ORANGE;
  } else if (!gpsAlive) {
    sat = "NO GPS";
    satCol = TFT_RED;
  } else {
    bool fix = gps.hasFix();
    sat = fix ? "FIX" : "NO FIX";
    satCol = fix ? satColor(used) : TFT_RED;
    satCount = " " + String(used) + "/" + String(gps.satsInView());
  }

  // NMEA server: orange = no network, grey = no device, green = devices connected
  bool nmea = nmeaserver::running();
  int devices = nmeaserver::deviceCount();
  uint16_t nmeaCol = !nmeaserver::networkUp() ? TFT_ORANGE : devices ? TFT_GREEN : COL_LABEL;

  const int gap = 8, right = 238;
  int pct = batteryPctFromMv(batteryMvUi());
  String pctText = String(pct) + "%";
  int need = left + gap + canvas.textWidth(sat) + gap + 24;
  if (!sdOk) need += gap + canvas.textWidth("NOSD");
  if (nmea) need += gap + NMEA_ARROW_W + canvas.textWidth("NMEA");
  bool withCount = satCount.length() && need + canvas.textWidth(satCount) <= right;
  if (withCount) need += canvas.textWidth(satCount);
  bool withPct = need + 3 + canvas.textWidth(pctText) <= right;

  int x = drawBattery(right, pct, withPct) - gap;
  if (nmea) {
    x -= NMEA_ARROW_W + canvas.textWidth("NMEA");
    drawNmeaUpload(x, nmeaCol);
    x -= gap;
  }
  if (!sdOk) {
    x -= canvas.textWidth("NOSD");
    canvas.setTextColor(TFT_RED);
    canvas.drawString("NOSD", x, 1);
    x -= gap;
  }
  if (withCount) {
    x -= canvas.textWidth(satCount);
    canvas.setTextColor(satCol);
    canvas.drawString(satCount, x, 1);
  }
  x -= canvas.textWidth(sat);
  canvas.setTextColor(satCol);
  canvas.drawString(sat, x, 1);
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
    canvas.drawString("UTC" + tzText(cfg.tzOffsetMin), 160, 121);
  } else {
    drawRow(106, "TIME", "--:--:--", COL_NOFIX);
  }
}

// Speed, direction and altitude in large letters, readable at a glance.
static void drawBigPage() {
  static const char* const DIRS[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  TinyGPSPlus& g = gps.gps();
  bool fix = gps.hasFix();
  char buf[16];
  auto row = [&](int baseline, const char* label, const char* value, const char* unit, uint16_t col) {
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_LABEL);
    canvas.setTextDatum(baseline_left);
    canvas.drawString(label, 3, baseline);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextColor(fix ? col : COL_NOFIX);
    canvas.drawString(value, 32, baseline);
    int x = 32 + canvas.textWidth(value);
    if (fix && unit) {
      canvas.setFont(&fonts::Font2);
      canvas.drawString(unit, x + 4, baseline);
    }
    return x;
  };

  if (fix && g.speed.isValid()) {
    double kmh = g.speed.kmph();
    snprintf(buf, sizeof(buf), kmh < 100 ? "%.1f" : "%.0f", kmh);
  } else {
    strcpy(buf, "--");
  }
  row(55, "SPD", buf, "km/h", COL_POS);

  bool course = fix && g.course.isValid();
  if (course) {
    double deg = g.course.deg();
    snprintf(buf, sizeof(buf), "%s %.0f", DIRS[(int)((deg + 22.5) / 45) % 8], deg);
  } else {
    strcpy(buf, "--");
  }
  int x = row(94, "DIR", buf, nullptr, TFT_WHITE);
  if (course) {  // degree sign, not in the font
    canvas.drawCircle(x + 7, 66, 4, TFT_WHITE);
    canvas.drawCircle(x + 7, 66, 3, TFT_WHITE);
  }

  if (fix && g.altitude.isValid()) snprintf(buf, sizeof(buf), "%.0f", g.altitude.meters());
  else strcpy(buf, "--");
  row(133, "ALT", buf, "m", COL_ALT);
  canvas.setTextDatum(top_left);
}

static void drawDetailsPage() {
  TinyGPSPlus& g = gps.gps();
  canvas.setFont(&fonts::Font2);
  int y = 20;
  auto line = [&](uint16_t col, const char* fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    canvas.setTextColor(col);
    canvas.drawString(buf, 3, y);
    y += 14;
  };
  line(TFT_CYAN, "Speed %.1f km/h  Course %.0f", g.speed.kmph(), g.course.deg());
  // Rough horizontal accuracy: HDOP x user range error (~5 m for a consumer
  // module without corrections; real error can be larger with weak signals).
  char acc[12] = "";
  if (gps.hasFix() && g.hdop.isValid()) snprintf(acc, sizeof(acc), " ~%.0fm", g.hdop.hdop() * HDOP_UERE_M);
  line(TFT_GREEN, "HDOP %.1f%s  Sats %lu/%d  Age %lus", g.hdop.hdop(), acc,
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
  line(TFT_SKYBLUE, "WiFi %s %s", !nmeaserver::running() ? "" : cfg.wifiClient ? "client" : "AP",
       nmeaserver::statusText().c_str());
}

// Label in grey, value in color, one Font2 line.
static void drawLabeled(int y, const char* label, const String& value, uint16_t color) {
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString(label, 3, y);
  canvas.setTextColor(color);
  canvas.drawString(value, 72, y);
}

static void drawWifiPage() {
  bool up = nmeaserver::networkUp();
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_SKYBLUE);
  canvas.drawString(cfg.wifiClient ? "NMEA SERVER - joins WiFi" : "NMEA SERVER - own WiFi", 3, 20);

  if (cfg.wifiClient) {
    drawLabeled(37, "Network", cfg.wifiSsid.length() ? cfg.wifiSsid : String("(not set)"), TFT_WHITE);
    String state = nmeaserver::stateText();
    drawLabeled(53, "Status", state, up ? TFT_GREEN : state == "connecting..." ? TFT_ORANGE : TFT_RED);
  } else {
    drawLabeled(37, "Network", cfg.apSsid, TFT_WHITE);
    drawLabeled(53, "Password", cfg.apPass.length() ? cfg.apPass : String("(open)"), TFT_WHITE);
  }

  // Address in large digits: this is what goes into the app
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("HOST", 3, 79);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(up ? TFT_CYAN : COL_NOFIX);
  canvas.drawString(up ? nmeaserver::ipText() : String("--"), 30, 70);

  drawLabeled(97, "Port", String(cfg.nmeaPort) + "  (TCP + UDP)", TFT_YELLOW);

  int tcp = nmeaserver::tcpClients();
  String devices = "TCP " + String(tcp);
  if (!cfg.wifiClient) devices += ", in WiFi " + String(nmeaserver::apStations());
  drawLabeled(113, "Devices", devices, nmeaserver::deviceCount() ? TFT_GREEN : COL_LABEL);
}

// Satellites per constellation: used (green) and in view (grey) as bars,
// scaled to the constellation with the most satellites in view.
static void drawSatsPage() {
  const int rowH = 19, barX = 66, barW = 128;
  int scale = 1;
  for (int sys = 0; sys < SYS_COUNT; sys++) scale = max(scale, gps.satsInView(sys));
  canvas.setFont(&fonts::Font2);
  for (int sys = 0; sys < SYS_COUNT; sys++) {
    int y = 20 + sys * rowH;
    int seen = gps.satsInView(sys), used = gps.satsUsed(sys);
    canvas.setTextColor(seen ? TFT_WHITE : COL_LABEL);
    canvas.drawString(GNSS_NAMES[sys], 3, y + 1);
    canvas.fillRoundRect(barX, y + 5, barW, 8, 3, COL_BAR);
    if (seen) canvas.fillRoundRect(barX, y + 5, barW * seen / scale, 8, 3, COL_SAT_SEEN);
    if (used) canvas.fillRoundRect(barX, y + 5, barW * min(used, scale) / scale, 8, 3, COL_SAT_USED);
    canvas.setTextColor(used ? TFT_GREEN : seen ? TFT_WHITE : COL_LABEL);
    canvas.setTextDatum(top_right);
    canvas.drawString(String(used) + "/" + String(seen), 237, y + 1);
    canvas.setTextDatum(top_left);
  }
}

// Signal strength (SNR, dB-Hz) of each satellite in view as vertical bars,
// green = used in fix. Below each bar the constellation letter, above it the
// SNR value if there is room.
static void drawSignalPage() {
  static const char SYS_LETTERS[SYS_COUNT] = {'G', 'R', 'E', 'C', 'J', 'S'};
  static SatInfo sats[64];
  int n = gps.satellites(sats, 64);
  const int top = 42, bottom = 124, maxSnr = 50;  // grid line every 10 dB-Hz
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  if (n == 0) {
    canvas.setFont(&fonts::Font2);
    canvas.drawString("No satellites in view", 3, 22);
    return;
  }
  canvas.drawString("G GPS R GLO E GAL C BDS J QZS S SBAS", 3, 21);
  for (int db = 10; db < maxSnr; db += 10) {
    int y = bottom - (bottom - top) * db / maxSnr;
    canvas.drawFastHLine(0, y, 240, COL_BAR);
  }
  int slot = min(16, 240 / n);
  int barW = max(slot - 2, 1);
  int x0 = (240 - slot * n) / 2;
  for (int i = 0; i < n; i++) {
    int x = x0 + i * slot;
    int h = (bottom - top) * min<int>(sats[i].snr, maxSnr) / maxSnr;
    uint16_t col = sats[i].used ? COL_SAT_USED : COL_SAT_SEEN;
    if (h) canvas.fillRect(x, bottom - h, barW, h, col);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(sats[i].used ? TFT_GREEN : COL_LABEL);
    if (slot >= 6) canvas.drawString(String(SYS_LETTERS[sats[i].sys]), x + barW / 2, bottom + 2);
    if (slot >= 13 && sats[i].snr) {
      canvas.setTextDatum(bottom_center);
      canvas.drawString(String(sats[i].snr), x + barW / 2, bottom - h - 1);
    }
  }
  canvas.setTextDatum(top_left);
}

// ---------------------------------------------------------------- settings

enum SettingId {
  S_INTERVAL, S_FORMAT, S_NMEA, S_SLEEP, S_SCREEN, S_FIXTO, S_MODULE, S_RX, S_TX, S_BAUD,
  S_SERVER, S_WIFIMODE, S_PORT, S_SSID, S_PASS, S_APSSID, S_APPASS,
  S_TZ, S_AUTOSTART, S_BRIGHT, S_COUNT
};
static const char* const SETTING_NAMES[S_COUNT] = {
  "Log interval", "Log format", "NMEA log", "Deep sleep", "Screen timeout", "Fix timeout", "GPS module",
  "GPS RX pin", "GPS TX pin", "GPS baud", "NMEA server", "WiFi mode", "NMEA port",
  "WiFi SSID", "WiFi password", "AP name", "AP password", "Time zone", "Auto start", "Brightness",
};
static const uint32_t INTERVALS[] = {1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600};
static const uint32_t SCREEN_TOS[] = {0, 15, 30, 60, 120, 300};
static const uint32_t FIX_TOS[] = {15, 30, 60, 90, 120, 180, 300};
static const uint32_t BAUDS[] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
static const uint32_t BRIGHTS[] = {16, 48, 96, 128, 192, 255};
// UTC offsets in use, in minutes
static const int16_t TZ_OFFSETS[] = {
  -720, -660, -600, -570, -540, -480, -420, -360, -300, -240, -210, -180, -120, -60, 0,
  60, 120, 180, 210, 240, 270, 300, 330, 345, 360, 390, 420, 480, 525, 540, 570, 600,
  630, 660, 720, 765, 780, 840,
};

// Next larger (dir > 0) or smaller entry of an ascending array, wrapping
// around; cur doesn't need to be one of the entries.
template <typename T, size_t N, typename V>
static T cycle(const T (&arr)[N], V cur, int dir) {
  if (dir > 0) {
    for (size_t i = 0; i < N; i++)
      if (arr[i] > cur) return arr[i];
    return arr[0];
  }
  for (size_t i = N; i-- > 0;)
    if (arr[i] < cur) return arr[i];
  return arr[N - 1];
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
    case S_NMEA: return cfg.logNmea ? "on" : "off";
    case S_SLEEP:
      if (!cfg.deepSleep) return "off";
      if (cfg.nmeaServer) return "on (WiFi on)";
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
    case S_TZ: return "UTC" + tzText(cfg.tzOffsetMin);
    case S_AUTOSTART: return cfg.autoStart ? "on" : "off";
    case S_BRIGHT: return String(cfg.brightness * 100 / 255) + " %";
    case S_SERVER: return cfg.nmeaServer ? "on" : "off";
    case S_WIFIMODE: return cfg.wifiClient ? "client" : "own AP";
    case S_PORT: return String(cfg.nmeaPort);
    case S_SSID: return cfg.wifiSsid.length() ? cfg.wifiSsid : String("-");
    case S_PASS: return cfg.wifiPass.length() ? String("****") : String("-");
    case S_APSSID: return cfg.apSsid;
    case S_APPASS: return cfg.apPass.length() ? String("****") : String("(open)");
  }
  return "";
}

// WiFi settings complete enough to start the NMEA server.
static bool wifiConfigured() {
  if (cfg.wifiClient) return cfg.wifiSsid.length() > 0;
  return cfg.apSsid.length() > 0 && (cfg.apPass.length() == 0 || cfg.apPass.length() >= 8);
}

// ---- text input for SSIDs, passwords and the port

static String* textSetting(int id) {
  switch (id) {
    case S_SSID: return &cfg.wifiSsid;
    case S_PASS: return &cfg.wifiPass;
    case S_APSSID: return &cfg.apSsid;
    case S_APPASS: return &cfg.apPass;
  }
  return nullptr;
}

static bool isTextSetting(int id) { return id == S_PORT || textSetting(id); }

static void startEdit() {
  editing = true;
  editBuf = settingsSel == S_PORT ? String(cfg.nmeaPort) : *textSetting(settingsSel);
}

static void finishEdit() {
  if (settingsSel == S_PORT) {
    long p = editBuf.toInt();
    if (p < 1 || p > 65535) { showStatus("Port must be 1-65535"); return; }
    cfg.nmeaPort = p;
  } else if (settingsSel == S_APSSID && editBuf.length() == 0) {
    showStatus("AP name must not be empty");
    return;
  } else if (settingsSel == S_APPASS && editBuf.length() && editBuf.length() < 8) {
    showStatus("Min. 8 characters (or empty)");
    return;
  } else {
    *textSetting(settingsSel) = editBuf;
  }
  editing = false;
}

static void handleEditKey(const std::vector<char>& word, bool enter, bool del) {
  if (enter) {
    finishEdit();
  } else if (del) {
    if (editBuf.length()) editBuf.remove(editBuf.length() - 1);
  } else {
    size_t maxLen = settingsSel == S_PORT ? 5 : (settingsSel == S_SSID || settingsSel == S_APSSID) ? 32 : 63;
    for (char c : word) {
      if (editBuf.length() < maxLen && (settingsSel != S_PORT || isdigit(c))) editBuf += c;
    }
  }
}

static void changeSetting(int id, int dir) {
  switch (id) {
    case S_INTERVAL: cfg.logIntervalSec = cycle(INTERVALS, cfg.logIntervalSec, dir); break;
    case S_FORMAT: cfg.logFormat = (cfg.logFormat + LOG_FORMAT_COUNT + dir) % LOG_FORMAT_COUNT; break;
    case S_NMEA: cfg.logNmea = !cfg.logNmea; break;
    case S_SLEEP: cfg.deepSleep = !cfg.deepSleep; break;
    case S_SCREEN: cfg.screenTimeoutSec = cycle(SCREEN_TOS, cfg.screenTimeoutSec, dir); break;
    case S_FIXTO: cfg.fixTimeoutSec = cycle(FIX_TOS, cfg.fixTimeoutSec, dir); break;
    case S_MODULE: cfg.applyPreset((cfg.gpsModule + GPS_MODULE_COUNT + dir) % GPS_MODULE_COUNT); break;
    case S_RX: cfg.gpsRx = constrain(cfg.gpsRx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_TX: cfg.gpsTx = constrain(cfg.gpsTx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_BAUD: cfg.gpsBaud = cycle(BAUDS, cfg.gpsBaud, dir); break;
    case S_TZ: cfg.tzOffsetMin = cycle(TZ_OFFSETS, cfg.tzOffsetMin, dir); break;
    case S_AUTOSTART: cfg.autoStart = !cfg.autoStart; break;
    case S_SERVER: cfg.nmeaServer = !cfg.nmeaServer; break;
    case S_WIFIMODE: cfg.wifiClient = !cfg.wifiClient; break;
    case S_BRIGHT:
      cfg.brightness = cycle(BRIGHTS, cfg.brightness, dir);
      M5.Display.setBrightness(cfg.brightness);
      break;
  }
}

static void openSettings() {
  cfgBeforeEdit = cfg;
  editing = false;
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
  bool wifiChanged = cfg.nmeaServer != cfgBeforeEdit.nmeaServer ||
                     cfg.wifiClient != cfgBeforeEdit.wifiClient ||
                     cfg.nmeaPort != cfgBeforeEdit.nmeaPort || cfg.wifiSsid != cfgBeforeEdit.wifiSsid ||
                     cfg.wifiPass != cfgBeforeEdit.wifiPass || cfg.apSsid != cfgBeforeEdit.apSsid ||
                     cfg.apPass != cfgBeforeEdit.apPass;
  bool wifiIncomplete = cfg.nmeaServer && !wifiConfigured();
  if (wifiIncomplete) cfg.nmeaServer = false;
  if (wifiChanged || wifiIncomplete) {
    if (cfg.nmeaServer) nmeaserver::begin(cfg);
    else nmeaserver::end();
  }
  bool saved = sdOk && configSave(cfg);
  if (wifiIncomplete) showStatus("WiFi incomplete: server off");
  else showStatus(saved ? "Settings saved to SD" : "No SD: settings not saved");
  page = returnPage == PAGE_WIFI && !cfg.nmeaServer ? PAGE_MAIN : returnPage;
}

static void drawSettingsPage() {
  canvas.fillRect(0, 0, 240, 18, COL_SEL);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("SETTINGS", 3, 1);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextDatum(top_right);
  canvas.drawString(editing ? "type, Enter OK, G0 cancel" : ";. select ,/ change `save", 237, 1);
  canvas.setTextDatum(top_left);

  // Small list for the overview
  const int rows = 4, rowH = 16;
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
    canvas.drawString(settingValue(id), 236, y);
    canvas.setTextDatum(top_left);
  }

  // Selected setting in a box with its value in the large menu font
  const int boxY = 87, maxW = 226;
  canvas.fillRoundRect(0, boxY, 240, 135 - boxY, 5, COL_SEL);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString(SETTING_NAMES[settingsSel], 7, boxY + 2);
  String value;
  if (editing) {
    value = editBuf;  // long input: show the end, where the cursor is
    while (value.length() && canvas.textWidth(value + "_", &fonts::Font4) > maxW) value.remove(0, 1);
    value += (millis() / 400) % 2 ? "_" : " ";
  } else {
    value = isTextSetting(settingsSel) ? settingValue(settingsSel)
                                       : "< " + settingValue(settingsSel) + " >";
  }
  bool large = canvas.textWidth(value, &fonts::Font4) <= maxW;
  canvas.setFont(large ? &fonts::Font4 : &fonts::Font2);
  canvas.setTextColor(TFT_YELLOW);
  canvas.drawString(value, 7, large ? boxY + 20 : boxY + 25);
}

static void drawHelpPage() {
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_YELLOW);
  canvas.drawString("KEYS", 3, 20);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextDatum(top_right);
  canvas.drawString("GPS Logger v" FW_VERSION, 237, 20);
  canvas.setTextDatum(top_left);
  const char* lines[] = {
    "Enter/R  start / stop logging",
    "Tab ,/ 1-7  page (1 pos .. 7 menu)",
    "S  settings   H  this help",
    "Z  screen off/sleep   P  screenshot",
    "G0 click: page  hold: rec",
    "G0 wakes from deep sleep",
  };
  canvas.setTextColor(TFT_WHITE);
  for (int i = 0; i < 6; i++) canvas.drawString(lines[i], 3, 38 + i * 16);
}

// ---------------------------------------------------------------- menu

enum MenuId { M_LOGGING, M_SERVER, M_SETTINGS, M_HELP, M_SLEEP, M_COUNT };

static void toggleNmeaServer() {
  if (!cfg.nmeaServer && !wifiConfigured()) {
    showStatus("Set up WiFi in settings first");
    return;
  }
  cfg.nmeaServer = !cfg.nmeaServer;
  if (cfg.nmeaServer) nmeaserver::begin(cfg);
  else nmeaserver::end();
  if (sdOk) configSave(cfg);
  showStatus(cfg.nmeaServer ? "NMEA server started" : "NMEA server stopped");
}

// Font4 needs ~24 px per line with descenders: rows touch, so the selection
// is drawn before all labels and doesn't cover the row above.
static void drawMenuPage() {
  const int top = 19, rowH = 23;
  canvas.fillRoundRect(2, top + menuSel * rowH, 236, rowH - 2, 5, COL_SEL);
  for (int id = 0; id < M_COUNT; id++) {
    int y = top + id * rowH;
    bool sel = id == menuSel;
    String label;
    uint16_t col = TFT_WHITE;
    switch (id) {
      case M_LOGGING:
        label = rtc.logging ? "Stop logging" : "Start logging";
        col = rtc.logging ? TFT_RED : TFT_GREEN;
        break;
      case M_SERVER:
        label = cfg.nmeaServer ? "Stop NMEA server" : "Start NMEA server";
        col = cfg.nmeaServer ? TFT_ORANGE : TFT_SKYBLUE;
        break;
      case M_SETTINGS: label = "Settings"; col = TFT_CYAN; break;
      case M_HELP: label = "Help / keys"; col = TFT_YELLOW; break;
      case M_SLEEP:
        label = canDeepSleep() && lastLogMs ? "Deep sleep now" : "Screen off";
        col = TFT_MAGENTA;
        break;
    }
    bool disabled = id == M_SERVER && !cfg.nmeaServer && !wifiConfigured();
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(disabled ? 0x4208 : sel ? col : COL_LABEL);
    canvas.drawString(label, 14, y);
  }
}

static void draw() {
  canvas.fillScreen(COL_BG);
  canvas.setTextDatum(top_left);
  if (page == PAGE_SETTINGS) {
    drawSettingsPage();
  } else {
    drawStatusBar();
    if (page == PAGE_MAIN) drawMainPage();
    else if (page == PAGE_BIG) drawBigPage();
    else if (page == PAGE_DETAILS) drawDetailsPage();
    else if (page == PAGE_SATS) drawSatsPage();
    else if (page == PAGE_SIGNAL) drawSignalPage();
    else if (page == PAGE_WIFI) drawWifiPage();
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

// Saves the current page (without status message) as 24-bit BMP to the SD
// card, row by row: PNG compression needs more RAM than is free.
static void saveScreenshot() {
  if (!sdOk) { showStatus("No SD card!"); return; }
  statusMsgUntil = 0;
  draw();
  SD.mkdir("/gpslogger");
  SD.mkdir("/gpslogger/screenshots");
  static int nextNum = 1;  // searched from the last free number, not from 1 each time
  char path[48];
  for (; nextNum < 10000; nextNum++) {
    snprintf(path, sizeof(path), "/gpslogger/screenshots/%04d.bmp", nextNum);
    if (!SD.exists(path)) break;
  }
  if (nextNum >= 10000) { showStatus("Screenshot folder full"); return; }
  File f = SD.open(path, FILE_WRITE);
  if (!f) { showStatus("SD write error!"); return; }

  const int w = 240, h = 135;
  const uint32_t rowBytes = w * 3, imageBytes = rowBytes * h;  // 720: no row padding needed
  uint8_t hdr[54] = {'B', 'M'};
  auto put32 = [&](int at, uint32_t v) { for (int i = 0; i < 4; i++) hdr[at + i] = v >> (8 * i); };
  put32(2, sizeof(hdr) + imageBytes);  // file size
  put32(10, sizeof(hdr));              // pixel data offset
  put32(14, 40);                       // info header size
  put32(18, w);
  put32(22, h);                        // positive: rows bottom-up
  hdr[26] = 1;                         // planes
  hdr[28] = 24;                        // bits per pixel
  put32(34, imageBytes);
  bool ok = f.write(hdr, sizeof(hdr)) == sizeof(hdr);

  static uint8_t row[w * 3];
  for (int y = h - 1; ok && y >= 0; y--) {
    canvas.readRectRGB(0, y, w, 1, row);  // R, G, B
    for (int x = 0; x < w; x++) std::swap(row[x * 3], row[x * 3 + 2]);  // BMP wants B, G, R
    ok = f.write(row, rowBytes) == rowBytes;
  }
  f.close();
  if (ok) nextNum++;
  showStatus(ok ? (String("Saved ") + (strrchr(path, '/') + 1)).c_str() : "SD write error!");
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
  do {
    page = (Page)((page + PAGE_CYCLE_COUNT + dir) % PAGE_CYCLE_COUNT);
  } while (page == PAGE_WIFI && !cfg.nmeaServer);
}

static void openHelp() {
  returnPage = page;
  page = PAGE_HELP;
}

static void menuSelect(int id) {
  switch (id) {
    case M_LOGGING: rtc.logging ? stopLogging() : startLogging(); break;
    case M_SERVER: toggleNmeaServer(); break;
    case M_SETTINGS: openSettings(); break;
    case M_HELP: openHelp(); break;
    case M_SLEEP: sleepNowIfPossible(); break;
  }
}

static void handleKey(char c, bool enter, bool del, bool tab) {
  if (page == PAGE_SETTINGS) {
    if (isTextSetting(settingsSel) && (enter || c == '/')) {
      if (hasKeyboard) startEdit();
      return;
    }
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
  if (c >= '1' && c < '1' + PAGE_CYCLE_COUNT) {  // pages in cycle order
    Page p = (Page)(c - '1');
    if (p == PAGE_WIFI && !cfg.nmeaServer) showStatus("NMEA server is off");
    else page = p;
    return;
  }
  if (enter || c == 'r') rtc.logging ? stopLogging() : startLogging();
  else if (tab || c == '/' || c == ' ') nextPage(+1);
  else if (c == ',') nextPage(-1);
  else if (c == 's') openSettings();
  else if (c == 'h') openHelp();
  else if (c == 'z') sleepNowIfPossible();
  else if (c == 'p') saveScreenshot();
}

static void handleInput() {
  bool activity = false;
  bool wasOn = screenOn;  // a key that turns the screen off must not wake it again

  if (hasKeyboard) {
    M5Cardputer.Keyboard.updateKeyList();
    M5Cardputer.Keyboard.updateKeysState();
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
      activity = true;
      if (screenOn) {
        auto& ks = M5Cardputer.Keyboard.keysState();
        if (page == PAGE_SETTINGS && editing) {
          handleEditKey(ks.word, ks.enter, ks.del);
        } else {
          char c = ks.word.empty() ? 0 : tolower(ks.word[0]);
          handleKey(c, ks.enter, ks.del, ks.tab);
        }
      }
    }
  }

  if (M5.BtnA.wasHold()) {
    activity = true;
    if (screenOn && !(page == PAGE_SETTINGS && editing)) rtc.logging ? stopLogging() : startLogging();
  } else if (M5.BtnA.wasClicked()) {
    activity = true;
    if (screenOn) {
      if (page == PAGE_SETTINGS && editing) editing = false;
      else if (page == PAGE_SETTINGS) closeSettings();
      else if (page == PAGE_HELP) page = returnPage;
      else nextPage(+1);
    }
  }

  if (activity) {
    lastInputMs = millis();
    if (!wasOn) setScreen(true);
  }
}

// Default AP password, so devices don't share a known one. Without letters
// and digits that are easy to confuse (l/1, o/0).
static String randomPassword() {
  static const char CHARS[] = "abcdefghijkmnpqrstuvwxyz23456789";
  String s;
  for (int i = 0; i < 10; i++) s += CHARS[esp_random() % (sizeof(CHARS) - 1)];
  return s;
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
  if (!sdOk || !configLoad(cfg)) {  // first start: own AP password, defaults to SD
    cfg.apPass = randomPassword();
    if (sdOk) configSave(cfg);
  }
  M5.Display.setBrightness(cfg.brightness);
  startGps();
  if (cfg.nmeaServer && !wifiConfigured()) {
    cfg.nmeaServer = false;
    showStatus("WiFi incomplete: server off", 4000);
  }
  if (cfg.nmeaServer) nmeaserver::begin(cfg);

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
  nmeaserver::update();
  int devices = nmeaserver::deviceCount();
  if (devices != lastDeviceCount) {
    showStatus(devices > lastDeviceCount ? ("Device connected (" + String(devices) + ")").c_str()
                                         : ("Device disconnected (" + String(devices) + ")").c_str());
    lastDeviceCount = devices;
  }

  uint32_t now = millis();
  if (rtc.logging && sdOk && (lastLogMs == 0 || now - lastLogMs >= cfg.logIntervalSec * 1000UL)) {
    LogResult result = logPoint(batteryPctFromMv(batteryMvUi()));
    if (result == LOG_OK) {
      lastLogMs = now;
      sdErrors = 0;
    } else if (result == LOG_SD_ERROR && ++sdErrors >= MAX_SD_ERRORS) {
      // Card removed or broken: stop, so the screen timeout works again
      sdOk = false;
      stopLogging();
      showStatus("SD error: logging stopped", 5000);
    } else if (result == LOG_SD_ERROR) {
      showStatus("SD write error!");
    }
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
