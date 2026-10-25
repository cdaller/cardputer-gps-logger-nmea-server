// GPS Logger for M5Stack Cardputer / Cardputer ADV
//
// Logs GPS positions to CSV and/or GPX files on the SD card. While logging
// with deep sleep enabled, the ESP32 sleeps between points: on each timer
// wake-up it only reads the GPS, writes one point and goes back to sleep
// without touching the display. Pressing G0 (BtnA) wakes up the full UI.

#include <M5Cardputer.h>
#include <SD.h>
#include <StreamString.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <time.h>

#include "config.h"
#include "gps.h"
#include "logger.h"
#include "motion.h"
#include "nmea_server.h"
#include "tracks.h"
#include "version.h"
#include "web.h"

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

// Recording filter. A fix with a worse HDOP than this (~25 m) is never recorded.
static const float MAX_HDOP = 5.0f;
// Smart mode: a point at turns and speed changes, but not for moves below
// the fix accuracy (noise), and at least every SMART_MAX_SEC / SMART_MAX_DIST_M.
static const float SMART_MIN_DIST_M = 5;
static const float SMART_MAX_DIST_M = 200;
static const uint32_t SMART_MAX_SEC = 60;
static const float SMART_TURN_DEG = 20;
static const float SMART_TURN_MIN_KMH = 3;    // course over ground is noise below this
static const float SMART_SPEED_KMH = 5;       // speed change: at least this ...
static const float SMART_SPEED_FRACTION = 0.25f;  // ... and this part of the last speed
// Auto pause: paused after PAUSE_AFTER_SEC below PAUSE_KMH, resumed at
// RESUME_KMH or when moved more than RESUME_DIST_M (slow walk, low speed readings).
static const float PAUSE_KMH = 1.5f;
static const float RESUME_KMH = 3;
static const uint32_t PAUSE_AFTER_SEC = 10;
static const float RESUME_DIST_M = 20;
// GPS standby: after STANDBY_AFTER_STILL_MS without motion the GPS module is sent
// to timed standby for GPS_STANDBY_SEC ($PCAS12, CASIC modules: ATGM336H, AT6558,
// AT6668), and again each time it wakes up while still not moving. It can't be
// woken early, so after starting to move it takes up to GPS_STANDBY_SEC plus a
// hot fix until positions come again. A module still sending NMEA
// STANDBY_CHECK_MS after the command doesn't support it.
static const uint32_t STANDBY_AFTER_STILL_MS = 30000;
static const uint16_t GPS_STANDBY_SEC = 15;
static const uint32_t STANDBY_CHECK_MS = 3500;
// Recording LED: fades in and out (LED_FADE_MS each) once every LED_PERIOD_MS while logging.
static const uint32_t LED_PERIOD_MS = 60000;
static const uint32_t LED_FADE_MS = 1000;
static const uint8_t LED_MAX = 160;  // of 255, red

// Survives deep sleep.
struct RtcState {
  uint32_t magic;
  bool logging;
  bool isAdv;
  char baseName[48];
  uint32_t points;
  uint32_t detectedBaud;
  uint64_t wakeAtMs;  // planned timer wake-up, on the RTC clock (rtcNowMs)
  // Recording filter: last recorded point and auto pause state
  bool hasLast;
  double lastLat, lastLon;
  float lastSpeedKmh, lastCourseDeg;
  uint64_t lastSavedMs;   // rtcNowMs
  bool paused;
  uint64_t stillSinceMs;  // rtcNowMs, 0 = moving
};
RTC_DATA_ATTR RtcState rtc;

enum Page {
  PAGE_MAIN, PAGE_BIG, PAGE_DETAILS, PAGE_SATS, PAGE_SIGNAL, PAGE_LIVE_INFO, PAGE_LIVE_MAP, PAGE_GG,
  PAGE_ACCEL, PAGE_WIFI, PAGE_MENU, PAGE_SETTINGS, PAGE_HELP, PAGE_TRACKS, PAGE_TRACK_INFO, PAGE_TRACK_MAP
};

// Main pages (left / right, number keys) with their sub pages (up / down).
// Each group remembers its last sub page. WIFI only with the NMEA server on,
// ACCEL only with an accelerometer.
enum PageGroupId { G_POS, G_GPS, G_TRACK, G_ACCEL, G_WIFI, G_MENU, G_COUNT };
struct PageGroup {
  Page pages[3];
  uint8_t count;
};
static const PageGroup GROUPS[G_COUNT] = {
  {{PAGE_MAIN, PAGE_BIG}, 2},
  {{PAGE_DETAILS, PAGE_SATS, PAGE_SIGNAL}, 3},
  {{PAGE_LIVE_INFO, PAGE_LIVE_MAP}, 2},
  {{PAGE_GG, PAGE_ACCEL}, 2},
  {{PAGE_WIFI}, 1},
  {{PAGE_MENU}, 1},
};
static uint8_t groupSub[G_COUNT];  // last sub page of each group

static Config cfg;
static GpsReader gps;
static M5Canvas canvas(&M5.Display);
static bool sdOk = false;
static bool hasKeyboard = false;
static Page page = PAGE_MAIN;
static Page returnPage = PAGE_MAIN;  // where settings / help / tracks go back to
static int menuSel = 0;
static uint32_t lastLogMs = 0;       // 0 = nothing logged since start/wake
static uint32_t lastInputMs = 0;
static uint32_t lastDrawMs = 0;
static bool screenOn = true;
static bool keepScreenOn = false;  // Z while the screen is off: no timeout until Z or restart
static char statusMsg[40] = "";
static uint32_t statusMsgUntil = 0;
static int settingsSel = 0;
static bool editing = false;   // text input for the selected setting
static String editBuf;
static int lastDeviceCount = 0;
static Config cfgBeforeEdit;
static int sdErrors = 0;       // failed point writes in a row
static std::vector<tracks::FileInfo> trackFiles;
static int trackSel = 0;
static bool trackDeleteAsk = false;
static int trackAction = -1;   // selection in the action popup, -1 = closed
static tracks::Track track;    // loaded for the info and map pages
// The track being logged, for its page: read again every LIVE_READ_MS while
// the page is shown, only the new points.
static const uint32_t LIVE_READ_MS = 5000;
static tracks::Track liveTrack;
static tracks::LoadState liveState;
static char liveBase[48] = "";  // liveTrack is from this track
static uint32_t livePoints = 0, liveReadMs = 0;
static uint32_t gpsStandbyMs = 0;     // when the standby command was sent, 0 = GPS on
static uint32_t gpsStandbyEndMs = 0;  // when the GPS was taken as on again
static bool gpsStandbyChecked = false;
static bool gpsStandbyWorked = false;
static bool gpsStandbyFailed = false;  // module doesn't support standby
static uint32_t ledStartMs = 0;        // first pulse when logging starts

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
  return rtc.logging && cfg.deepSleep && cfg.logMode == LOG_MODE_TIME &&
         cfg.logIntervalSec >= MIN_SLEEP_INTERVAL_SEC && sdOk &&
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

// UTC date/time from the GPS shifted by the configured offset (incl. daylight saving).
static bool localTime(struct tm& out, int16_t& offsetMin) {
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
  offsetMin = tzOffsetAt(cfg, epoch);
  epoch += (time_t)offsetMin * 60;
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
  p.hasAccel = false;
  return true;
}

// LOG_SKIPPED: fix not accurate enough or not a key point, try the next one.
// LOG_PAUSED: standing still, counts like a logged point for the interval.
enum LogResult { LOG_OK, LOG_NO_FIX, LOG_SKIPPED, LOG_PAUSED, LOG_SD_ERROR };

static float angleDiff(float a, float b) {
  return fabsf(fmodf(a - b + 540, 360) - 180);
}

// Smart mode: turn, speed change, or too long since the last point.
static bool isKeyPoint(const LogPoint& p, double distM, uint64_t now) {
  if (distM < max(SMART_MIN_DIST_M, p.hdop * HDOP_UERE_M)) return false;
  if (distM >= SMART_MAX_DIST_M || now - rtc.lastSavedMs >= SMART_MAX_SEC * 1000ULL) return true;
  if (fabsf(p.speedKmh - rtc.lastSpeedKmh) >= max(SMART_SPEED_KMH, rtc.lastSpeedKmh * SMART_SPEED_FRACTION))
    return true;
  return p.speedKmh >= SMART_TURN_MIN_KMH && rtc.lastSpeedKmh >= SMART_TURN_MIN_KMH &&
         angleDiff(p.courseDeg, rtc.lastCourseDeg) >= SMART_TURN_DEG;
}

// Whether a new fix is recorded. Time mode only gets here when the interval is due.
static LogResult filterPoint(const LogPoint& p) {
  if (p.hdop <= 0 || p.hdop > MAX_HDOP) return LOG_SKIPPED;
  uint64_t now = rtcNowMs();
  double distM = rtc.hasLast ? TinyGPSPlus::distanceBetween(rtc.lastLat, rtc.lastLon, p.lat, p.lon) : 0;
  if (!cfg.autoPause) {
    rtc.paused = false;
  } else if (p.speedKmh < PAUSE_KMH) {
    if (!rtc.stillSinceMs) rtc.stillSinceMs = now;
    if (now - rtc.stillSinceMs >= PAUSE_AFTER_SEC * 1000ULL) rtc.paused = true;
  } else {
    rtc.stillSinceMs = 0;
    if (p.speedKmh >= RESUME_KMH) rtc.paused = false;
  }
  if (rtc.paused && distM > max(RESUME_DIST_M, p.hdop * HDOP_UERE_M)) {
    rtc.paused = false;
    rtc.stillSinceMs = 0;
  }
  if (rtc.paused) return LOG_PAUSED;
  if (!rtc.hasLast) return LOG_OK;
  switch (cfg.logMode) {
    case LOG_MODE_SMART: return isKeyPoint(p, distM, now) ? LOG_OK : LOG_SKIPPED;
    case LOG_MODE_DISTANCE: return distM >= cfg.logDistanceM ? LOG_OK : LOG_SKIPPED;
    default: return LOG_OK;
  }
}

static LogResult logPoint(int batteryPct) {
  LogPoint p;
  if (!buildPoint(p, batteryPct)) return LOG_NO_FIX;
  LogResult result = filterPoint(p);
  if (result != LOG_OK) return result;
  p.hasAccel = motion::calibrated();
  if (p.hasAccel) motion::takePeaks(p.accelG, p.brakeG, p.latG, p.upG, p.downG);
  if (!logger::write(p, cfg.logFormat, rtc.baseName, sizeof(rtc.baseName))) return LOG_SD_ERROR;
  rtc.points++;
  rtc.hasLast = true;
  rtc.lastLat = p.lat;
  rtc.lastLon = p.lon;
  rtc.lastSpeedKmh = p.speedKmh;
  rtc.lastCourseDeg = p.courseDeg;
  rtc.lastSavedMs = rtcNowMs();
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
  rtc.hasLast = false;
  rtc.paused = false;
  rtc.stillSinceMs = 0;
  lastLogMs = 0;
  ledStartMs = millis();
  motion::resetMax();
  showStatus("Logging started");
}

static void stopLogging() {
  if (rtc.baseName[0]) logger::nmeaFlush(rtc.baseName);  // also empties the buffer on SD errors
  rtc.logging = false;
  showStatus("Logging stopped");
}

// ---------------------------------------------------------------- GPS standby

// Not with deep sleep (has its own wake-ups) or the NMEA server (devices want positions).
static void updateGpsStandby() {
  if (!cfg.gpsStandby || gpsStandbyFailed || !motion::available() || canDeepSleep() || cfg.nmeaServer) {
    gpsStandbyMs = 0;
    return;
  }
  uint32_t now = millis();
  if (gpsStandbyMs) {
    uint32_t since = now - gpsStandbyMs;
    if (!gpsStandbyChecked && since >= STANDBY_CHECK_MS) {
      gpsStandbyChecked = true;
      if ((int32_t)(gps.lastDataMs() - gpsStandbyMs) > (int32_t)(STANDBY_CHECK_MS - 1500)) {
        gpsStandbyMs = 0;
        gpsStandbyEndMs = now;
        if (!gpsStandbyWorked) {  // a module that did standby before just missed one command
          gpsStandbyFailed = true;
          showStatus("GPS has no standby", 4000);
        }
        return;
      }
      gpsStandbyWorked = true;
    }
    // The module wakes up by itself after GPS_STANDBY_SEC
    if (since >= GPS_STANDBY_SEC * 1000UL || !motion::still()) {
      gpsStandbyMs = 0;
      gpsStandbyEndMs = now;
    }
    return;
  }
  bool slow = !gps.hasFix() || gps.gps().speed.kmph() < PAUSE_KMH;  // the GPS agrees
  bool awake = (int32_t)(gps.lastDataMs() - gpsStandbyEndMs) > 0;   // can receive the command
  if (motion::stillMs() >= STANDBY_AFTER_STILL_MS && slow && awake && gps.baudLocked()) {
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "PCAS12,%u", GPS_STANDBY_SEC);
    gps.sendCommand(cmd);
    gpsStandbyMs = now | 1;  // never 0
    gpsStandbyChecked = false;
    if (rtc.logging && cfg.autoPause) rtc.paused = true;
  }
}

// ---------------------------------------------------------------- deep sleep

[[noreturn]] static void enterDeepSleep(uint32_t sleepMs, bool displayActive) {
  gps.end();
  if (sdOk && rtc.baseName[0]) logger::nmeaFlush(rtc.baseName);
  SD.end();
  if (displayActive) {  // M5 is set up: not in a quiet wake-up
    M5.Display.setBrightness(0);
    M5.Display.sleep();
    M5.Led.setAllColor(0, 0, 0);  // may be in the middle of a pulse
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
      // Inaccurate fix: wait for a better one. SD error: try again at the next wake-up.
      if (result != LOG_NO_FIX && result != LOG_SKIPPED) break;
    }
    delay(5);
  }
  log_i("Quiet wake: %s after %lu ms",
        result == LOG_OK ? "logged" : result == LOG_PAUSED ? "paused" : result == LOG_SD_ERROR ? "SD error" : "no fix",
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

// Small sun: screen kept on with Z.
static const int SUN_W = 11;
static void drawSun(int x) {
  const int cx = x + 5, cy = 9;
  canvas.fillCircle(cx, cy, 2, TFT_YELLOW);
  canvas.drawFastHLine(x, cy, 2, TFT_YELLOW);
  canvas.drawFastHLine(x + 9, cy, 2, TFT_YELLOW);
  canvas.drawFastVLine(cx, cy - 5, 2, TFT_YELLOW);
  canvas.drawFastVLine(cx, cy + 4, 2, TFT_YELLOW);
}

static uint16_t satColor(int sats) {
  return sats >= 7 ? TFT_GREEN : sats >= 4 ? TFT_YELLOW : TFT_RED;
}

// Left: recording state. Right to left: battery, screen kept on, NMEA server, SD (only if
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
    String rec = (rtc.paused ? "PAUSE " : "REC ") + String(rtc.points);
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
  if (gpsStandbyMs) {
    sat = "GPS STBY";
    satCol = COL_LABEL;
  } else if (!gps.baudLocked()) {
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
  if (keepScreenOn) need += gap + SUN_W;
  bool withCount = satCount.length() && need + canvas.textWidth(satCount) <= right;
  if (withCount) need += canvas.textWidth(satCount);
  bool withPct = need + 3 + canvas.textWidth(pctText) <= right;

  int x = drawBattery(right, pct, withPct) - gap;
  if (keepScreenOn) {
    x -= SUN_W;
    drawSun(x);
    x -= gap;
  }
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
  int16_t tz;
  if (localTime(t, tz)) {
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    drawRow(106, "TIME", buf, COL_TIME);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COL_LABEL);
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    canvas.drawString(buf, 160, 110);
    canvas.drawString("UTC" + tzText(tz), 160, 121);
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
  String mode = cfg.logMode == LOG_MODE_TIME ? String(cfg.logIntervalSec) + "s"
                : cfg.logMode == LOG_MODE_DISTANCE ? String(cfg.logDistanceM) + "m" : String("smart");
  line(TFT_ORANGE, "Log %s%s %s sleep:%s", mode.c_str(), cfg.autoPause ? "+pause" : "",
       LOG_FORMAT_NAMES[cfg.logFormat], canDeepSleep() ? "on" : cfg.deepSleep ? "n/a" : "off");
  const char* name = strrchr(rtc.baseName, '/');
  line(TFT_MAGENTA, "File %s (%lu)", name ? name + 1 : "-", (unsigned long)rtc.points);
  int mv = batteryMvUi();
  line(TFT_GREEN, "Battery %.2fV %d%%  %s", mv / 1000.0f, batteryPctFromMv(mv),
       hasKeyboard ? "KB" : "no KB");
  line(TFT_SKYBLUE, "WiFi %s %s", !nmeaserver::running() ? "" : nmeaserver::apActive() ? "AP" : "client",
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
  bool ap = nmeaserver::apActive();
  canvas.drawString(!ap ? "NMEA SERVER - joins WiFi"
                    : nmeaserver::apFallback() ? "NMEA SERVER - own WiFi (fallback)" : "NMEA SERVER - own WiFi",
                    3, 20);

  if (!ap) {
    String name = nmeaserver::networkName();
    drawLabeled(37, "Network", name.length() ? name : String("-"), TFT_WHITE);
    drawLabeled(53, "Status", nmeaserver::stateText(), up ? TFT_GREEN : TFT_ORANGE);
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

  drawLabeled(97, "Port", String(cfg.nmeaPort) + " (TCP+UDP)  web 80", TFT_YELLOW);

  int tcp = nmeaserver::tcpClients();
  String devices = "TCP " + String(tcp);
  if (ap) devices += ", in WiFi " + String(nmeaserver::apStations());
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
  const int width = 236;  // right of it the sub page bar
  int slot = min(16, width / n);
  int barW = max(slot - 2, 1);
  int x0 = (width - slot * n) / 2;
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

// ---- acceleration pages

static const uint16_t COL_LON = TFT_CYAN;
static const uint16_t COL_LAT = TFT_YELLOW;
static const uint16_t COL_VERT = TFT_MAGENTA;
static const uint16_t COL_ENVELOPE = TFT_ORANGE;

// Smallest of 0.5 / 1 / 2 g that holds g.
static float accelScale(float g) {
  return g <= 0.45f ? 0.5f : g <= 0.9f ? 1.0f : 2.0f;
}

// Calibration state in the bottom right corner; false if not calibrated.
static bool drawAccelState(int x, int y) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_right);
  bool cal = motion::calibrated();
  canvas.setTextColor(!cal ? TFT_ORANGE : motion::forwardLearned() ? COL_LABEL : TFT_ORANGE);
  canvas.drawString(!cal ? "C calibrate" : motion::forwardLearned() ? "C cal X reset" : "learning fwd", x, y);
  canvas.setTextDatum(top_left);
  return cal;
}

static void drawNotCalibrated() {
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("Accelerometer not calibrated:", 3, 26);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("Fix it in the car, stand still", 3, 46);
  canvas.drawString("and press C. Forward is learned", 3, 62);
  canvas.drawString("from the GPS when driving.", 3, 78);
  canvas.setTextColor(motion::still() ? TFT_GREEN : TFT_ORANGE);
  canvas.drawString(motion::still() ? "still - ready" : "moving", 3, 100);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("vib " + String(motion::vibration() * 1000, 0) + " mg", 3, 124);
  drawAccelState(237, 124);
}

// ".32" below 1 g, "1.3" above: short enough for 6 values in one Font0 line.
static String shortG(float g) {
  if (g >= 0.995f) return String(g, 1);
  char buf[8];
  snprintf(buf, sizeof(buf), ".%02d", (int)lroundf(g * 100));
  return buf;
}

// G-G diagram: forward is up, right is right. Dot = now with a 10 s trail,
// orange = max per direction. Right of it a bar for up / down, then the values.
static void drawGgPage() {
  if (!motion::calibrated()) { drawNotCalibrated(); return; }
  const motion::Maxima& m = motion::maxima();
  const int cx = 57, cy = 77, r = 54;
  float scale = accelScale(max(max(m.accel, m.brake), max(m.left, m.right)));
  float k = r / scale;
  canvas.drawCircle(cx, cy, r, COL_LABEL);
  canvas.drawCircle(cx, cy, r / 2, COL_BAR);
  canvas.drawFastHLine(cx - r, cy, 2 * r + 1, COL_BAR);
  canvas.drawFastVLine(cx, cy - r, 2 * r + 1, COL_BAR);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString(String(scale, 1) + "g", cx + r - 18, cy - r);

  auto sectorPoint = [&](int s, int& x, int& y) {
    float a = s * TWO_PI / motion::SECTORS, g = min(motion::envelope(s), scale);
    x = cx + sinf(a) * g * k;
    y = cy - cosf(a) * g * k;
  };
  for (int s = 0; s < motion::SECTORS; s++) {
    int n = (s + 1) % motion::SECTORS;
    if (!motion::envelope(s) || !motion::envelope(n)) continue;
    int x0, y0, x1, y1;
    sectorPoint(s, x0, y0);
    sectorPoint(n, x1, y1);
    canvas.drawLine(x0, y0, x1, y1, COL_ENVELOPE);
  }

  auto toX = [&](float lat) { return cx + (int)(constrain(lat, -scale, scale) * k); };
  auto toY = [&](float lon) { return cy - (int)(constrain(lon, -scale, scale) * k); };
  int count = motion::historyCount();
  const int trail = 100;  // 10 samples per second
  for (int i = max(0, count - trail); i < count; i++) {
    motion::Sample smp = motion::historyAt(i);
    uint8_t grey = 24 + 104 * (trail - (count - i)) / trail;  // fades out to almost black, newest = COL_LABEL
    canvas.fillCircle(toX(smp.lat / 1000.0f), toY(smp.lon / 1000.0f), 1, canvas.color565(grey, grey, grey));
  }
  canvas.fillCircle(toX(motion::latG()), toY(motion::lonG()), 4, TFT_RED);

  // Up / down bar with its own scale: filled = now, orange ticks = max
  const int bx = 115, bw = 7;
  float vert = motion::vertG();
  float vScale = accelScale(max(max(m.up, m.down), fabsf(vert)));
  auto toBarY = [&](float g) { return cy - (int)(constrain(g, -vScale, vScale) / vScale * r); };
  canvas.drawRect(bx, cy - r, bw, 2 * r + 1, COL_LABEL);
  int vy = toBarY(vert);
  canvas.fillRect(bx + 1, min(vy, cy), bw - 2, abs(vy - cy) + 1, COL_VERT);
  canvas.drawFastHLine(bx - 2, cy, bw + 4, COL_LABEL);
  if (m.up) canvas.drawFastHLine(bx - 2, toBarY(m.up), bw + 4, COL_ENVELOPE);
  if (m.down) canvas.drawFastHLine(bx - 2, toBarY(-m.down), bw + 4, COL_ENVELOPE);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextDatum(top_center);
  canvas.drawString(String(vScale, 1), bx + bw / 2, 124);
  canvas.setTextDatum(top_left);

  // Values: now (g, with sign) and max pairs
  const int lx = 128, vx = 237;
  auto value = [&](int y, const char* label, const String& text, uint16_t col) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COL_LABEL);
    canvas.drawString(label, lx, y);
    canvas.setTextColor(col);
    canvas.setTextDatum(top_right);
    canvas.drawString(text, vx, y);
    canvas.setTextDatum(top_left);
  };
  auto now = [](float g) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%+.2f g", g);
    return String(buf);
  };
  value(20, "Lon", now(motion::lonG()), COL_LON);
  value(35, "Lat", now(motion::latG()), COL_LAT);
  value(50, "Vert", now(vert), COL_VERT);
  canvas.drawFastHLine(lx, 68, vx - lx, COL_BAR);
  value(71, "A/B", String(m.accel, 2) + "/" + String(m.brake, 2), TFT_GREEN);
  value(86, "L/R", String(m.left, 2) + "/" + String(m.right, 2), COL_ENVELOPE);
  value(101, "U/D", String(m.up, 2) + "/" + String(m.down, 2), COL_VERT);
  drawAccelState(vx, 124);
}

// Longitudinal, lateral and vertical acceleration of the last 24 s, 1 px = 100 ms.
static void drawAccelChartPage() {
  if (!motion::calibrated()) { drawNotCalibrated(); return; }
  const int top = 21, bottom = 119, mid = (top + bottom) / 2, half = (bottom - top) / 2;
  int count = motion::historyCount();
  int peak = 0;
  for (int i = 0; i < count; i++) {
    motion::Sample smp = motion::historyAt(i);
    peak = max(peak, max(max(abs(smp.lon), abs(smp.lat)), abs(smp.vert)));
  }
  float scale = accelScale(peak / 1000.0f);
  canvas.drawFastHLine(0, mid, 240, COL_LABEL);
  canvas.drawFastHLine(0, mid - half / 2, 240, COL_BAR);
  canvas.drawFastHLine(0, mid + half / 2, 240, COL_BAR);
  auto toY = [&](int16_t mg) { return mid - (int)(constrain(mg / 1000.0f, -scale, scale) / scale * half); };
  int x0 = 236 - count;  // newest sample left of the sub page bar, the oldest ones may be cut off
  for (int i = 1; i < count; i++) {
    motion::Sample a = motion::historyAt(i - 1), b = motion::historyAt(i);
    canvas.drawLine(x0 + i - 1, toY(a.vert), x0 + i, toY(b.vert), COL_VERT);
    canvas.drawLine(x0 + i - 1, toY(a.lat), x0 + i, toY(b.lat), COL_LAT);
    canvas.drawLine(x0 + i - 1, toY(a.lon), x0 + i, toY(b.lon), COL_LON);
  }
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("+" + String(scale, 1) + "g", 2, top);
  canvas.drawString("-" + String(scale, 1) + "g", 2, bottom - 8);

  const motion::Maxima& m = motion::maxima();
  canvas.setTextColor(COL_LON);
  canvas.drawString("lon", 2, 124);
  canvas.setTextColor(COL_LAT);
  canvas.drawString("lat", 23, 124);
  canvas.setTextColor(COL_VERT);
  canvas.drawString("up", 44, 124);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("A" + shortG(m.accel) + " B" + shortG(m.brake) + " L" + shortG(m.left) + " R" +
                        shortG(m.right) + " U" + shortG(m.up) + " D" + shortG(m.down),
                    62, 124);
  if (!motion::forwardLearned()) {
    canvas.setTextColor(TFT_ORANGE);
    canvas.setTextDatum(top_right);
    canvas.drawString("learning fwd", 237, top);
    canvas.setTextDatum(top_left);
  }
}

// ---------------------------------------------------------------- settings

enum SettingId {
  S_LOGMODE, S_INTERVAL, S_DISTANCE, S_AUTOPAUSE, S_FORMAT, S_NMEA, S_SLEEP, S_SCREEN, S_FIXTO,
  S_MODULE, S_RX, S_TX, S_BAUD, S_STANDBY,
  S_SERVER, S_WIFIMODE, S_PORT, S_SSID, S_PASS, S_SSID2, S_PASS2, S_SSID3, S_PASS3, S_APSSID, S_APPASS,
  S_TZ, S_DST, S_AUTOSTART, S_BRIGHT, S_RECLED, S_COUNT
};
static const char* const SETTING_NAMES[S_COUNT] = {
  "Log mode", "Log interval", "Log distance", "Auto pause", "Log format", "NMEA log", "Deep sleep",
  "Screen timeout", "Fix timeout", "GPS module",
  "GPS RX pin", "GPS TX pin", "GPS baud", "GPS standby", "NMEA server", "WiFi mode", "NMEA port",
  "WiFi 1 SSID", "WiFi 1 password", "WiFi 2 SSID", "WiFi 2 password", "WiFi 3 SSID", "WiFi 3 password",
  "AP name", "AP password", "Time zone", "Summer time", "Auto start", "Brightness",
  "Recording LED",
};
static const uint32_t INTERVALS[] = {1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600};
static const uint32_t DISTANCES[] = {5, 10, 20, 50, 100, 200, 500, 1000};
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

static String* textSetting(int id);
static bool isSsidSetting(int id);

static String settingValue(int id) {
  switch (id) {
    case S_LOGMODE: return LOG_MODE_NAMES[cfg.logMode];
    case S_INTERVAL: return secondsText(cfg.logIntervalSec);
    case S_DISTANCE: return String(cfg.logDistanceM) + " m";
    case S_AUTOPAUSE: return cfg.autoPause ? "on" : "off";
    case S_FORMAT: return LOG_FORMAT_NAMES[cfg.logFormat];
    case S_NMEA: return cfg.logNmea ? "on" : "off";
    case S_SLEEP:
      if (!cfg.deepSleep) return "off";
      if (cfg.nmeaServer) return "on (WiFi on)";
      if (cfg.logMode != LOG_MODE_TIME) return "on (time mode)";
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
    case S_STANDBY:
      if (!cfg.gpsStandby) return "off";
      if (!motion::available()) return "on (no IMU)";
      if (gpsStandbyFailed) return "on (unsupported)";
      if (cfg.nmeaServer) return "on (WiFi on)";
      return cfg.deepSleep && cfg.logMode == LOG_MODE_TIME ? "on (deep sleep)" : "on";
    case S_TZ: return "UTC" + tzText(cfg.tzOffsetMin);
    case S_DST: return DST_NAMES[cfg.dstRule];
    case S_AUTOSTART: return cfg.autoStart ? "on" : "off";
    case S_BRIGHT: return String(cfg.brightness * 100 / 255) + " %";
    case S_RECLED: return cfg.recLed ? "on" : "off";
    case S_SERVER: return cfg.nmeaServer ? "on" : "off";
    case S_WIFIMODE: return cfg.wifiClient ? "client" : "own AP";
    case S_PORT: return String(cfg.nmeaPort);
    case S_SSID: case S_SSID2: case S_SSID3:
    case S_PASS: case S_PASS2: case S_PASS3: {
      const String& v = *textSetting(id);
      return !v.length() ? String("-") : isSsidSetting(id) ? v : String("****");
    }
    case S_APSSID: return cfg.apSsid;
    case S_APPASS: return cfg.apPass.length() ? String("****") : String("(open)");
  }
  return "";
}

// WiFi settings complete enough to start the NMEA server.
static bool wifiConfigured() {
  if (cfg.wifiClient) {
    for (const String& ssid : cfg.wifiSsid)
      if (ssid.length()) return true;
    return false;
  }
  return cfg.apSsid.length() > 0 && (cfg.apPass.length() == 0 || cfg.apPass.length() >= 8);
}

// ---- text input for SSIDs, passwords and the port

static String* textSetting(int id) {
  switch (id) {
    case S_SSID: return &cfg.wifiSsid[0];
    case S_PASS: return &cfg.wifiPass[0];
    case S_SSID2: return &cfg.wifiSsid[1];
    case S_PASS2: return &cfg.wifiPass[1];
    case S_SSID3: return &cfg.wifiSsid[2];
    case S_PASS3: return &cfg.wifiPass[2];
    case S_APSSID: return &cfg.apSsid;
    case S_APPASS: return &cfg.apPass;
  }
  return nullptr;
}

static bool isTextSetting(int id) { return id == S_PORT || textSetting(id); }

static bool isSsidSetting(int id) { return id == S_SSID || id == S_SSID2 || id == S_SSID3 || id == S_APSSID; }

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
    size_t maxLen = settingsSel == S_PORT ? 5 : isSsidSetting(settingsSel) ? 32 : 63;
    for (char c : word) {
      if (editBuf.length() < maxLen && (settingsSel != S_PORT || isdigit(c))) editBuf += c;
    }
  }
}

static void changeSetting(int id, int dir) {
  switch (id) {
    case S_LOGMODE: cfg.logMode = (cfg.logMode + LOG_MODE_COUNT + dir) % LOG_MODE_COUNT; break;
    case S_INTERVAL: cfg.logIntervalSec = cycle(INTERVALS, cfg.logIntervalSec, dir); break;
    case S_DISTANCE: cfg.logDistanceM = cycle(DISTANCES, cfg.logDistanceM, dir); break;
    case S_AUTOPAUSE: cfg.autoPause = !cfg.autoPause; break;
    case S_FORMAT: cfg.logFormat = (cfg.logFormat + LOG_FORMAT_COUNT + dir) % LOG_FORMAT_COUNT; break;
    case S_NMEA: cfg.logNmea = !cfg.logNmea; break;
    case S_SLEEP: cfg.deepSleep = !cfg.deepSleep; break;
    case S_SCREEN: cfg.screenTimeoutSec = cycle(SCREEN_TOS, cfg.screenTimeoutSec, dir); break;
    case S_FIXTO: cfg.fixTimeoutSec = cycle(FIX_TOS, cfg.fixTimeoutSec, dir); break;
    case S_MODULE: cfg.applyPreset((cfg.gpsModule + GPS_MODULE_COUNT + dir) % GPS_MODULE_COUNT); break;
    case S_RX: cfg.gpsRx = constrain(cfg.gpsRx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_TX: cfg.gpsTx = constrain(cfg.gpsTx + dir, -1, 48); cfg.gpsModule = GPS_CUSTOM; break;
    case S_BAUD: cfg.gpsBaud = cycle(BAUDS, cfg.gpsBaud, dir); break;
    case S_STANDBY: cfg.gpsStandby = !cfg.gpsStandby; break;
    case S_TZ: cfg.tzOffsetMin = cycle(TZ_OFFSETS, cfg.tzOffsetMin, dir); break;
    case S_DST: cfg.dstRule = (cfg.dstRule + DST_COUNT + dir) % DST_COUNT; break;
    case S_AUTOSTART: cfg.autoStart = !cfg.autoStart; break;
    case S_RECLED: cfg.recLed = !cfg.recLed; break;
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

// Restarts what the changes from before to cfg affect and saves cfg.
// Returns the message for the status line.
static const char* applyConfig(const Config& before) {
  bool gpsChanged = cfg.gpsRx != before.gpsRx || cfg.gpsTx != before.gpsTx ||
                    cfg.gpsBaud != before.gpsBaud;
  if (gpsChanged || cfg.gpsModule != before.gpsModule || cfg.gpsStandby != before.gpsStandby) {
    gpsStandbyFailed = gpsStandbyWorked = false;  // try again, maybe another module
  }
  if (gpsChanged) {
    rtc.detectedBaud = 0;
    startGps();
  }
  bool wifiChanged = cfg.nmeaServer != before.nmeaServer ||
                     cfg.wifiClient != before.wifiClient ||
                     cfg.nmeaPort != before.nmeaPort || cfg.apSsid != before.apSsid ||
                     cfg.apPass != before.apPass;
  for (int i = 0; i < WIFI_NETWORKS; i++) {
    wifiChanged |= cfg.wifiSsid[i] != before.wifiSsid[i] || cfg.wifiPass[i] != before.wifiPass[i];
  }
  bool wifiIncomplete = cfg.nmeaServer && !wifiConfigured();
  if (wifiIncomplete) cfg.nmeaServer = false;
  if (wifiChanged || wifiIncomplete) {
    if (cfg.nmeaServer) nmeaserver::begin(cfg);
    else nmeaserver::end();
  }
  bool saved = sdOk && configSave(cfg);
  if (wifiIncomplete) return "WiFi incomplete: server off";
  return saved ? "Settings saved to SD" : "No SD: settings not saved";
}

static void closeSettings() {
  showStatus(applyConfig(cfgBeforeEdit));
  page = returnPage == PAGE_WIFI && !cfg.nmeaServer ? PAGE_MAIN : returnPage;
}

// ---- settings from the web page

static const uint32_t WEB_APPLY_DELAY_MS = 500;  // response sent before the WiFi may restart
static String webConfigLines;     // accepted key=value lines
static uint32_t webConfigMs = 0;  // when they were accepted, 0 = nothing pending

static String webConfigText() { return configText(cfg); }

static String webApplyConfig(const String& lines) {
  Config test = cfg;
  StreamString in;
  in.print(lines);
  configParse(test, in);
  if (test.apPass.length() && test.apPass.length() < 8) return "ap_pass: min. 8 characters";
  webConfigLines = lines;
  webConfigMs = millis() | 1;
  return "";
}

// Called from loop(): applying in the web request would cut it off with a WiFi restart.
static void applyWebConfig() {
  if (!webConfigMs || millis() - webConfigMs < WEB_APPLY_DELAY_MS) return;
  webConfigMs = 0;
  Config before = cfg;
  StreamString in;
  in.print(webConfigLines);
  configParse(cfg, in);
  if (cfg.gpsModule != before.gpsModule) cfg.applyPreset(cfg.gpsModule);  // pins as on the device
  if (cfg.brightness != before.brightness && screenOn) M5.Display.setBrightness(cfg.brightness);
  if (memcmp(cfg.imuUp, before.imuUp, sizeof(cfg.imuUp)) || memcmp(cfg.imuFwd, before.imuFwd, sizeof(cfg.imuFwd)) ||
      cfg.imuFwdLearned != before.imuFwdLearned) {
    motion::setCalibration(cfg.imuUp, cfg.imuFwd, cfg.imuFwdLearned);
  }
  showStatus(applyConfig(before));
  if (page == PAGE_WIFI && !cfg.nmeaServer) page = PAGE_MAIN;
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

// Key, what it does; scrolled with up / down, 5 rows at a time.
static const char* const HELP[][2] = {
  {"Enter  R", "start / stop logging"},
  {",  /  Tab", "main page"},
  {";  .", "sub page"},
  {"1 - 6", "main page"},
  {"G0 click", "next page"},
  {"G0 hold", "start / stop logging"},
  {"G0", "wakes from sleep"},
  {"S", "settings"},
  {"T", "tracks"},
  {"H", "help"},
  {"Z", "screen off / sleep"},
  {"Z (off)", "screen on, stays on"},
  {"P", "screenshot"},
  {"I  M", "track info / map"},
  {"C", "colors, calibrate"},
  {"=  -", "map zoom in / out"},
  {"[  ]", "time cursor"},
  {"{  }", "cursor 10 %"},
  {"F", "map follows you"},
  {"X", "reset max accel."},
  {"`  Esc", "back, reset zoom"},
};
static const int HELP_COUNT = sizeof(HELP) / sizeof(HELP[0]);
static const int HELP_ROWS = 5;
static int helpFirst = 0;  // first row shown

static void drawTitleBar(const String& title, const char* keys);

static void drawHelpPage() {
  drawTitleBar("HELP  v" FW_VERSION, ";. scroll  other: back");
  const int top = 21, rowH = 22;
  canvas.setFont(&fonts::FreeSans9pt7b);
  for (int i = 0; i < HELP_ROWS && helpFirst + i < HELP_COUNT; i++) {
    int y = top + i * rowH;
    canvas.setTextColor(TFT_YELLOW);
    canvas.drawString(HELP[helpFirst + i][0], 3, y);
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(HELP[helpFirst + i][1], 74, y);
  }
  // Scroll bar: the visible part of the list
  const int x = 237, w = 3, h = 134 - top;
  canvas.fillRect(x, top, w, h, COL_BAR);
  canvas.fillRect(x, top + h * helpFirst / HELP_COUNT, w, h * HELP_ROWS / HELP_COUNT, TFT_WHITE);
}

// ---------------------------------------------------------------- menu

enum MenuId { M_LOGGING, M_SERVER, M_SETTINGS, M_TRACKS, M_HELP, M_SLEEP, M_COUNT };

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
// is drawn before all labels and doesn't cover the row above. Only 5 rows
// fit, the list scrolls with the selection.
static void drawMenuPage() {
  const int top = 19, rowH = 23, rows = 5;
  static int first = 0;
  first = constrain(first, menuSel - rows + 1, menuSel);
  canvas.fillRoundRect(2, top + (menuSel - first) * rowH, 236, rowH - 2, 5, COL_SEL);
  if (first > 0) canvas.fillTriangle(229, top + 8, 225, top + 14, 233, top + 14, COL_LABEL);
  if (first + rows < M_COUNT) {
    int y = top + rows * rowH - 9;
    canvas.fillTriangle(229, y + 6, 225, y, 233, y, COL_LABEL);
  }
  for (int id = first; id < first + rows; id++) {
    int y = top + (id - first) * rowH;
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
      case M_TRACKS: label = "Tracks"; col = TFT_GREENYELLOW; break;
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

// ---------------------------------------------------------------- tracks

static void drawMessageBox(const char* msg) {
  canvas.setFont(&fonts::Font2);
  int w = canvas.textWidth(msg) + 16;
  int x = (240 - w) / 2;
  canvas.fillRoundRect(x, 56, w, 24, 5, TFT_NAVY);
  canvas.drawRoundRect(x, 56, w, 24, 5, TFT_WHITE);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString(msg, 120, 68);
  canvas.setTextDatum(top_left);
}

// Title bar of the track pages, like the one of the settings.
static void drawTitleBar(const String& title, const char* keys) {
  canvas.fillRect(0, 0, 240, 18, COL_SEL);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString(title, 3, 1);
  canvas.setTextColor(COL_LABEL);
  canvas.setTextDatum(top_right);
  canvas.drawString(keys, 237, 1);
  canvas.setTextDatum(top_left);
}

static uint16_t trackTypeColor(uint8_t type) {
  return type == tracks::TYPE_CSV ? TFT_CYAN : type == tracks::TYPE_GPX ? TFT_YELLOW : TFT_MAGENTA;
}

// Local date and time, with the configured UTC offset and daylight saving.
static String dateTimeText(time_t utc) {
  if (!utc) return "--";
  time_t local = utc + (time_t)tzOffsetAt(cfg, utc) * 60;
  struct tm t;
  gmtime_r(&local, &t);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  return buf;
}

// Blue (low) - green - red (high), f from 0 to 1.
static uint16_t rangeColor(float f) {
  f = constrain(f, 0.0f, 1.0f);
  if (f < 0.5f) return canvas.color565(0, f * 2 * 255, (1 - f * 2) * 255);
  return canvas.color565((f - 0.5f) * 2 * 255, (1 - (f - 0.5f) * 2) * 255, 0);
}

static float altFraction(const tracks::Track& t, float alt) {
  return t.altMax > t.altMin ? (alt - t.altMin) / (t.altMax - t.altMin) : 0.5f;
}

// Color of a track point: by altitude or by speed (0 .. max), as selected with C.
static uint16_t pointColor(const tracks::Track& t, const tracks::Point& p) {
  if (cfg.mapSpeed) return rangeColor(t.spdMax > 0 ? p.speed / t.spdMax : 0);
  return rangeColor(altFraction(t, p.alt));
}

static void toggleMapColor() {
  cfg.mapSpeed = !cfg.mapSpeed;
  if (sdOk) configSave(cfg);
  showStatus(cfg.mapSpeed ? "Colors: speed" : "Colors: altitude");
}

enum TrackActionId { A_INFO, A_MAP, A_DELETE, A_COUNT };
static const char* const TRACK_ACTION_NAMES[A_COUNT] = {"Info", "Map", "Delete"};
static const char* const TRACK_ACTION_KEYS[A_COUNT] = {"I", "M", "D"};  // shortcuts in the list

// Actions for the selected file, with their shortcut keys on the right.
static void drawTrackActions() {
  const int x = 120, y = 34, w = 110, rowH = 18;
  bool nmea = trackFiles[trackSel].type == tracks::TYPE_NMEA;
  canvas.fillRoundRect(x, y, w, A_COUNT * rowH + 6, 5, TFT_NAVY);
  canvas.drawRoundRect(x, y, w, A_COUNT * rowH + 6, 5, TFT_WHITE);
  for (int a = 0; a < A_COUNT; a++) {
    int ry = y + 3 + a * rowH;
    bool sel = a == trackAction;
    bool disabled = nmea && a != A_DELETE;
    if (sel) canvas.fillRoundRect(x + 3, ry, w - 6, rowH, 3, COL_SEL);
    canvas.setTextColor(disabled ? 0x4208 : sel ? TFT_WHITE : COL_LABEL);
    canvas.drawString(TRACK_ACTION_NAMES[a], x + 10, ry + 1);
    canvas.setTextColor(COL_LABEL);
    canvas.setTextDatum(top_right);
    canvas.drawString(TRACK_ACTION_KEYS[a], x + w - 10, ry + 1);
    canvas.setTextDatum(top_left);
  }
}

static void drawTracksPage() {
  drawTitleBar("TRACKS", ";. select Ent actions");
  int n = trackFiles.size();
  if (n == 0) {
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("No tracks on the SD card", 6, 22);
    return;
  }
  const int rows = 7, rowH = 16;
  int first = constrain(trackSel - rows / 2, 0, max(0, n - rows));
  for (int i = first; i < min(n, first + rows); i++) {
    const tracks::FileInfo& f = trackFiles[i];
    int y = 20 + (i - first) * rowH;
    bool sel = i == trackSel;
    if (sel) canvas.fillRoundRect(0, y, 240, rowH, 3, COL_SEL);
    canvas.setTextColor(sel ? TFT_WHITE : COL_LABEL);
    canvas.drawString(f.name.substring(0, f.name.lastIndexOf('.')), 6, y);
    canvas.setTextColor(trackTypeColor(f.type));
    canvas.drawString(tracks::TYPE_NAMES[f.type], 130, y);
    canvas.setTextColor(sel ? TFT_WHITE : COL_LABEL);
    canvas.setTextDatum(top_right);
    canvas.drawString(tracks::sizeText(f.size), 236, y);
    canvas.setTextDatum(top_left);
  }
  if (trackAction >= 0) drawTrackActions();
  if (trackDeleteAsk) drawMessageBox(("Delete " + trackFiles[trackSel].name + "? Y / N").c_str());
}

// Large value with a small label in front, like on the position page; returns the right edge.
static int drawTrackValue(int x, int y, const char* label, const String& value, uint16_t col) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString(label, x, y + 9);
  x += canvas.textWidth(label) + 4;
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(col);
  canvas.drawString(value, x, y);
  return x + canvas.textWidth(value);
}

// Map zoom and pan. At zoom 1 the whole track is shown and the arrow keys
// switch pages; zoomed in they pan. The current track's map follows the
// latest point until it is panned.
// Time cursor ([ ]) on the altitude profile and the map, kept as a time so it
// stays on its point when the current track grows or gets thinned out.
static const int MAP_X = 4, MAP_Y = 21, MAP_W = 232, MAP_H = 95;
static const uint8_t MAP_MAX_ZOOM = 64;
static const int PROFILE_X = 3, PROFILE_W = 234;
struct MapView {
  uint8_t zoom;
  bool follow;     // current track: centered on the latest point
  float lat, lon;  // center while zoomed
  bool cursor;
  time_t cursorTime;
};
static MapView trackView = {1, false, 0, 0, false, 0};
static MapView liveView = {1, true, 0, 0, false, 0};

// Point nearest to the cursor time; the points are in time order.
static int cursorIndex(const tracks::Track& t, time_t time) {
  int lo = 0, hi = t.n - 1;
  while (lo < hi) {
    int mid = (lo + hi) / 2;
    if (t.pts[mid].time < time) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0 && time - t.pts[lo - 1].time <= t.pts[lo].time - time) lo--;
  return lo;
}

// [ ] move the cursor by one profile pixel, { } (shift) by a tenth of the
// track; the first key puts it at the end ([) or the start (]). Esc hides it.
// A zoomed map is centered on the cursor.
static bool cursorKey(MapView& v, const tracks::Track& t, char c) {
  if (c == '`' && v.cursor) {
    v.cursor = false;
    return true;
  }
  int dir = c == '[' || c == '{' ? -1 : c == ']' || c == '}' ? 1 : 0;
  if (!dir || !t.n) return false;
  int step = c == '{' || c == '}' ? max(1, t.n / 10) : max(1, (t.n + PROFILE_W - 1) / PROFILE_W);
  int i = !v.cursor ? (dir < 0 ? t.n - 1 : 0) : constrain(cursorIndex(t, v.cursorTime) + dir * step, 0, t.n - 1);
  v.cursor = true;
  v.cursorTime = t.pts[i].time;
  if (v.zoom > 1) {
    v.follow = false;
    v.lat = t.pts[i].lat;
    v.lon = t.pts[i].lon;
  }
  return true;
}

// "14:23:05  432 m  57 km/h" of the point at the cursor.
static String cursorText(const tracks::Track& t, const MapView& v) {
  const tracks::Point& p = t.pts[cursorIndex(t, v.cursorTime)];
  String time = p.time ? dateTimeText(p.time).substring(11) : String("--:--:--");
  return time + "  " + String(p.alt, 0) + " m  " + String(p.speed, 0) + " km/h";
}

static void drawTrackInfo(const tracks::Track& t, const MapView& v) {
  String duration = "--";
  if (t.start && t.end >= t.start) {
    uint32_t s = t.end - t.start;
    char buf[16];
    if (s >= 3600) snprintf(buf, sizeof(buf), "%luh %02lum", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60));
    else snprintf(buf, sizeof(buf), "%lum %02lus", (unsigned long)(s / 60), (unsigned long)(s % 60));
    duration = buf;
  }
  String dist = String(t.distM / 1000, t.distM < 10000 ? 2 : 1) + " km";
  drawTrackValue(3, 20, "DIST", dist, TFT_CYAN);
  canvas.setTextDatum(top_right);
  canvas.drawString(duration, 237, 20);
  canvas.setTextDatum(top_left);

  int x = drawTrackValue(3, 46, "ALT", String(t.altMin, 0) + "-" + String(t.altMax, 0) + " m", COL_ALT);
  String spd = String(t.spdMax, 0) + " km/h";
  canvas.setTextColor(TFT_GREEN);
  canvas.setTextDatum(top_right);
  canvas.drawString(spd, 237, 46);
  canvas.setTextDatum(top_left);
  int sx = 237 - canvas.textWidth(spd);
  canvas.setFont(&fonts::Font0);
  if (sx - 4 - canvas.textWidth("MAX") > x + 4) {  // label only if there is room
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("MAX", sx - 4 - canvas.textWidth("MAX"), 55);
  }

  // Local start date and time - end time, points
  String when = "--";
  if (t.start) {
    when = dateTimeText(t.start).substring(0, 16);
    if (t.end) when += " - " + dateTimeText(t.end).substring(11, 16);
  }
  canvas.setFont(&fonts::Font2);
  if (v.cursor) {  // instead of start / end: the point at the cursor
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(cursorText(t, v), 3, 73);
  } else {
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(when, 3, 73);
    canvas.setTextColor(COL_LABEL);
    canvas.setTextDatum(top_right);
    canvas.drawString(String(t.count) + " pts", 237, 73);
    canvas.setTextDatum(top_left);
  }

  // Altitude profile over the points, colored like the map
  const int x0 = PROFILE_X, w = PROFILE_W, bottom = 133, h = 40;
  canvas.drawFastHLine(x0, bottom, w, COL_BAR);
  for (int i = 0; i < w; i++) {
    const tracks::Point& p = t.pts[i * t.n / w];
    int hh = 1 + altFraction(t, p.alt) * (h - 1);
    canvas.drawFastVLine(x0 + i, bottom - hh, hh, pointColor(t, p));
  }
  if (v.cursor) canvas.drawFastVLine(x0 + cursorIndex(t, v.cursorTime) * w / t.n, bottom - h - 2, h + 2, TFT_WHITE);
}


// Pixels per degree latitude that fit the whole track, 0 for a single position.
static float mapFitScale(const tracks::Track& t, float& cosLat) {
  cosLat = cos((t.latMin + t.latMax) / 2 * DEG_TO_RAD);
  float dx = (t.lonMax - t.lonMin) * cosLat, dy = t.latMax - t.latMin;
  float scale = min(dx > 0 ? MAP_W / dx : INFINITY, dy > 0 ? MAP_H / dy : INFINITY);
  return isinf(scale) ? 0 : scale;
}

// Zoom and pan keys of a map page, false if the key is none of them.
// latest: the newest point of the current track, nullptr for a stored track.
static bool mapKey(MapView& v, const tracks::Track& t, const tracks::Point* latest, char c) {
  float cosLat, scale = mapFitScale(t, cosLat);
  if (c == '=') {
    if (v.zoom == 1) {  // start at the middle, or at the latest point when following
      v.lat = (t.latMin + t.latMax) / 2;
      v.lon = (t.lonMin + t.lonMax) / 2;
    }
    if (v.zoom < MAP_MAX_ZOOM && scale > 0) v.zoom *= 2;
    return true;
  }
  if (c == '-') {
    if (v.zoom > 1) v.zoom /= 2;
    if (v.zoom == 1) v.follow = latest;
    return true;
  }
  if (v.zoom == 1) return false;
  if (c == '`') {
    v.zoom = 1;
    v.follow = latest;
    return true;
  }
  if (c == 'f' && latest) {
    v.follow = true;
    return true;
  }
  int dx = c == ',' ? -1 : c == '/' ? 1 : 0, dy = c == ';' ? 1 : c == '.' ? -1 : 0;
  if (!dx && !dy) return false;
  if (v.follow && latest) {  // panning starts where the map is now
    v.lat = latest->lat;
    v.lon = latest->lon;
  }
  v.follow = false;
  float s = scale * v.zoom;  // a quarter of the map per key
  v.lon += dx * (MAP_W / 4.0f) / (s * cosLat);
  v.lat += dy * (MAP_H / 4.0f) / s;
  return true;
}

// Track points as lines, colored by altitude or speed; north is up, equal
// scale in both directions, the whole track fitted into the map at zoom 1.
static void drawTrackMap(const tracks::Track& t, const MapView& v, const tracks::Point* latest) {
  float cosLat, scale = mapFitScale(t, cosLat) * v.zoom;
  float cLat = (t.latMin + t.latMax) / 2, cLon = (t.lonMin + t.lonMax) / 2;
  if (v.zoom > 1 && v.follow && latest) {
    cLat = latest->lat;
    cLon = latest->lon;
  } else if (v.zoom > 1) {
    cLat = v.lat;
    cLon = v.lon;
  }
  auto px = [&](const tracks::Point& p) { return (int)(MAP_X + MAP_W / 2 + (p.lon - cLon) * cosLat * scale); };
  auto py = [&](const tracks::Point& p) { return (int)(MAP_Y + MAP_H / 2 - (p.lat - cLat) * scale); };
  canvas.setClipRect(MAP_X, MAP_Y, MAP_W, MAP_H);
  for (int i = 1; i < t.n; i++) {
    const tracks::Point &a = t.pts[i - 1], &b = t.pts[i];
    canvas.drawLine(px(a), py(a), px(b), py(b), pointColor(t, b));
  }
  const tracks::Point &first = t.pts[0], &last = latest ? *latest : t.pts[t.n - 1];
  canvas.fillCircle(px(first), py(first), 3, TFT_GREEN);
  canvas.drawCircle(px(last), py(last), 3, TFT_RED);
  if (v.cursor) {
    const tracks::Point& c = t.pts[cursorIndex(t, v.cursorTime)];
    canvas.drawCircle(px(c), py(c), 5, TFT_WHITE);
    canvas.fillCircle(px(c), py(c), 2, TFT_WHITE);
  }
  canvas.clearClipRect();

  canvas.setFont(&fonts::Font2);
  if (v.cursor) {  // the point at the cursor instead of the color legend, zoom only if zoomed
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(cursorText(t, v), 3, 119);
    if (v.zoom > 1) {
      canvas.setTextDatum(top_right);
      canvas.drawString("x" + String(v.zoom), 236, 119);
      canvas.setTextDatum(top_left);
    }
    return;
  }

  // Color legend: low - high
  String low = cfg.mapSpeed ? String("0") : String(t.altMin, 0);
  String high = cfg.mapSpeed ? String(t.spdMax, 0) + " km/h" : String(t.altMax, 0) + " m";
  canvas.setTextColor(rangeColor(0));
  canvas.drawString(low, 3, 119);
  int x = 3 + canvas.textWidth(low);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString(" - ", x, 119);
  x += canvas.textWidth(" - ");
  canvas.setTextColor(rangeColor(1));
  canvas.drawString(high, x, 119);
  if (scale > 0) {  // width of the map area, 1 degree latitude = 111.32 km
    float km = MAP_W / scale * 111.32f;
    String width = (km < 1 ? String(km * 1000, 0) + " m" : String(km, 1) + " km");
    if (v.zoom > 1) width = String(latest && !v.follow ? "F follow  " : "") + "x" + String(v.zoom) + "  " + width;
    else width = "width " + width;
    canvas.setTextColor(v.zoom > 1 ? TFT_WHITE : COL_LABEL);
    canvas.setTextDatum(top_right);
    canvas.drawString(width, 236, 119);
    canvas.setTextDatum(top_left);
  }
}

static void drawTrackInfoPage() {
  drawTitleBar(trackFiles[trackSel].name, "C color M map");
  drawTrackInfo(track, trackView);
}

static void drawTrackMapPage() {
  drawTitleBar(trackFiles[trackSel].name, trackView.zoom > 1 ? "` reset" : "=- zoom I info");
  drawTrackMap(track, trackView, nullptr);
}

// Reads the points logged since the last time: from the CSV if written (it has
// the speed), else from the GPX.
static void updateLiveTrack() {
  if (!rtc.logging || !rtc.baseName[0] || !sdOk) return;
  if (strcmp(liveBase, rtc.baseName) != 0) {  // new track
    memset(&liveTrack, 0, sizeof(liveTrack));
    liveState = {};
    strlcpy(liveBase, rtc.baseName, sizeof(liveBase));
    livePoints = 0;
    liveView = {1, true, 0, 0, false, 0};
  }
  if (rtc.points == livePoints || (livePoints && millis() - liveReadMs < LIVE_READ_MS)) return;
  bool gpx = cfg.logFormat == LOG_GPX;
  tracks::FileInfo f = {String(strrchr(rtc.baseName, '/') + 1) + (gpx ? ".gpx" : ".csv"),
                        (uint8_t)(gpx ? tracks::TYPE_GPX : tracks::TYPE_CSV), 0};
  tracks::loadMore(f, liveTrack, liveState);
  livePoints = rtc.points;
  liveReadMs = millis();
}

static void drawLivePage(bool map) {
  drawTitleBar("CURRENT TRACK", !map ? "C color . map" : liveView.zoom > 1 ? "` reset" : "=- zoom ; info");
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COL_LABEL);
  if (!rtc.logging) canvas.drawString("Not logging - Enter starts", 3, 22);
  else if (!liveTrack.count || strcmp(liveBase, rtc.baseName) != 0) canvas.drawString("No points yet", 3, 22);
  else if (map) drawTrackMap(liveTrack, liveView, &liveState.prev);
  else drawTrackInfo(liveTrack, liveView);
}

static void drawSubPageBar();

static void draw() {
  canvas.fillScreen(COL_BG);
  canvas.setTextDatum(top_left);
  if (page == PAGE_SETTINGS) {
    drawSettingsPage();
  } else if (page == PAGE_TRACKS) {
    drawTracksPage();
  } else if (page == PAGE_TRACK_INFO) {
    drawTrackInfoPage();
  } else if (page == PAGE_TRACK_MAP) {
    drawTrackMapPage();
  } else if (page == PAGE_LIVE_INFO || page == PAGE_LIVE_MAP) {
    drawLivePage(page == PAGE_LIVE_MAP);
  } else if (page == PAGE_HELP) {
    drawHelpPage();
  } else {
    drawStatusBar();
    if (page == PAGE_MAIN) drawMainPage();
    else if (page == PAGE_BIG) drawBigPage();
    else if (page == PAGE_DETAILS) drawDetailsPage();
    else if (page == PAGE_SATS) drawSatsPage();
    else if (page == PAGE_SIGNAL) drawSignalPage();
    else if (page == PAGE_GG) drawGgPage();
    else if (page == PAGE_ACCEL) drawAccelChartPage();
    else if (page == PAGE_WIFI) drawWifiPage();
    else if (page == PAGE_MENU) drawMenuPage();
  }
  drawSubPageBar();
  if (millis() < statusMsgUntil) drawMessageBox(statusMsg);
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

// Current GPS data as table rows for the web page.
static String webStatusHtml() {
  TinyGPSPlus& g = gps.gps();
  bool fix = gps.hasFix();
  String s;
  char buf[80];
  auto row = [&](const char* label, const char* value) {
    s += "<tr><td>";
    s += label;
    s += "</td><td>";
    s += value;
    s += "</td></tr>";
  };
  bool gpsAlive = gps.lastDataMs() && millis() - gps.lastDataMs() < 5000;
  row("Status", gpsStandbyMs ? "GPS standby (no motion)" : !gpsAlive ? "no GPS data" : fix ? "fix" : "no fix");
  if (fix) {
    double lat = g.location.lat(), lon = g.location.lng();
    snprintf(buf, sizeof(buf), "%.6f %c", fabs(lat), lat >= 0 ? 'N' : 'S');
    row("Latitude", buf);
    snprintf(buf, sizeof(buf), "%.6f %c", fabs(lon), lon >= 0 ? 'E' : 'W');
    row("Longitude", buf);
    if (g.altitude.isValid()) snprintf(buf, sizeof(buf), "%.0f m", g.altitude.meters());
    else strcpy(buf, "--");
    row("Altitude", buf);
    snprintf(buf, sizeof(buf), "%.1f km/h, course %.0f&deg;", g.speed.kmph(), g.course.deg());
    row("Speed", buf);
  }
  char acc[12] = "";
  if (fix && g.hdop.isValid()) snprintf(acc, sizeof(acc), " (~%.0f m)", g.hdop.hdop() * HDOP_UERE_M);
  snprintf(buf, sizeof(buf), "%lu used / %d in view, HDOP %.1f%s", (unsigned long)g.satellites.value(),
           gps.satsInView(), g.hdop.hdop(), acc);
  row("Satellites", buf);
  struct tm t;
  int16_t tz;
  if (localTime(t, tz)) {
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d UTC%s", t.tm_year + 1900, t.tm_mon + 1,
             t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, tzText(tz).c_str());
    row("Time", buf);
  }
  const char* name = strrchr(rtc.baseName, '/');
  if (rtc.logging) snprintf(buf, sizeof(buf), "recording, %lu points %s", (unsigned long)rtc.points, name ? name + 1 : "");
  else strcpy(buf, "off");
  row("Logging", buf);
  if (motion::calibrated()) {
    snprintf(buf, sizeof(buf), "lon %+.2f, lat %+.2f, vert %+.2f g%s", motion::lonG(), motion::latG(),
             motion::vertG(), motion::forwardLearned() ? "" : " (learning)");
    row("Acceleration", buf);
    const motion::Maxima& m = motion::maxima();
    snprintf(buf, sizeof(buf), "accel %.2f, brake %.2f, left %.2f, right %.2f, up %.2f, down %.2f g", m.accel,
             m.brake, m.left, m.right, m.up, m.down);
    row("Max accel.", buf);
  } else if (motion::available()) {
    row("Acceleration", "not calibrated");
  }
  int mv = batteryMvUi();
  snprintf(buf, sizeof(buf), "%.2f V, %d %%", mv / 1000.0f, batteryPctFromMv(mv));
  row("Battery", buf);
  return s;
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
  keepScreenOn = false;
  if (canDeepSleep() && lastLogMs) {
    uint32_t ms = remainingIntervalMs(millis() - lastLogMs);
    draw();
    enterDeepSleep(max(ms, MIN_SLEEP_MS), true);
  }
  setScreen(false);
}

static bool groupAvailable(int g) {
  if (g == G_WIFI) return cfg.nmeaServer;
  if (g == G_ACCEL) return motion::available();
  return true;
}

// Group and sub page of the current page, false for pages outside the groups.
static bool findPage(int& g, int& sub) {
  for (g = 0; g < G_COUNT; g++)
    for (sub = 0; sub < GROUPS[g].count; sub++)
      if (GROUPS[g].pages[sub] == page) return true;
  return false;
}

static void showGroup(int g) {
  page = GROUPS[g].pages[groupSub[g]];
}

// Main page left / right, with the sub page shown there last.
// Sub page position as a bar at the right edge, one segment per sub page,
// the current one bright: shows there is more with up / down.
static void drawSubPageBar() {
  int g, sub;
  if (!findPage(g, sub) || GROUPS[g].count < 2) return;
  const int x = 237, w = 3, top = 20, bottom = 134;
  int segH = (bottom - top) / GROUPS[g].count;
  for (int i = 0; i < GROUPS[g].count; i++) {
    canvas.fillRect(x, top + i * segH + 1, w, segH - 2, i == sub ? TFT_WHITE : COL_BAR);
  }
}

static void nextGroup(int dir) {
  int g, sub;
  if (!findPage(g, sub)) g = G_POS;
  do {
    g = (g + G_COUNT + dir) % G_COUNT;
  } while (!groupAvailable(g));
  showGroup(g);
}

static void nextSubPage(int dir) {
  int g, sub;
  if (!findPage(g, sub) || GROUPS[g].count < 2) return;
  groupSub[g] = (sub + GROUPS[g].count + dir) % GROUPS[g].count;
  showGroup(g);
}

// G0: all pages in turn, sub pages first.
static void nextPageFlat() {
  int g, sub;
  if (!findPage(g, sub)) g = sub = 0;
  if (sub + 1 < GROUPS[g].count) {
    groupSub[g] = sub + 1;
    showGroup(g);
    return;
  }
  do {
    g = (g + 1) % G_COUNT;
  } while (!groupAvailable(g));
  groupSub[g] = 0;
  showGroup(g);
}

static void saveImuCalibration() {
  motion::getCalibration(cfg.imuUp, cfg.imuFwd);
  cfg.imuFwdLearned = motion::forwardLearned();
  if (sdOk) configSave(cfg);
}

static void openHelp() {
  helpFirst = 0;
  returnPage = page;
  page = PAGE_HELP;
}

static void openTracks() {
  if (!sdOk) { showStatus("No SD card!"); return; }
  trackFiles = tracks::list();
  trackSel = 0;
  trackDeleteAsk = false;
  trackAction = -1;
  returnPage = page;
  page = PAGE_TRACKS;
}

// Info or map page of the selected file; the file is read only when coming from the list.
static void openTrack(Page p) {
  if (trackFiles.empty()) return;
  const tracks::FileInfo& f = trackFiles[trackSel];
  if (f.type == tracks::TYPE_NMEA) { showStatus("No info / map for NMEA files"); return; }
  if (page == PAGE_TRACKS) {
    showStatus("Reading track...", 60000);
    draw();
    bool ok = tracks::load(f, track);
    trackView = {1, false, 0, 0, false, 0};
    statusMsgUntil = 0;
    if (!ok) { showStatus("No track points found"); return; }
  }
  page = p;
}

static void askDeleteTrack() {
  if (trackFiles.empty()) return;
  String base = tracks::path(trackFiles[trackSel]);
  base.remove(base.lastIndexOf('.'));
  if (rtc.logging && base == rtc.baseName) showStatus("Track is being logged");
  else trackDeleteAsk = true;
}

static void deleteTrack() {
  const tracks::FileInfo& f = trackFiles[trackSel];
  if (!tracks::remove(f)) { showStatus("SD error: not deleted"); return; }
  showStatus(("Deleted " + f.name).c_str());
  trackFiles.erase(trackFiles.begin() + trackSel);
  trackSel = min(trackSel, max(0, (int)trackFiles.size() - 1));
}

static void runTrackAction(int a) {
  if (a == A_INFO) openTrack(PAGE_TRACK_INFO);
  else if (a == A_MAP) openTrack(PAGE_TRACK_MAP);
  else askDeleteTrack();
}

static void menuSelect(int id) {
  switch (id) {
    case M_LOGGING: rtc.logging ? stopLogging() : startLogging(); break;
    case M_SERVER: toggleNmeaServer(); break;
    case M_SETTINGS: openSettings(); break;
    case M_TRACKS: openTracks(); break;
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
    if (c == ';') helpFirst = max(0, helpFirst - 1);
    else if (c == '.') helpFirst = min(HELP_COUNT - HELP_ROWS, helpFirst + 1);
    else page = returnPage;
    return;
  }
  if (page == PAGE_TRACKS) {
    int n = trackFiles.size();
    int shortcut = c == 'i' ? A_INFO : c == 'm' ? A_MAP : c == 'd' ? A_DELETE : -1;
    if (trackDeleteAsk) {
      trackDeleteAsk = false;
      if (c == 'y') deleteTrack();
    } else if (trackAction >= 0) {
      if (c == ';') trackAction = (trackAction + A_COUNT - 1) % A_COUNT;
      else if (c == '.') trackAction = (trackAction + 1) % A_COUNT;
      else if (enter || shortcut >= 0 || c == '`' || del) {
        int a = enter ? trackAction : shortcut;
        trackAction = -1;
        if (a >= 0) runTrackAction(a);
      }
    } else if (c == ';' && n) trackSel = (trackSel + n - 1) % n;
    else if (c == '.' && n) trackSel = (trackSel + 1) % n;
    else if (enter && n) trackAction = A_INFO;
    else if (shortcut >= 0) runTrackAction(shortcut);
    else if (c == '`' || del) page = returnPage;
    return;
  }
  if ((page == PAGE_TRACK_INFO || page == PAGE_TRACK_MAP) && cursorKey(trackView, track, c)) return;
  if (page == PAGE_TRACK_MAP && mapKey(trackView, track, nullptr, c)) return;
  if (page == PAGE_TRACK_INFO || page == PAGE_TRACK_MAP) {
    if (c == 'c') toggleMapColor();
    else if (c == 'i') page = PAGE_TRACK_INFO;
    else if (c == 'm') page = PAGE_TRACK_MAP;
    else if (c == ';' || c == '.') page = page == PAGE_TRACK_INFO ? PAGE_TRACK_MAP : PAGE_TRACK_INFO;
    else if (c == '`' || del || enter) page = PAGE_TRACKS;
    return;
  }
  if (page == PAGE_MENU) {
    if (c == ';') { menuSel = (menuSel + M_COUNT - 1) % M_COUNT; return; }
    if (c == '.') { menuSel = (menuSel + 1) % M_COUNT; return; }
    if (enter) { menuSelect(menuSel); return; }
  }
  if (page == PAGE_GG || page == PAGE_ACCEL) {
    if (c == 'c') {
      if (motion::calibrate()) {
        saveImuCalibration();
        showStatus("Calibrated, now drive");
      } else {
        showStatus("Not still - try again");
      }
      return;
    }
    if (c == 'x') { motion::resetMax(); showStatus("Max values reset"); return; }
  }
  bool livePage = page == PAGE_LIVE_INFO || page == PAGE_LIVE_MAP;
  if (livePage && liveTrack.count && cursorKey(liveView, liveTrack, c)) return;
  if (page == PAGE_LIVE_MAP && liveTrack.count && mapKey(liveView, liveTrack, &liveState.prev, c)) return;
  if (page == PAGE_LIVE_INFO || page == PAGE_LIVE_MAP) {
    if (c == 'c') { toggleMapColor(); return; }
    if (c == 'i' || c == 'm') {
      groupSub[G_TRACK] = c == 'm';
      showGroup(G_TRACK);
      return;
    }
  }
  if (c >= '1' && c < '1' + G_COUNT) {  // main pages
    int g = c - '1';
    if (g == G_WIFI && !cfg.nmeaServer) showStatus("NMEA server is off");
    else if (!groupAvailable(g)) showStatus("No accelerometer");
    else showGroup(g);
    return;
  }
  if (enter || c == 'r') rtc.logging ? stopLogging() : startLogging();
  else if (tab || c == '/' || c == ' ') nextGroup(+1);
  else if (c == ',') nextGroup(-1);
  else if (c == ';') nextSubPage(-1);
  else if (c == '.') nextSubPage(+1);
  else if (c == 's') openSettings();
  else if (c == 't') openTracks();
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
      auto& ks = M5Cardputer.Keyboard.keysState();
      if (!screenOn && !ks.word.empty() && tolower(ks.word[0]) == 'z') {
        keepScreenOn = true;
        showStatus("Screen stays on until Z");
      } else if (screenOn) {
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
      else if (page == PAGE_TRACKS && trackDeleteAsk) trackDeleteAsk = false;
      else if (page == PAGE_TRACKS && trackAction >= 0) trackAction = -1;
      else if (page == PAGE_TRACKS) page = returnPage;
      else if (page == PAGE_TRACK_INFO || page == PAGE_TRACK_MAP) page = PAGE_TRACKS;
      else nextPageFlat();
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
  if (motion::begin()) motion::setCalibration(cfg.imuUp, cfg.imuFwd, cfg.imuFwdLearned);
  startGps();
  web::setStatusHtml(webStatusHtml);
  web::setConfigHandlers(webConfigText, webApplyConfig);
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

// Each new fix's speed to the accelerometer, which learns forward from it.
static void feedMotion() {
  static uint32_t lastFixTime = 0;
  TinyGPSPlus& g = gps.gps();
  if (!gps.hasFix() || !g.time.isValid() || !g.speed.isValid() || g.time.value() == lastFixTime) return;
  lastFixTime = g.time.value();
  if (motion::gpsSpeed(g.speed.kmph(), millis())) {
    saveImuCalibration();
    showStatus("Forward direction learned");
  }
}

// Red pulse while logging, brightness squared for an even looking fade.
// Not in deep sleep: the quiet wake-ups don't use the LED.
static void updateRecLed() {
  static uint8_t shown = 0;
  uint8_t level = 0;
  if (rtc.logging && cfg.recLed) {
    uint32_t t = (millis() - ledStartMs) % LED_PERIOD_MS;
    if (t < 2 * LED_FADE_MS) {
      float f = (t < LED_FADE_MS ? t : 2 * LED_FADE_MS - t) / (float)LED_FADE_MS;
      level = f * f * LED_MAX;
    }
  }
  if (level == shown) return;
  shown = level;
  M5.Led.setAllColor(level, 0, 0);
}

void loop() {
  M5.update();
  motion::update();
  gps.update();
  if (gps.baudLocked() && cfg.gpsBaud == 0) rtc.detectedBaud = gps.baud();
  feedMotion();
  updateGpsStandby();
  updateRecLed();
  handleInput();
  nmeaserver::update();
  applyWebConfig();
  int devices = nmeaserver::deviceCount();
  if (devices != lastDeviceCount) {
    showStatus(devices > lastDeviceCount ? ("Device connected (" + String(devices) + ")").c_str()
                                         : ("Device disconnected (" + String(devices) + ")").c_str());
    lastDeviceCount = devices;
  }

  uint32_t now = millis();
  bool due = cfg.logMode != LOG_MODE_TIME || lastLogMs == 0 || now - lastLogMs >= cfg.logIntervalSec * 1000UL;
  if (rtc.logging && sdOk && due) {
    LogResult result = logPoint(batteryPctFromMv(batteryMvUi()));
    if (result == LOG_OK) {
      lastLogMs = now;
      sdErrors = 0;
    } else if (result == LOG_PAUSED) {
      lastLogMs = now;
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
  if (cfg.screenTimeoutSec && !keepScreenOn && page != PAGE_SETTINGS &&
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
    if (page == PAGE_LIVE_INFO || page == PAGE_LIVE_MAP) updateLiveTrack();
    draw();
  }
  delay(5);
}
