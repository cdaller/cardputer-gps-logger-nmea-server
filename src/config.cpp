#include "config.h"
#include <SD.h>

const GpsPreset GPS_PRESETS[GPS_MODULE_COUNT] = {
  {"cap_lora1262", "CAP LoRa-1262", 15, 13, 115200},
  {"unit_gps_v11", "Unit GPS v1.1", 1, 2, 115200},
  {"unit_gps_at6558", "Unit GPS AT6558", 1, 2, 9600},
  {"grove_nmea", "Grove NMEA", 1, 2, 0},
  {"custom", "Custom", 1, 2, 0},
};

const char* const LOG_FORMAT_NAMES[LOG_FORMAT_COUNT] = {"CSV", "GPX", "CSV+GPX"};

void Config::applyPreset(uint8_t module) {
  gpsModule = module;
  if (module == GPS_CUSTOM) return;
  gpsRx = GPS_PRESETS[module].rx;
  gpsTx = GPS_PRESETS[module].tx;
  gpsBaud = GPS_PRESETS[module].baud;
}

void configDefaults(Config& cfg, bool isAdv) {
  cfg = Config();
  cfg.applyPreset(isAdv ? GPS_CAP_LORA1262 : GPS_UNIT_V11);
}

bool configLoad(Config& cfg) {
  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    key.trim();
    val.trim();
    long n = val.toInt();
    if (key == "gps_module") {
      for (uint8_t i = 0; i < GPS_MODULE_COUNT; i++)
        if (val == GPS_PRESETS[i].key) cfg.gpsModule = i;
    } else if (key == "gps_rx") cfg.gpsRx = n;
    else if (key == "gps_tx") cfg.gpsTx = n;
    else if (key == "gps_baud") cfg.gpsBaud = (val == "auto") ? 0 : n;
    else if (key == "log_interval") cfg.logIntervalSec = max(1L, n);
    else if (key == "log_format") {
      for (uint8_t i = 0; i < LOG_FORMAT_COUNT; i++)
        if (val.equalsIgnoreCase(LOG_FORMAT_NAMES[i])) cfg.logFormat = i;
    } else if (key == "nmea_log") cfg.logNmea = n != 0;
    else if (key == "deep_sleep") cfg.deepSleep = n != 0;
    else if (key == "screen_timeout") cfg.screenTimeoutSec = n;
    else if (key == "fix_timeout") cfg.fixTimeoutSec = max(5L, n);
    else if (key == "tz_offset") cfg.tzOffsetHours = constrain(n, -12, 14);
    else if (key == "auto_start") cfg.autoStart = n != 0;
    else if (key == "brightness") cfg.brightness = constrain(n, 8, 255);
  }
  f.close();
  return true;
}

bool configSave(const Config& cfg) {
  SD.mkdir("/gpslogger");
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) return false;
  f.println("# GPS Logger configuration (edit on PC or in the app)");
  f.println("# gps_module: cap_lora1262 | unit_gps_v11 | unit_gps_at6558 | grove_nmea | custom");
  f.printf("gps_module=%s\n", GPS_PRESETS[cfg.gpsModule].key);
  f.printf("gps_rx=%d\n", cfg.gpsRx);
  f.printf("gps_tx=%d\n", cfg.gpsTx);
  if (cfg.gpsBaud == 0) f.println("gps_baud=auto");
  else f.printf("gps_baud=%lu\n", (unsigned long)cfg.gpsBaud);
  f.println("# seconds between two logged points");
  f.printf("log_interval=%lu\n", (unsigned long)cfg.logIntervalSec);
  f.println("# CSV | GPX | CSV+GPX");
  f.printf("log_format=%s\n", LOG_FORMAT_NAMES[cfg.logFormat]);
  f.println("# 1 = also write all NMEA sentences from the GPS to a .nmea file");
  f.printf("nmea_log=%d\n", cfg.logNmea ? 1 : 0);
  f.println("# 1 = deep sleep between points while logging (interval >= 15s)");
  f.printf("deep_sleep=%d\n", cfg.deepSleep ? 1 : 0);
  f.println("# seconds without key press until screen off / deep sleep, 0 = never");
  f.printf("screen_timeout=%u\n", cfg.screenTimeoutSec);
  f.println("# max. seconds to wait for a fix after waking from deep sleep");
  f.printf("fix_timeout=%u\n", cfg.fixTimeoutSec);
  f.println("# hours added to UTC for the displayed time (log files are always UTC)");
  f.printf("tz_offset=%d\n", cfg.tzOffsetHours);
  f.println("# 1 = start logging automatically after power on");
  f.printf("auto_start=%d\n", cfg.autoStart ? 1 : 0);
  f.printf("brightness=%u\n", cfg.brightness);
  f.close();
  return true;
}
