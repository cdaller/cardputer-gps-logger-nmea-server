#include "config.h"
#include <SD.h>
#include <StreamString.h>

const GpsPreset GPS_PRESETS[GPS_MODULE_COUNT] = {
  {"cap_lora1262", "CAP LoRa-1262", 15, 13, 115200},
  {"unit_gps_v11", "Unit GPS v1.1", 1, 2, 115200},
  {"unit_gps_at6558", "Unit GPS AT6558", 1, 2, 9600},
  {"grove_nmea", "Grove NMEA", 1, 2, 0},
  {"custom", "Custom", 1, 2, 0},
};

const char* const LOG_FORMAT_NAMES[LOG_FORMAT_COUNT] = {"CSV", "GPX", "CSV+GPX"};
const char* const LOG_MODE_NAMES[LOG_MODE_COUNT] = {"Smart", "Time", "Distance"};

// Written first and renamed to CONFIG_PATH, so a power loss while saving
// doesn't leave a truncated config.
static const char CONFIG_TMP_PATH[] = CONFIG_PATH ".tmp";

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

String tzText(int16_t minutes) {
  char buf[8];
  int m = abs(minutes);
  if (m % 60) snprintf(buf, sizeof(buf), "%c%d:%02d", minutes < 0 ? '-' : '+', m / 60, m % 60);
  else snprintf(buf, sizeof(buf), "%c%d", minutes < 0 ? '-' : '+', m / 60);
  return buf;
}

// "2", "-3.5" (hours) or "+5:45", "-3:30" (hours:minutes)
static long parseTzMinutes(const String& val) {
  int colon = val.indexOf(':');
  if (colon < 0) return lround(val.toFloat() * 60);
  long m = labs(val.substring(0, colon).toInt()) * 60 + val.substring(colon + 1).toInt();
  return val.startsWith("-") ? -m : m;
}

bool configLoad(Config& cfg) {
  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) f = SD.open(CONFIG_TMP_PATH, FILE_READ);  // power lost between remove and rename
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
    } else if (key == "gps_rx") cfg.gpsRx = constrain(n, -1, 48);
    else if (key == "gps_tx") cfg.gpsTx = constrain(n, -1, 48);
    else if (key == "gps_baud") cfg.gpsBaud = (val == "auto") ? 0 : n;
    else if (key == "log_mode") {
      for (uint8_t i = 0; i < LOG_MODE_COUNT; i++)
        if (val.equalsIgnoreCase(LOG_MODE_NAMES[i])) cfg.logMode = i;
    } else if (key == "log_interval") cfg.logIntervalSec = constrain(n, 1, 86400);
    else if (key == "log_distance") cfg.logDistanceM = constrain(n, 1, 10000);
    else if (key == "auto_pause") cfg.autoPause = n != 0;
    else if (key == "log_format") {
      for (uint8_t i = 0; i < LOG_FORMAT_COUNT; i++)
        if (val.equalsIgnoreCase(LOG_FORMAT_NAMES[i])) cfg.logFormat = i;
    } else if (key == "nmea_log") cfg.logNmea = n != 0;
    else if (key == "deep_sleep") cfg.deepSleep = n != 0;
    else if (key == "screen_timeout") cfg.screenTimeoutSec = constrain(n, 0, 65535);
    else if (key == "fix_timeout") cfg.fixTimeoutSec = constrain(n, 5, 3600);
    else if (key == "tz_offset") cfg.tzOffsetMin = constrain(parseTzMinutes(val), -12 * 60, 14 * 60);
    else if (key == "auto_start") cfg.autoStart = n != 0;
    else if (key == "brightness") cfg.brightness = constrain(n, 8, 255);
    else if (key == "rec_led") cfg.recLed = n != 0;
    else if (key == "map_color") cfg.mapSpeed = val.equalsIgnoreCase("speed");
    else if (key == "nmea_server") cfg.nmeaServer = n != 0;
    else if (key == "wifi_mode") cfg.wifiClient = val.equalsIgnoreCase("client");
    else if (key == "wifi_ssid") cfg.wifiSsid = val;
    else if (key == "wifi_pass") cfg.wifiPass = val;
    else if (key == "ap_ssid" && val.length()) cfg.apSsid = val;
    else if (key == "ap_pass") cfg.apPass = val;
    else if (key == "nmea_port" && n >= 1 && n <= 65535) cfg.nmeaPort = n;
    else if (key == "gps_standby") cfg.gpsStandby = n != 0;
    else if (key == "imu_up") sscanf(val.c_str(), "%f,%f,%f", &cfg.imuUp[0], &cfg.imuUp[1], &cfg.imuUp[2]);
    else if (key == "imu_fwd") sscanf(val.c_str(), "%f,%f,%f", &cfg.imuFwd[0], &cfg.imuFwd[1], &cfg.imuFwd[2]);
    else if (key == "imu_fwd_learned") cfg.imuFwdLearned = n != 0;
  }
  f.close();
  return true;
}

bool configSave(const Config& cfg) {
  StreamString f;
  f.println("# GPS Logger configuration (edit on PC or in the app)");
  f.println("# gps_module: cap_lora1262 | unit_gps_v11 | unit_gps_at6558 | grove_nmea | custom");
  f.printf("gps_module=%s\n", GPS_PRESETS[cfg.gpsModule].key);
  f.printf("gps_rx=%d\n", cfg.gpsRx);
  f.printf("gps_tx=%d\n", cfg.gpsTx);
  if (cfg.gpsBaud == 0) f.println("gps_baud=auto");
  else f.printf("gps_baud=%lu\n", (unsigned long)cfg.gpsBaud);
  f.println("# smart = at turns and speed changes, time = every log_interval, distance = every log_distance");
  f.printf("log_mode=%s\n", LOG_MODE_NAMES[cfg.logMode]);
  f.println("# seconds between two logged points (time mode)");
  f.printf("log_interval=%lu\n", (unsigned long)cfg.logIntervalSec);
  f.println("# meters between two logged points (distance mode)");
  f.printf("log_distance=%u\n", cfg.logDistanceM);
  f.println("# 1 = no points while standing still");
  f.printf("auto_pause=%d\n", cfg.autoPause ? 1 : 0);
  f.println("# CSV | GPX | CSV+GPX");
  f.printf("log_format=%s\n", LOG_FORMAT_NAMES[cfg.logFormat]);
  f.println("# 1 = also write all NMEA sentences from the GPS to a .nmea file");
  f.printf("nmea_log=%d\n", cfg.logNmea ? 1 : 0);
  f.println("# 1 = deep sleep between points while logging (time mode, interval >= 15s)");
  f.printf("deep_sleep=%d\n", cfg.deepSleep ? 1 : 0);
  f.println("# seconds without key press until screen off / deep sleep, 0 = never");
  f.printf("screen_timeout=%u\n", cfg.screenTimeoutSec);
  f.println("# max. seconds to wait for a fix after waking from deep sleep");
  f.printf("fix_timeout=%u\n", cfg.fixTimeoutSec);
  f.println("# added to UTC for the displayed time, hours or hours:minutes (log files are always UTC)");
  f.printf("tz_offset=%s\n", tzText(cfg.tzOffsetMin).c_str());
  f.println("# 1 = start logging automatically after power on");
  f.printf("auto_start=%d\n", cfg.autoStart ? 1 : 0);
  f.printf("brightness=%u\n", cfg.brightness);
  f.println("# 1 = RGB LED fades in and out red once a minute while logging, 0 = off (not in deep sleep)");
  f.printf("rec_led=%d\n", cfg.recLed ? 1 : 0);
  f.println("# altitude | speed: color of the track map and the altitude profile (C on the track pages)");
  f.printf("map_color=%s\n", cfg.mapSpeed ? "speed" : "altitude");
  f.println("# 1 = send the GPS NMEA sentences over WiFi (TCP server + UDP broadcast)");
  f.printf("nmea_server=%d\n", cfg.nmeaServer ? 1 : 0);
  f.println("# ap = own WiFi network (ap_ssid/ap_pass), client = join wifi_ssid/wifi_pass");
  f.printf("wifi_mode=%s\n", cfg.wifiClient ? "client" : "ap");
  f.printf("wifi_ssid=%s\n", cfg.wifiSsid.c_str());
  f.printf("wifi_pass=%s\n", cfg.wifiPass.c_str());
  f.println("# ap_pass: empty = open network, otherwise at least 8 characters");
  f.printf("ap_ssid=%s\n", cfg.apSsid.c_str());
  f.printf("ap_pass=%s\n", cfg.apPass.c_str());
  f.printf("nmea_port=%u\n", cfg.nmeaPort);
  f.println("# 1 = GPS module to standby while the accelerometer sees no motion (Cardputer ADV)");
  f.printf("gps_standby=%d\n", cfg.gpsStandby ? 1 : 0);
  f.println("# accelerometer calibration, set on the acceleration page");
  f.printf("imu_up=%.4f,%.4f,%.4f\n", cfg.imuUp[0], cfg.imuUp[1], cfg.imuUp[2]);
  f.printf("imu_fwd=%.4f,%.4f,%.4f\n", cfg.imuFwd[0], cfg.imuFwd[1], cfg.imuFwd[2]);
  f.printf("imu_fwd_learned=%d\n", cfg.imuFwdLearned ? 1 : 0);

  SD.mkdir("/gpslogger");
  File out = SD.open(CONFIG_TMP_PATH, FILE_WRITE);
  if (!out) return false;
  bool ok = out.write((const uint8_t*)f.c_str(), f.length()) == f.length();
  out.close();
  if (!ok) return false;
  SD.remove(CONFIG_PATH);  // FAT rename doesn't replace an existing file
  return SD.rename(CONFIG_TMP_PATH, CONFIG_PATH);
}
