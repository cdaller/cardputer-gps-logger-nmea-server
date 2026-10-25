#pragma once
#include <Arduino.h>

// Known GPS modules. Selecting one in the settings copies its RX/TX/baud
// into the config; "Custom" leaves the pins as they are.
enum GpsModule : uint8_t {
  GPS_CAP_LORA1262 = 0,  // Cap LoRa-1262 on Cardputer ADV EXT header (ATGM336H)
  GPS_UNIT_V11,          // M5 Unit GPS v1.1 (AT6668) on Grove
  GPS_UNIT_AT6558,       // M5 Unit GPS (AT6558, older) on Grove
  GPS_GROVE_NMEA,        // Generic NMEA module (NEO-6M/M8N, ...) on Grove
  GPS_CUSTOM,
  GPS_MODULE_COUNT
};

struct GpsPreset {
  const char* key;    // used in config file
  const char* name;   // shown on screen
  int8_t rx;          // Cardputer RX <- GPS TX
  int8_t tx;          // Cardputer TX -> GPS RX
  uint32_t baud;      // 0 = auto detect
};

extern const GpsPreset GPS_PRESETS[GPS_MODULE_COUNT];

enum LogFormat : uint8_t { LOG_CSV = 0, LOG_GPX, LOG_CSV_GPX, LOG_FORMAT_COUNT };
extern const char* const LOG_FORMAT_NAMES[LOG_FORMAT_COUNT];

// When a point is recorded: at turns and speed changes (smart), every
// log_interval seconds (time) or every log_distance meters (distance).
enum LogMode : uint8_t { LOG_MODE_SMART = 0, LOG_MODE_TIME, LOG_MODE_DISTANCE, LOG_MODE_COUNT };
extern const char* const LOG_MODE_NAMES[LOG_MODE_COUNT];

// Daylight saving time rule applied on top of tz_offset.
enum DstRule : uint8_t { DST_OFF = 0, DST_EU, DST_US, DST_COUNT };
extern const char* const DST_NAMES[DST_COUNT];

static const int WIFI_NETWORKS = 3;  // known networks for client mode

struct Config {
  uint8_t gpsModule = GPS_UNIT_V11;
  int8_t gpsRx = 1;
  int8_t gpsTx = 2;
  uint32_t gpsBaud = 115200;      // 0 = auto
  uint8_t logMode = LOG_MODE_SMART;
  uint32_t logIntervalSec = 5;    // time mode
  uint16_t logDistanceM = 10;     // distance mode
  bool autoPause = true;          // no points while standing still
  uint8_t logFormat = LOG_CSV_GPX;
  bool logNmea = false;            // raw NMEA sentences to .nmea
  bool deepSleep = false;
  uint16_t screenTimeoutSec = 60; // 0 = never
  uint16_t fixTimeoutSec = 60;    // max. time to wait for a fix after waking up
  int16_t tzOffsetMin = 0;        // minutes added to UTC for the display
  uint8_t dstRule = DST_EU;       // +1 h in summer
  bool autoStart = false;
  uint8_t brightness = 128;
  bool recLed = true;             // RGB LED pulses red once a minute while logging
  bool mapSpeed = false;          // track map / profile colored by speed instead of altitude
  bool nmeaServer = false;        // serve NMEA over WiFi (TCP + UDP)
  bool wifiClient = false;        // false: own access point, true: join the strongest wifiSsid
  String wifiSsid[WIFI_NETWORKS]; // empty = slot unused
  String wifiPass[WIFI_NETWORKS];
  String apSsid = "GPS-NMEA-Logger";
  String apPass = "gpslogger";    // empty = open network, else min. 8 characters
  uint16_t nmeaPort = 2000;
  bool gpsStandby = false;        // GPS module to standby while the IMU sees no motion
  float imuUp[3] = {0, 0, 0};     // accelerometer calibration, all 0 = not calibrated
  float imuFwd[3] = {0, 0, 0};
  bool imuFwdLearned = false;

  void applyPreset(uint8_t module);
};

// Config lives in a plain text file on the SD card, so it survives
// flashing other firmware via the launcher and can be edited on a PC.
#define CONFIG_PATH "/gpslogger/config.txt"

void configDefaults(Config& cfg, bool isAdv);
bool configLoad(Config& cfg);        // false if no file (cfg unchanged)
bool configSave(const Config& cfg);
void configParse(Config& cfg, Stream& in);  // key=value lines on top of cfg
String configText(const Config& cfg);       // the config file content, with comments
String tzText(int16_t minutes);      // "+1", "+5:30", "-3:30"
int16_t tzOffsetAt(const Config& cfg, time_t utc);  // tz_offset plus daylight saving at that time
