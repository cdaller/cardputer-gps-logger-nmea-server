#pragma once
#include <Arduino.h>

struct LogPoint {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  double lat, lon;
  float altM, speedKmh, courseDeg, hdop;
  uint8_t sats;
  int8_t batteryPct;
};

#define LOG_DIR "/gpslogger/tracks"

// Writes track points to CSV and/or GPX files on the SD card.
// The base file name (without extension) is created from the time of the
// first point and kept by the caller across deep sleep.
namespace logger {
bool sdBegin();
// baseName: in/out, empty => a new file is created from the point's time.
bool write(const LogPoint& p, uint8_t format, char* baseName, size_t baseNameLen);
}  // namespace logger
