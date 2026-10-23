#pragma once
#include <Arduino.h>
#include <time.h>
#include <vector>

// Track files on the SD card: listing, deleting and reading them back for
// the info and map pages. Only CSV and GPX files can be read, NMEA files
// are listed and deleted only.
namespace tracks {

enum Type : uint8_t { TYPE_CSV, TYPE_GPX, TYPE_NMEA };
extern const char* const TYPE_NAMES[3];

struct FileInfo {
  String name;  // with extension, without directory
  uint8_t type;
  uint32_t size;
};

// Long tracks are thinned out evenly to this many points.
static const int MAX_POINTS = 1024;

struct Point {
  float lat, lon, alt;
};

struct Track {
  uint32_t count;      // all points in the file
  time_t start, end;   // UTC, 0 = no time
  double distM;
  float altMin, altMax;
  float latMin, latMax, lonMin, lonMax;
  int n;               // points in pts
  Point pts[MAX_POINTS];
};

std::vector<FileInfo> list();  // newest first
String sizeText(uint32_t bytes);  // "512 B", "12.3 KB", "1.2 MB"
String path(const FileInfo& f);
bool remove(const FileInfo& f);
bool load(const FileInfo& f, Track& t);  // false if not readable or no points

}  // namespace tracks
