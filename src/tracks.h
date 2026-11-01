#pragma once
#include <Arduino.h>
#include <time.h>
#include <vector>

// Track files on the SD card: listing folders, deleting and reading them
// back for the info and map pages. Only CSV and GPX files can be read, NMEA
// files are listed and deleted only.
namespace tracks {

enum Type : uint8_t { TYPE_CSV, TYPE_GPX, TYPE_NMEA, TYPE_DIR };
extern const char* const TYPE_NAMES[4];

struct FileInfo {
  String name;  // with extension, without directory; ".." for the parent folder
  uint8_t type;
  uint32_t size;
  String dir;   // folder it is in, "/" = root
};

struct Bounds {
  float latMin, latMax, lonMin, lonMax;
};

// Long tracks are thinned out evenly to this many points.
static const int MAX_POINTS = 1024;

struct Point {
  float lat, lon, alt;
  float speed;  // km/h: from the CSV, computed from distance and time for GPX
  time_t time;  // UTC, 0 = no time
};

struct Track {
  uint32_t count;      // all points in the file
  time_t start, end;   // UTC, 0 = no time
  double distM;
  float altMin, altMax;
  float spdMax;
  float latMin, latMax, lonMin, lonMax;
  int n;               // points in pts
  Point pts[MAX_POINTS];
};

// Sub folders (".." first, except in the root) and track files of dir, by
// name; files newest first (track names start with date and time) or A-Z.
// gpxOnly: no CSV and NMEA files. A dir that can't be opened is replaced by
// its nearest parent that can.
std::vector<FileInfo> list(String& dir, bool gpxOnly, bool newestFirst);
String parentDir(const String& dir);  // "/" for the root
String sizeText(uint32_t bytes);  // "512 B", "12.3 KB", "1.2 MB"
String path(const FileInfo& f);
String path(const String& dir, const String& name);
bool remove(const FileInfo& f);
bool load(const FileInfo& f, Track& t);  // false if not readable or no points

// Where reading stopped, so a file that is being logged is read only from
// there on. Zeroed together with the track to start from the beginning.
struct LoadState {
  uint32_t pos;     // after the last point line
  uint32_t stride;
  Point prev;
  time_t prevTime;
};
bool loadMore(const FileInfo& f, Track& t, LoadState& st);  // false if not readable or no points

}  // namespace tracks
