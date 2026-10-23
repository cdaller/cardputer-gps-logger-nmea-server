#include "tracks.h"
#include "logger.h"
#include <SD.h>
#include <algorithm>

namespace tracks {

const char* const TYPE_NAMES[3] = {"CSV", "GPX", "NMEA"};

static int typeOf(const String& name) {
  if (name.endsWith(".csv")) return TYPE_CSV;
  if (name.endsWith(".gpx")) return TYPE_GPX;
  if (name.endsWith(".nmea")) return TYPE_NMEA;
  return -1;
}

std::vector<FileInfo> list() {
  std::vector<FileInfo> files;
  File dir = SD.open(LOG_DIR);
  if (!dir || !dir.isDirectory()) return files;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    String name = f.name();
    int type = typeOf(name);
    if (!f.isDirectory() && type >= 0) files.push_back({name, (uint8_t)type, (uint32_t)f.size()});
    f.close();
  }
  dir.close();
  // Names start with the date and time, so this sorts by age.
  std::sort(files.begin(), files.end(), [](const FileInfo& a, const FileInfo& b) { return a.name > b.name; });
  return files;
}

String path(const FileInfo& f) { return String(LOG_DIR "/") + f.name; }

bool remove(const FileInfo& f) { return SD.remove(path(f)); }

static time_t parseTime(const char* s) {
  struct tm t = {};
  if (sscanf(s, "%d-%d-%dT%d:%d:%d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min,
             &t.tm_sec) != 6)
    return 0;
  t.tm_year -= 1900;
  t.tm_mon -= 1;
  return mktime(&t);  // TZ is UTC on the ESP32 unless set
}

// One track point per line, as written by the logger.
static bool parseCsv(const char* line, Point& p, time_t& time) {
  double lat, lon;
  const char* comma = strchr(line, ',');
  if (!comma || sscanf(comma + 1, "%lf,%lf,%f", &lat, &lon, &p.alt) != 3) return false;  // header
  p.lat = lat;
  p.lon = lon;
  time = parseTime(line);
  return true;
}

static bool parseGpx(const char* line, Point& p, time_t& time) {
  const char* pt = strstr(line, "<trkpt");
  if (!pt) return false;
  const char* lat = strstr(pt, "lat=\"");
  const char* lon = strstr(pt, "lon=\"");
  if (!lat || !lon) return false;
  p.lat = atof(lat + 5);
  p.lon = atof(lon + 5);
  const char* ele = strstr(pt, "<ele>");
  p.alt = ele ? atof(ele + 5) : 0;
  const char* t = strstr(pt, "<time>");
  time = t ? parseTime(t + 6) : 0;
  return true;
}

static double distanceM(const Point& a, const Point& b) {
  const double R = 6371000, D2R = M_PI / 180;
  double dLat = (b.lat - a.lat) * D2R, dLon = (b.lon - a.lon) * D2R;
  double h = sin(dLat / 2) * sin(dLat / 2) +
             cos(a.lat * D2R) * cos(b.lat * D2R) * sin(dLon / 2) * sin(dLon / 2);
  return 2 * R * asin(sqrt(min(h, 1.0)));
}

// Every point counts for the statistics, but only every stride-th point is
// kept: when the buffer is full, every second point is dropped and the
// stride doubles. The distance is summed up in load(), from all points.
static void addPoint(Track& t, const Point& p, time_t time, uint32_t& stride) {
  if (t.count == 0) {
    t.altMin = t.altMax = p.alt;
    t.latMin = t.latMax = p.lat;
    t.lonMin = t.lonMax = p.lon;
  }
  t.altMin = min(t.altMin, p.alt);
  t.altMax = max(t.altMax, p.alt);
  t.latMin = min(t.latMin, p.lat);
  t.latMax = max(t.latMax, p.lat);
  t.lonMin = min(t.lonMin, p.lon);
  t.lonMax = max(t.lonMax, p.lon);
  if (time && !t.start) t.start = time;
  if (time) t.end = time;
  if (t.count % stride == 0) {
    if (t.n == MAX_POINTS) {
      for (int i = 0; i < t.n / 2; i++) t.pts[i] = t.pts[2 * i];
      t.n /= 2;
      stride *= 2;
    }
    if (t.count % stride == 0) t.pts[t.n++] = p;
  }
  t.count++;
}

bool load(const FileInfo& f, Track& t) {
  if (f.type == TYPE_NMEA) return false;
  File file = SD.open(path(f));
  if (!file) return false;
  memset(&t, 0, sizeof(t));
  uint32_t stride = 1;
  Point prev;
  static uint8_t buf[512];
  char line[256];
  size_t len = 0;
  auto parseLine = [&] {
    line[len] = '\0';
    len = 0;
    Point p;
    time_t time;
    if (!(f.type == TYPE_CSV ? parseCsv(line, p, time) : parseGpx(line, p, time))) return;
    if (t.count) t.distM += distanceM(prev, p);
    prev = p;
    addPoint(t, p, time, stride);
  };
  for (int n; (n = file.read(buf, sizeof(buf))) > 0;) {
    for (int i = 0; i < n; i++) {
      if (buf[i] == '\n') parseLine();
      else if (len < sizeof(line) - 1) line[len++] = buf[i];  // longer lines are cut off
    }
  }
  if (len) parseLine();
  file.close();
  return t.count > 0;
}

}  // namespace tracks
