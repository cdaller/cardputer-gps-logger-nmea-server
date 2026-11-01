#include "overlay.h"
#include <SD.h>
#include <algorithm>
#include "gpx_reader.h"

namespace overlay {

// Free heap left for the rest (WiFi, web server, file lists) after loading.
static const size_t HEAP_RESERVE = 64 * 1024;
// Lines are cut into chunks of this many points, each with its bounds, so
// only the chunks in view are drawn.
static const uint32_t CHUNK_POINTS = 64;
// Labels only with this few waypoints in view. At most MAX_NAME_READS names
// are read from the SD card per frame, the other labels follow in the next ones.
static const int MAX_LABELS = 30;
static const int MAX_NAME_READS = 8;
static const int DENSE_WPTS = 300;  // more in view: single pixels instead of 3x3 dots
static const int NAME_LEN = 20;
static const int NAME_CACHE = 48;

static const uint16_t COLORS[MAX_FILES] = {TFT_MAGENTA, TFT_ORANGE,      TFT_WHITE, TFT_PINK,
                                           TFT_SKYBLUE, TFT_GREENYELLOW, TFT_GOLD,  TFT_VIOLET};

struct LatLon {
  float lat, lon;
};
struct Wpt {
  float lat, lon;
  uint32_t nameAt;  // file position of the name text, 0 = no name
};
// Points first .. first + n - 1 of a line, the last one is also the first of the next chunk.
struct Chunk {
  uint32_t first;
  uint32_t n;
  tracks::Bounds b;
};
struct Line {  // only while loading
  uint32_t first, n;
};

struct Overlay {
  String path;
  uint16_t color;
  tracks::Bounds b;
  Wpt* wpts;  // sorted by latitude
  uint32_t nWpts;
  LatLon* pts;
  Chunk* chunks;
  uint32_t nChunks;
};

static Overlay files[MAX_FILES];
static int nFiles = 0;

// Names of the labels shown last, replaced in turn.
struct Name {
  uint8_t file;  // index + 1, 0 = empty
  uint32_t wpt;
  char text[NAME_LEN];
};
static Name names[NAME_CACHE];
static int nextName = 0;

static void clearNames() {
  for (Name& n : names) n.file = 0;
}

static void extend(tracks::Bounds& b, bool& any, float lat, float lon) {
  if (!any) b = {lat, lat, lon, lon};
  any = true;
  b.latMin = min(b.latMin, lat);
  b.latMax = max(b.latMax, lat);
  b.lonMin = min(b.lonMin, lon);
  b.lonMax = max(b.lonMax, lon);
}

static bool overlaps(const tracks::Bounds& a, const tracks::Bounds& b) {
  return a.latMin <= b.latMax && a.latMax >= b.latMin && a.lonMin <= b.lonMax && a.lonMax >= b.lonMin;
}

template <typename F>
static void scan(File& f, F onEvent) {
  static uint8_t buf[512];
  GpxReader r;
  uint32_t pos = 0;
  f.seek(0);
  for (int n; (n = f.read(buf, sizeof(buf))) > 0;) {
    for (int i = 0; i < n; i++, pos++) {
      GpxReader::Event e = r.feed(buf[i], pos);
      if (e) onEvent(e, r);
    }
  }
}

static void release(Overlay& o) {
  free(o.wpts);
  free(o.pts);
  free(o.chunks);
  o = Overlay();
}

int count() { return nFiles; }

int find(const String& path) {
  for (int i = 0; i < nFiles; i++)
    if (files[i].path == path) return i;
  return -1;
}

uint16_t color(int i) { return files[i].color; }

String paths() {
  String s;
  for (int i = 0; i < nFiles; i++) s += (i ? "|" : "") + files[i].path;
  return s;
}

bool bounds(tracks::Bounds& b) {
  bool any = false;
  for (int i = 0; i < nFiles; i++) {
    extend(b, any, files[i].b.latMin, files[i].b.lonMin);
    extend(b, any, files[i].b.latMax, files[i].b.lonMax);
  }
  return any;
}

bool add(const String& path, String& msg) {
  if (find(path) >= 0) return true;
  if (nFiles == MAX_FILES) {
    msg = "Max. " + String(MAX_FILES) + " overlays";
    return false;
  }
  File f = SD.open(path);
  if (!f) {
    msg = "Can't open file";
    return false;
  }

  // Pass 1: count. A line starts with its first point after <trkseg> / <rte>.
  uint32_t nWpts = 0, nLinePts = 0, nLines = 0;
  bool open = false, any = false;
  Overlay o = {};
  scan(f, [&](GpxReader::Event e, const GpxReader& r) {
    if (e == GpxReader::WPT) {
      nWpts++;
      extend(o.b, any, r.lat, r.lon);
    } else if (e == GpxReader::LINE_POINT) {
      if (!open) nLines++;
      open = true;
      nLinePts++;
      extend(o.b, any, r.lat, r.lon);
    } else if (e == GpxReader::LINE_START || e == GpxReader::LINE_END) {
      open = false;
    }
  });
  if (!nWpts && !nLinePts) {
    f.close();
    msg = "No waypoints or lines";
    return false;
  }

  // Memory: waypoints first (at most 3/4 if there are lines), lines get the
  // rest, thinned out to every stride-th point (plus the last one of each line) if needed.
  size_t heap = ESP.getFreeHeap(), block = ESP.getMaxAllocHeap();
  size_t avail = heap > HEAP_RESERVE ? heap - HEAP_RESERVE : 0;
  uint32_t keepWpts = min<size_t>(nWpts, min(nLinePts ? avail / 4 * 3 : avail, block) / sizeof(Wpt));
  avail -= keepWpts * sizeof(Wpt);
  uint32_t cap = 0, stride = 1;
  if (nLinePts && avail > nLines * sizeof(Line)) {
    size_t perPoint = sizeof(LatLon) + sizeof(Chunk) / (CHUNK_POINTS - 1) + 1;
    uint32_t maxPts = min((avail - nLines * sizeof(Line)) / perPoint, block / sizeof(LatLon));
    if (maxPts > 2 * nLines) {
      stride = (nLinePts + maxPts - 2 * nLines - 1) / (maxPts - 2 * nLines);
      cap = min(nLinePts, nLinePts / stride + 2 * nLines);
    }
  }
  o.wpts = keepWpts ? (Wpt*)malloc(keepWpts * sizeof(Wpt)) : nullptr;
  o.pts = cap ? (LatLon*)malloc(cap * sizeof(LatLon)) : nullptr;
  Line* lines = cap ? (Line*)malloc(nLines * sizeof(Line)) : nullptr;
  if ((keepWpts && !o.wpts) || (cap && (!o.pts || !lines))) {
    free(lines);
    release(o);
    f.close();
    msg = "Out of memory";
    return false;
  }

  // Pass 2: read
  uint32_t nKept = 0, nPts = 0, nLinesRead = 0, idx = 0;
  bool kept = false, pending = false;
  LatLon last = {};
  open = false;
  auto closeLine = [&] {
    if (open && pending && nPts < cap) o.pts[nPts++] = last;
    if (open) lines[nLinesRead - 1].n = nPts - lines[nLinesRead - 1].first;
    open = pending = false;
  };
  scan(f, [&](GpxReader::Event e, const GpxReader& r) {
    switch (e) {
      case GpxReader::WPT:
        kept = nKept < keepWpts;
        if (kept) o.wpts[nKept++] = {(float)r.lat, (float)r.lon, 0};
        break;
      case GpxReader::WPT_NAME:
        if (kept) o.wpts[nKept - 1].nameAt = r.nameAt;
        break;
      case GpxReader::LINE_START:
      case GpxReader::LINE_END:
        closeLine();
        break;
      case GpxReader::LINE_POINT:
        if (!cap) break;
        if (!open) {
          if (nLinesRead == nLines) break;
          lines[nLinesRead++] = {nPts, 0};
          open = true;
          idx = 0;
        }
        last = {(float)r.lat, (float)r.lon};
        pending = idx++ % stride != 0 || nPts == cap;
        if (!pending) o.pts[nPts++] = last;
        break;
      default:
        break;
    }
  });
  closeLine();
  f.close();
  o.nWpts = nKept;
  std::sort(o.wpts, o.wpts + o.nWpts, [](const Wpt& a, const Wpt& b) { return a.lat < b.lat; });

  // Lines into chunks with their bounds
  uint32_t nChunks = 0;
  for (uint32_t i = 0; i < nLinesRead; i++)
    if (lines[i].n >= 2) nChunks += (lines[i].n - 2) / (CHUNK_POINTS - 1) + 1;
  o.chunks = nChunks ? (Chunk*)malloc(nChunks * sizeof(Chunk)) : nullptr;
  if (o.chunks) {
    for (uint32_t i = 0; i < nLinesRead; i++) {
      for (uint32_t k = 0; k + 1 < lines[i].n; k += CHUNK_POINTS - 1) {
        Chunk& c = o.chunks[o.nChunks++];
        c.first = lines[i].first + k;
        c.n = min(CHUNK_POINTS, lines[i].n - k);
        bool anyPt = false;
        for (uint32_t j = c.first; j < c.first + c.n; j++) extend(c.b, anyPt, o.pts[j].lat, o.pts[j].lon);
      }
    }
  }
  free(lines);
  if (!o.chunks) {
    free(o.pts);
    o.pts = nullptr;
  } else if (nPts < cap) {
    o.pts = (LatLon*)realloc(o.pts, nPts * sizeof(LatLon));  // shrinks in place
  }
  if (!o.nWpts && !o.nChunks) {
    release(o);
    msg = "Out of memory";
    return false;
  }

  o.path = path;
  for (uint16_t c : COLORS) {  // first color not in use
    bool used = false;
    for (int i = 0; i < nFiles; i++) used |= files[i].color == c;
    if (!used) {
      o.color = c;
      break;
    }
  }
  files[nFiles++] = o;
  if (keepWpts < nWpts || (nLines && !o.nChunks)) {
    msg = "Memory full: " + String(o.nWpts) + "/" + String(nWpts) + " wpts";
    if (nLines) msg += o.nChunks ? " 1:" + String(stride) : String(", no lines");
  } else {
    msg = String(o.nWpts) + " wpts, " + String(nLinesRead) + " lines";
    if (stride > 1) msg += " 1:" + String(stride);  // every stride-th line point
  }
  return true;
}

void remove(const String& path) {
  int i = find(path);
  if (i < 0) return;
  release(files[i]);
  for (; i + 1 < nFiles; i++) files[i] = files[i + 1];
  files[--nFiles] = Overlay();
  clearNames();
}

void clear() {
  for (int i = 0; i < nFiles; i++) release(files[i]);
  nFiles = 0;
  clearNames();
}

// Name of waypoint w of file i, from the cache or from the SD card while
// reads is below MAX_NAME_READS; nullptr if not read.
static const char* wptName(int i, uint32_t w, File& f, int& fileOpen, int& reads) {
  for (Name& n : names)
    if (n.file == i + 1 && n.wpt == w) return n.text;
  if (reads >= MAX_NAME_READS) return nullptr;
  reads++;
  if (fileOpen != i) {
    if (f) f.close();
    f = SD.open(files[i].path);
    fileOpen = i;
  }
  char raw[48];
  int len = f && f.seek(files[i].wpts[w].nameAt) ? f.read((uint8_t*)raw, sizeof(raw) - 1) : 0;
  raw[max(len, 0)] = '\0';
  Name& n = names[nextName];
  nextName = (nextName + 1) % NAME_CACHE;
  n.file = i + 1;
  n.wpt = w;
  gpxNameText(raw, n.text, sizeof(n.text));
  return n.text;
}

void draw(M5Canvas& c, const Projection& p) {
  if (!nFiles) return;
  // Shown area in degrees
  tracks::Bounds view = {p.cLat - (p.y + p.h - p.cy) / p.ky, p.cLat + (p.cy - p.y) / p.ky,
                         p.cLon - (p.cx - p.x) / p.kx, p.cLon + (p.x + p.w - p.cx) / p.kx};
  // Far outside points are clamped: off screen either way, but no int overflow.
  auto sx = [&](float lon) { return (int)constrain(p.cx + (lon - p.cLon) * p.kx, -30000.0f, 30000.0f); };
  auto sy = [&](float lat) { return (int)constrain(p.cy - (lat - p.cLat) * p.ky, -30000.0f, 30000.0f); };
  c.setClipRect(p.x, p.y, p.w, p.h);

  // Lines: only the chunks in view, nothing for points on the same pixel
  for (int i = 0; i < nFiles; i++) {
    const Overlay& o = files[i];
    if (!overlaps(o.b, view)) continue;
    for (uint32_t k = 0; k < o.nChunks; k++) {
      const Chunk& ch = o.chunks[k];
      if (!overlaps(ch.b, view)) continue;
      const LatLon* pt = o.pts + ch.first;
      int x0 = sx(pt[0].lon), y0 = sy(pt[0].lat);
      for (uint32_t j = 1; j < ch.n; j++) {
        int x1 = sx(pt[j].lon), y1 = sy(pt[j].lat);
        if (x1 == x0 && y1 == y0) continue;
        c.drawLine(x0, y0, x1, y1, o.color);
        x0 = x1;
        y0 = y1;
      }
    }
  }

  // Waypoints in view: the latitude band by binary search, then by longitude
  uint32_t lo[MAX_FILES], hi[MAX_FILES];
  int visible = 0;
  for (int i = 0; i < nFiles; i++) {
    const Overlay& o = files[i];
    lo[i] = hi[i] = 0;
    if (!o.nWpts || !overlaps(o.b, view)) continue;
    lo[i] = std::lower_bound(o.wpts, o.wpts + o.nWpts, view.latMin,
                             [](const Wpt& w, float lat) { return w.lat < lat; }) - o.wpts;
    hi[i] = std::upper_bound(o.wpts, o.wpts + o.nWpts, view.latMax,
                             [](float lat, const Wpt& w) { return lat < w.lat; }) - o.wpts;
    for (uint32_t w = lo[i]; w < hi[i]; w++) visible += o.wpts[w].lon >= view.lonMin && o.wpts[w].lon <= view.lonMax;
  }
  bool labels = visible <= MAX_LABELS, dense = visible > DENSE_WPTS;
  c.setFont(&fonts::Font0);
  c.setTextDatum(top_left);
  File nameFile;
  int fileOpen = -1, reads = 0, nPlaced = 0;
  struct { int16_t x, y, w; } placed[MAX_LABELS];
  for (int i = 0; i < nFiles; i++) {
    const Overlay& o = files[i];
    c.setTextColor(o.color);
    for (uint32_t w = lo[i]; w < hi[i]; w++) {
      const Wpt& wp = o.wpts[w];
      if (wp.lon < view.lonMin || wp.lon > view.lonMax) continue;
      int x = sx(wp.lon), y = sy(wp.lat);
      if (dense) c.drawPixel(x, y, o.color);
      else c.fillRect(x - 1, y - 1, 3, 3, o.color);
      if (!labels || !wp.nameAt) continue;
      const char* name = wptName(i, w, nameFile, fileOpen, reads);
      if (!name || !*name) continue;
      // Right of the dot, left of it if that runs off the map; not over another label
      int lw = c.textWidth(name), lx = x + 4 + lw > p.x + p.w ? x - 4 - lw : x + 4, ly = y - 3;
      bool room = true;
      for (int k = 0; k < nPlaced && room; k++)
        room = lx + lw < placed[k].x || lx > placed[k].x + placed[k].w || ly + 8 < placed[k].y || ly > placed[k].y + 8;
      if (!room) continue;
      placed[nPlaced++] = {(int16_t)lx, (int16_t)ly, (int16_t)lw};
      c.drawString(name, lx, ly);
    }
  }
  if (nameFile) nameFile.close();
  c.clearClipRect();
}

}  // namespace overlay
