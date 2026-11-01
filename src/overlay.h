#pragma once
#include <Arduino.h>
#include <M5GFX.h>
#include "tracks.h"

// GPX files shown on the maps: waypoints (cities, summits, ...) with their
// names and lines (tracks and routes). Kept in RAM as compact as possible:
// each file is read twice, first to count, then into exactly sized arrays.
// Lines are thinned out evenly when they don't fit into the free memory.
// Waypoint names stay on the SD card and are read only for the labels shown.
namespace overlay {

static const int MAX_FILES = 8;
static const char* const DEFAULT_DIR = "/gpslogger/gpx";

// Reads a GPX file; msg says what was loaded or why not. False if nothing to show.
bool add(const String& path, String& msg);
void remove(const String& path);
void clear();
int count();
int find(const String& path);  // index, -1 if not loaded
uint16_t color(int i);
String paths();                // all loaded files, separated by '|'
bool bounds(tracks::Bounds& b);  // of all overlays, false if none

// Screen position of lat / lon: x = cx + (lon - cLon) * kx, y = cy - (lat - cLat) * ky.
// Drawing is clipped to the rectangle x, y, w, h.
struct Projection {
  float cLat, cLon, kx, ky;
  int cx, cy;
  int x, y, w, h;
};
void draw(M5Canvas& c, const Projection& p);

}  // namespace overlay
