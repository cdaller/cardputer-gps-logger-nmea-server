#pragma once
#include <stddef.h>
#include <stdint.h>

// Streaming scanner for the GPX elements the map overlays need: waypoints
// with the file position of their name, track segments and routes with their
// points. Works on any layout (one element per line or all in one line),
// skips comments and everything else. Plain C++, no Arduino.
class GpxReader {
 public:
  enum Event : uint8_t {
    NONE,
    WPT,         // <wpt>: lat, lon
    WPT_NAME,    // <name> inside a <wpt>: nameAt
    LINE_START,  // <trkseg> or <rte>
    LINE_POINT,  // <trkpt> or <rtept>: lat, lon
    LINE_END,    // </trkseg> or </rte>
  };

  // One byte of the file at position pos; returns the event it completes.
  Event feed(char c, uint32_t pos);

  double lat = 0, lon = 0;
  uint32_t nameAt = 0;  // file position of the name text

 private:
  enum State : uint8_t { TEXT, TAG, COMMENT };
  Event tag(uint32_t posAfter);
  bool latLon();

  State state = TEXT;
  char quote = 0;     // inside an attribute value
  uint8_t dashes = 0; // comment: '-' in a row, "-->" ends it
  bool inWpt = false;
  uint8_t len = 0;
  char buf[128];      // tag without < >, longer tags are cut off
};

// Name text as read from the file at nameAt (up to the next '<', CDATA too)
// for the screen fonts, which have ASCII only: XML entities decoded, accented
// Latin letters without accent, German umlauts as ae, oe, ue, ss, other
// characters as '?'. Trimmed, cut off at outLen - 1 characters.
void gpxNameText(const char* raw, char* out, size_t outLen);
