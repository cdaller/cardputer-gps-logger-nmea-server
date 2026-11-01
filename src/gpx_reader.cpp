#include "gpx_reader.h"
#include <stdlib.h>
#include <string.h>

GpxReader::Event GpxReader::feed(char c, uint32_t pos) {
  switch (state) {
    case TEXT:
      if (c == '<') {
        state = TAG;
        len = 0;
        quote = 0;
      }
      return NONE;
    case COMMENT:
      if (c == '>' && dashes >= 2) state = TEXT;
      dashes = c == '-' ? dashes + 1 : 0;
      return NONE;
    case TAG:
      if (quote) {
        if (c == quote) quote = 0;
      } else if (c == '"' || c == '\'') {
        quote = c;
      } else if (c == '>') {
        state = TEXT;
        buf[len] = '\0';
        return tag(pos + 1);
      }
      if (len < sizeof(buf) - 1) buf[len++] = c;
      if (len == 3 && memcmp(buf, "!--", 3) == 0) {
        state = COMMENT;
        dashes = 0;
      }
      return NONE;
  }
  return NONE;
}

// lat="..." and lon="..." in any order, quoted with " or '.
bool GpxReader::latLon() {
  bool hasLat = false, hasLon = false;
  for (char* p = buf; (p = strstr(p, "=")) != nullptr; p++) {
    char* name = p;
    while (name > buf && name[-1] != ' ' && name[-1] != '\t' && name[-1] != '\n' && name[-1] != '\r') name--;
    size_t n = p - name;
    char q = p[1];
    if (q != '"' && q != '\'') continue;
    if (n == 3 && memcmp(name, "lat", 3) == 0) {
      lat = strtod(p + 2, nullptr);
      hasLat = true;
    } else if (n == 3 && memcmp(name, "lon", 3) == 0) {
      lon = strtod(p + 2, nullptr);
      hasLon = true;
    }
  }
  return hasLat && hasLon;
}

GpxReader::Event GpxReader::tag(uint32_t posAfter) {
  bool end = buf[0] == '/';
  bool selfClosing = len && buf[len - 1] == '/';
  char* name = buf + (end ? 1 : 0);
  size_t n = strcspn(name, " \t\r\n/");
  char* colon = (char*)memchr(name, ':', n);  // namespace prefix
  if (colon) {
    n -= colon + 1 - name;
    name = colon + 1;
  }
  auto is = [&](const char* s) { return strlen(s) == n && memcmp(name, s, n) == 0; };
  if (end) {
    if (is("wpt")) inWpt = false;
    else if (is("trkseg") || is("rte")) return LINE_END;
    return NONE;
  }
  if (is("wpt")) {
    if (!latLon()) return NONE;
    inWpt = !selfClosing;
    return WPT;
  }
  if (is("name")) {
    if (!inWpt || selfClosing) return NONE;
    nameAt = posAfter;
    return WPT_NAME;
  }
  if (is("trkseg") || is("rte")) return selfClosing ? NONE : LINE_START;
  if (is("trkpt") || is("rtept")) return latLon() ? LINE_POINT : NONE;
  return NONE;
}

// U+00C0 .. U+017F without accents.
static const char LATIN[] =
    "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty"
    "AaAaAaCcCcCcCcDdDdEeEeEeEeEeGgGgGgGgHhHhIiIiIiIiIiIiJjKkkLlLlLlLlLlNnNnNnnNnOoOoOoOoRrRrRrSsSsSsSsTtTtTtUuUuUuUuUuUuWwYyYZzZzZzs";

void gpxNameText(const char* s, char* out, size_t outLen) {
  static const struct { const char* entity; char c; } ENTITIES[] = {
    {"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}};
  size_t len = 0;
  auto put = [&](const char* t) {
    while (*t && len + 1 < outLen) out[len++] = *t++;
  };
  bool cdata = strncmp(s, "<![CDATA[", 9) == 0;
  if (cdata) s += 9;
  for (; *s && len + 1 < outLen; s++) {
    uint8_t c = *s;
    if (cdata ? c == ']' && s[1] == ']' : c == '<') break;
    if (c == '&' && !cdata) {
      char one[2] = {'&', 0};
      for (auto& en : ENTITIES)
        if (strncmp(s, en.entity, strlen(en.entity)) == 0) {
          one[0] = en.c;
          s += strlen(en.entity) - 1;
          break;
        }
      put(one);
    } else if (c >= 0xC0 && c < 0xE0 && (s[1] & 0xC0) == 0x80) {  // 2 byte UTF-8
      unsigned cp = (c & 0x1F) << 6 | (s[1] & 0x3F);
      s++;
      if (cp == 0xC4) put("Ae");
      else if (cp == 0xD6) put("Oe");
      else if (cp == 0xDC) put("Ue");
      else if (cp == 0xDF) put("ss");
      else if (cp == 0xE4) put("ae");
      else if (cp == 0xF6) put("oe");
      else if (cp == 0xFC) put("ue");
      else if (cp >= 0xC0 && cp < 0x180) {
        char one[2] = {LATIN[cp - 0xC0], 0};
        put(one);
      } else put("?");
    } else if (c >= 0x80) {
      if ((c & 0xC0) == 0xC0) put("?");  // lead byte of a longer sequence, continuation bytes skipped
    } else {
      char one[2] = {(char)(c < ' ' ? ' ' : c), 0};
      put(one);
    }
  }
  out[len] = '\0';
  // trim
  size_t start = 0;
  while (out[start] == ' ') start++;
  while (len > start && out[len - 1] == ' ') len--;
  memmove(out, out + start, len - start);
  out[len - start] = '\0';
}
