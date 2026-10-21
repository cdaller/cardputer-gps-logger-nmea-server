#include "logger.h"
#include "config.h"
#include "version.h"
#include <SD.h>
#include <SPI.h>

// Cardputer / Cardputer ADV micro SD pins
static const int SD_SCK = 40;
static const int SD_MISO = 39;
static const int SD_MOSI = 14;
static const int SD_CS = 12;

static const char GPX_FOOTER[] = "  </trkseg>\n </trk>\n</gpx>\n";

static char nmeaBuf[4096];
static size_t nmeaLen = 0;

namespace logger {

bool sdBegin() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 20000000) && !SD.begin(SD_CS, SPI, 4000000)) return false;
  return SD.cardType() != CARD_NONE;
}

static void isoTime(const LogPoint& p, char* buf, size_t len) {
  snprintf(buf, len, "%04u-%02u-%02uT%02u:%02u:%02uZ", p.year, p.month, p.day, p.hour, p.minute,
           p.second);
}

static bool writeCsv(const LogPoint& p, const String& path) {
  bool isNew = !SD.exists(path);
  File f = SD.open(path, FILE_APPEND);
  if (!f) return false;
  if (isNew) f.println("time_utc,latitude,longitude,altitude_m,speed_kmh,course_deg,satellites,hdop,battery_pct");
  char t[24];
  isoTime(p, t, sizeof(t));
  f.printf("%s,%.7f,%.7f,%.1f,%.1f,%.1f,%u,%.2f,%d\n", t, p.lat, p.lon, p.altM, p.speedKmh,
           p.courseDeg, p.sats, p.hdop, p.batteryPct);
  f.close();
  return true;
}

// The GPX file always ends with a valid footer: each new point overwrites
// the footer and appends it again, so the file stays readable if power is lost.
static bool writeGpx(const LogPoint& p, const String& path) {
  File f;
  if (!SD.exists(path)) {
    f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    f.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<gpx version=\"1.1\" creator=\"Cardputer GPS Logger v" FW_VERSION "\" "
            "xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
            " <trk>\n  <name>");
    f.print(path.substring(path.lastIndexOf('/') + 1));
    f.print("</name>\n  <trkseg>\n");
  } else {
    f = SD.open(path, "r+");
    if (!f) return false;
    size_t footerLen = strlen(GPX_FOOTER);
    f.seek(f.size() >= footerLen ? f.size() - footerLen : f.size());
  }
  char t[24];
  isoTime(p, t, sizeof(t));
  f.printf("   <trkpt lat=\"%.7f\" lon=\"%.7f\"><ele>%.1f</ele><time>%s</time>"
           "<sat>%u</sat><hdop>%.2f</hdop></trkpt>\n",
           p.lat, p.lon, p.altM, t, p.sats, p.hdop);
  f.print(GPX_FOOTER);
  f.close();
  return true;
}

bool write(const LogPoint& p, uint8_t format, char* baseName, size_t baseNameLen) {
  if (baseName[0] == '\0') {
    SD.mkdir("/gpslogger");
    SD.mkdir(LOG_DIR);
    snprintf(baseName, baseNameLen, LOG_DIR "/%04u%02u%02u_%02u%02u%02u", p.year, p.month, p.day,
             p.hour, p.minute, p.second);
  }
  String base(baseName);
  bool ok = true;
  if (format == LOG_CSV || format == LOG_CSV_GPX) ok &= writeCsv(p, base + ".csv");
  if (format == LOG_GPX || format == LOG_CSV_GPX) ok &= writeGpx(p, base + ".gpx");
  return ok;
}

void nmeaPut(char c, const char* baseName) {
  nmeaBuf[nmeaLen++] = c;
  if (nmeaLen == sizeof(nmeaBuf)) nmeaFlush(baseName);
}

bool nmeaFlush(const char* baseName) {
  if (nmeaLen == 0) return true;
  File f = SD.open(String(baseName) + ".nmea", FILE_APPEND);
  bool ok = f && f.write((const uint8_t*)nmeaBuf, nmeaLen) == nmeaLen;
  if (f) f.close();
  nmeaLen = 0;
  return ok;
}

}  // namespace logger
