#include "web.h"
#include "tracks.h"
#include "version.h"
#include <SD.h>
#include <WebServer.h>

static WebServer server(80);
static bool isRunning = false;
static String (*statusHtml)() = nullptr;

static const char PAGE_HEAD[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>GPS Logger</title><style>"
    "body{font-family:sans-serif;margin:16px;background:#111;color:#eee}"
    "table{border-collapse:collapse;width:100%;max-width:640px}"
    "td,th{padding:6px 8px;border-bottom:1px solid #333;text-align:left}"
    ".r{text-align:right}a{color:#4cf}#gps td:first-child{color:#999;width:7em}"
    "footer{color:#777;margin-top:16px;font-size:small}"
    "</style></head><body><h2>GPS</h2><table id=\"gps\">";

// The GPS table is refreshed without reloading the track list.
static const char PAGE_SCRIPT[] =
    "<script>setInterval(()=>fetch('/status').then(r=>r.text())"
    ".then(t=>document.getElementById('gps').innerHTML=t).catch(()=>{}),2000)</script>";

static String htmlEscape(const String& s) {
  String out;
  for (char c : s) {
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

static String urlEncode(const String& s) {
  String out;
  char hex[4];
  for (char c : s) {
    if (isalnum((unsigned char)c) || c == '.' || c == '_' || c == '-') {
      out += c;
    } else {
      snprintf(hex, sizeof(hex), "%%%02X", (uint8_t)c);
      out += hex;
    }
  }
  return out;
}

static void handleIndex() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  server.sendContent(PAGE_HEAD);
  if (statusHtml) server.sendContent(statusHtml());
  server.sendContent("</table><h2>Tracks</h2>");
  std::vector<tracks::FileInfo> files = tracks::list();
  if (files.empty()) {
    server.sendContent("<p>No tracks on the SD card.</p>");
  } else {
    server.sendContent("<table><tr><th>File</th><th>Type</th><th class=\"r\">Size</th></tr>");
    for (const tracks::FileInfo& f : files) {
      server.sendContent("<tr><td><a href=\"/download?f=" + urlEncode(f.name) + "\">" + htmlEscape(f.name) +
                         "</a></td><td>" + tracks::TYPE_NAMES[f.type] + "</td><td class=\"r\">" +
                         tracks::sizeText(f.size) + "</td></tr>");
    }
    server.sendContent("</table>");
  }
  server.sendContent("<footer>GPS Logger v" FW_VERSION "</footer>");
  server.sendContent(PAGE_SCRIPT);
  server.sendContent("</body></html>");
  server.sendContent("");  // end of the chunked response
}

static void handleStatus() {
  server.send(200, "text/html; charset=utf-8", statusHtml ? statusHtml() : String());
}

// Only names from the track list are served, so no other file can be read.
static void handleDownload() {
  static const char* const TYPES[] = {"text/csv", "application/gpx+xml", "text/plain"};
  String name = server.arg("f");
  for (const tracks::FileInfo& f : tracks::list()) {
    if (f.name != name) continue;
    File file = SD.open(tracks::path(f));
    if (!file) break;
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + f.name + "\"");
    server.streamFile(file, TYPES[f.type]);
    file.close();
    return;
  }
  server.send(404, "text/plain", "Track not found");
}

namespace web {

void setStatusHtml(String (*fn)()) { statusHtml = fn; }

void begin() {
  if (isRunning) return;
  static bool handlersAdded = false;  // they would add up with each WiFi restart
  if (!handlersAdded) {
    server.on("/", handleIndex);
    server.on("/status", handleStatus);
    server.on("/download", handleDownload);
    handlersAdded = true;
  }
  server.begin();
  isRunning = true;
}

void end() {
  if (!isRunning) return;
  server.stop();
  isRunning = false;
}

void update() {
  if (isRunning) server.handleClient();
}

}  // namespace web
