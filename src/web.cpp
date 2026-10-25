#include "web.h"
#include "tracks.h"
#include "version.h"
#include <SD.h>
#include <WebServer.h>

static WebServer server(80);
static bool isRunning = false;
static String (*statusHtml)() = nullptr;
static String (*configText)() = nullptr;
static String (*applyConfig)(const String& lines) = nullptr;

static const char PAGE_HEAD[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>GPS Logger</title><style>"
    "body{font-family:sans-serif;margin:16px;background:#111;color:#eee}"
    "table{border-collapse:collapse;width:100%;max-width:640px}"
    "td,th{padding:6px 8px;border-bottom:1px solid #333;text-align:left}"
    ".r{text-align:right}a{color:#4cf}#gps td:first-child{color:#999;width:7em}"
    "footer{color:#777;margin-top:16px;font-size:small}"
    "label{display:block;margin-top:12px;font-weight:bold}small{display:block;color:#999}"
    "input{width:100%;max-width:640px;box-sizing:border-box;padding:6px;background:#222;color:#eee;"
    "border:1px solid #444}button{margin-top:16px;padding:8px 24px}"
    "</style></head><body>";

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
  server.sendContent("<h2>GPS</h2><table id=\"gps\">");
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
  server.sendContent("<p><a href=\"/settings\">Settings</a></p>");
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

// Passwords are not sent to the browser: an empty field keeps the stored one.
static bool isPassword(const String& key) { return key.indexOf("pass") >= 0; }

// The config file as a form: one field per key, its comment lines as hint.
static void sendSettingsForm(const String& message) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  server.sendContent(PAGE_HEAD);
  server.sendContent("<h2>Settings</h2>");
  if (message.length()) server.sendContent("<p><b>" + htmlEscape(message) + "</b></p>");
  server.sendContent("<form method=\"post\" action=\"/settings\">");
  String text = configText(), hint;
  int start = 0;
  while (start < (int)text.length()) {
    int end = text.indexOf('\n', start);
    if (end < 0) end = text.length();
    String line = text.substring(start, end);
    start = end + 1;
    line.trim();
    if (line.startsWith("#")) {
      hint += htmlEscape(line.substring(1)) + "<br>";
      continue;
    }
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = htmlEscape(line.substring(0, eq));
    String field = "<label>" + key + "</label>";
    if (hint.length()) field += "<small>" + hint + "</small>";
    if (isPassword(key)) {
      field += "<input type=\"password\" name=\"" + key + "\" placeholder=\"unchanged\" autocomplete=\"off\">";
    } else {
      field += "<input name=\"" + key + "\" value=\"" + htmlEscape(line.substring(eq + 1)) + "\">";
    }
    server.sendContent(field);
    hint = "";
  }
  server.sendContent("<button>Save</button></form><p><a href=\"/\">Back</a></p>");
  server.sendContent("<footer>GPS Logger v" FW_VERSION "</footer></body></html>");
  server.sendContent("");
}

static void handleSettings() {
  if (!configText || !applyConfig) {
    server.send(404, "text/plain", "Not available");
    return;
  }
  if (server.method() != HTTP_POST) {
    sendSettingsForm("");
    return;
  }
  String lines;
  for (int i = 0; i < server.args(); i++) {
    String key = server.argName(i), val = server.arg(i);
    val.replace("\r", "");
    val.replace("\n", "");
    if (key == "plain" || (isPassword(key) && val.length() == 0)) continue;  // plain: raw body
    lines += key + "=" + val + "\n";
  }
  String error = applyConfig(lines);
  sendSettingsForm(error.length() ? error
                                  : "Saved. If the WiFi settings changed, the network restarts: reconnect.");
}

namespace web {

void setStatusHtml(String (*fn)()) { statusHtml = fn; }

void setConfigHandlers(String (*text)(), String (*apply)(const String& lines)) {
  configText = text;
  applyConfig = apply;
}

void begin() {
  if (isRunning) return;
  static bool handlersAdded = false;  // they would add up with each WiFi restart
  if (!handlersAdded) {
    server.on("/", handleIndex);
    server.on("/status", handleStatus);
    server.on("/download", handleDownload);
    server.on("/settings", handleSettings);
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
