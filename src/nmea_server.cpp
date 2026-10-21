#include "nmea_server.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/sockets.h>

static const int MAX_CLIENTS = 4;

static WiFiServer server;
static WiFiClient clients[MAX_CLIENTS];
static WiFiUDP udp;
static bool isRunning = false;
static bool apMode = false;
static uint16_t port = 0;
static char line[128];
static size_t lineLen = 0;

namespace nmeaserver {

void begin(const Config& cfg) {
  end();
  apMode = !cfg.wifiClient;
  port = cfg.nmeaPort;
  if (apMode) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.length() ? cfg.apPass.c_str() : nullptr);
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
  }
  server.begin(port);
  server.setNoDelay(true);
  lineLen = 0;
  isRunning = true;
}

void end() {
  if (!isRunning) return;
  for (auto& c : clients) c.stop();
  server.end();
  udp.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  isRunning = false;
}

void update() {
  if (!isRunning || !server.hasClient()) return;
  WiFiClient c = server.available();
  for (auto& slot : clients) {
    if (!slot.connected()) {
      slot = c;
      return;
    }
  }
  c.stop();  // all slots in use
}

static void sendLine(const char* data, size_t len) {
  for (auto& c : clients) {
    // Non-blocking: a client that stopped reading (e.g. out of WiFi range) fills
    // the send buffer; drop it instead of blocking the main loop.
    if (c.connected() && lwip_send(c.fd(), data, len, MSG_DONTWAIT) != (int)len) c.stop();
  }
  if (apMode || WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(apMode ? WiFi.softAPBroadcastIP() : WiFi.broadcastIP(), port);
    udp.write((const uint8_t*)data, len);
    udp.endPacket();
  }
}

void put(char c) {
  if (!isRunning) return;
  if (lineLen < sizeof(line)) line[lineLen++] = c;
  if (c != '\n') return;
  // Overlong lines are garbage; the first line may start mid-sentence.
  if (lineLen < sizeof(line) && (line[0] == '$' || line[0] == '!')) sendLine(line, lineLen);
  lineLen = 0;
}

int tcpClients() {
  int n = 0;
  for (auto& c : clients) n += c.connected();
  return n;
}

int apStations() { return isRunning && apMode ? WiFi.softAPgetStationNum() : 0; }

bool running() { return isRunning; }

bool networkUp() { return isRunning && (apMode || WiFi.status() == WL_CONNECTED); }

String stateText() {
  if (!isRunning) return "off";
  if (apMode) return "running";
  switch (WiFi.status()) {
    case WL_CONNECTED: return "connected";
    case WL_NO_SSID_AVAIL: return "network not found";
    case WL_CONNECT_FAILED: return "connect failed";
    default: return "connecting...";
  }
}

String ipText() {
  if (!networkUp()) return "";
  return (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString();
}

String statusText() {
  if (!networkUp()) return stateText();
  String s = ipText() + ":" + String(port) + " tcp:" + String(tcpClients());
  if (apMode) s += " ap:" + String(apStations());
  return s;
}

int deviceCount() { return max(tcpClients(), apStations()); }

}  // namespace nmeaserver
