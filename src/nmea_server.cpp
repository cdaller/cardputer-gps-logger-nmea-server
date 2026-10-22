#include "nmea_server.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/sockets.h>

static const int MAX_CLIENTS = 4;
static const uint32_t TCP_FLUSH_GAP_MS = 20;     // pause that ends a GPS burst
static const uint32_t TCP_STALL_DROP_MS = 5000;  // client not reading for this long is dropped

static WiFiServer server;
static WiFiClient clients[MAX_CLIENTS];
static bool alive[MAX_CLIENTS];  // slot holds a connected client
static uint32_t blockedSince[MAX_CLIENTS];  // send buffer full since, 0 = not blocked
// Sentences of one GPS burst are collected and sent as one TCP segment: one
// segment per sentence overflows lwIP's per-connection queue (~16 segments).
static char tcpBuf[1400];
static size_t tcpLen = 0;
static uint32_t lastLineMs = 0;
static WiFiUDP udp;
static bool isRunning = false;
static bool apMode = false;
static bool apStarted = false;  // softAP() refuses e.g. a password shorter than 8 characters
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
    apStarted = WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.length() ? cfg.apPass.c_str() : nullptr);
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
  }
  server.begin(port);
  server.setNoDelay(true);
  lineLen = 0;
  tcpLen = 0;
  isRunning = true;
}

void end() {
  if (!isRunning) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    clients[i].stop();
    alive[i] = false;
    blockedSince[i] = 0;
  }
  server.end();
  udp.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  isRunning = false;
}

static void drop(int i) {
  clients[i].stop();
  alive[i] = false;
  blockedSince[i] = 0;
}

static void flushTcp() {
  if (tcpLen == 0) return;
  uint32_t now = millis() | 1;  // never 0, which means "not blocked"
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!alive[i]) continue;
    int n = lwip_send(clients[i].fd(), tcpBuf, tcpLen, MSG_DONTWAIT);
    // A partial send cuts a sentence and the next chunk is appended to it without a line
    // break: apps that resync at '$' lose one sentence, others two.
    if (n >= 0) {
      blockedSince[i] = 0;
    } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
      drop(i);
    } else if (!blockedSince[i]) {  // send buffer full: skip this chunk for the client
      blockedSince[i] = now;
    } else if (now - blockedSince[i] > TCP_STALL_DROP_MS) {
      drop(i);  // stopped reading, e.g. out of WiFi range
    }
  }
  tcpLen = 0;
}

// WiFiClient::connected() can't be used: it calls recv() with length 0, which
// leaves errno untouched, so the result depends on an unrelated earlier error.
// Check the socket directly instead; data sent by the client is discarded.
static bool checkAlive(WiFiClient& c) {
  char buf[64];
  int n = lwip_recv(c.fd(), buf, sizeof(buf), MSG_DONTWAIT);
  if (n > 0) return true;
  if (n == 0) return false;  // closed by the client
  return errno == EWOULDBLOCK || errno == EAGAIN;
}

void update() {
  if (!isRunning) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (alive[i] && !checkAlive(clients[i])) drop(i);
  }
  if (tcpLen && millis() - lastLineMs >= TCP_FLUSH_GAP_MS) flushTcp();
  if (!server.hasClient()) return;
  WiFiClient c = server.available();
  if (c.fd() < 0) return;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!alive[i]) {
      clients[i] = c;
      alive[i] = true;
      blockedSince[i] = 0;
      return;
    }
  }
  c.stop();  // all slots in use
}

static void sendLine(const char* data, size_t len) {
  if (tcpLen + len > sizeof(tcpBuf)) flushTcp();
  memcpy(tcpBuf + tcpLen, data, len);
  tcpLen += len;
  lastLineMs = millis();
  if (networkUp()) {
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
  for (bool a : alive) n += a;
  return n;
}

int apStations() { return isRunning && apMode ? WiFi.softAPgetStationNum() : 0; }

bool running() { return isRunning; }

bool networkUp() { return isRunning && (apMode ? apStarted : WiFi.status() == WL_CONNECTED); }

String stateText() {
  if (!isRunning) return "off";
  if (apMode) return apStarted ? "running" : "AP start failed";
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
