#include "nmea_server.h"
#include "web.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/sockets.h>

static const int MAX_CLIENTS = 4;
static const uint32_t TCP_FLUSH_GAP_MS = 20;     // pause that ends a GPS burst
static const uint32_t TCP_STALL_DROP_MS = 5000;  // client not reading for this long is dropped
static const uint32_t WIFI_FALLBACK_MS = 30000;  // no known network joined: own AP after this round
// One network, then the next: the core retries within this time, e.g. after a
// handshake timeout, which many routers cause on the first attempt.
static const uint32_t WIFI_CONNECT_MS = 15000;

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
static bool apMode = false;     // own AP: configured or as fallback in client mode
static bool apStarted = false;  // softAP() refuses e.g. a password shorter than 8 characters
static bool fallbackAp = false;
// Client mode: scan, join the strongest known network not tried yet in this
// round, scan again if that fails. Known networks missing in the scan are
// tried too: hidden networks don't show their name.
enum WifiState : uint8_t { W_SCAN, W_CONNECT, W_CONNECTED, W_AP };
static WifiState wifiState = W_AP;
static uint32_t searchStartMs = 0;   // no network since
static uint32_t connectStartMs = 0;
static String joinSsid;              // network joined or being joined
static bool tried[WIFI_NETWORKS];
static volatile uint8_t failReason = 0;  // last disconnect reason, set by the WiFi task
static String failText;  // last failed attempt: "name: reason"
static uint32_t failures = 0;
static Config netCfg;
static uint16_t port = 0;
static char line[128];
static size_t lineLen = 0;

static void startAp() {
  WiFi.mode(WIFI_AP);
  apStarted = WiFi.softAP(netCfg.apSsid.c_str(), netCfg.apPass.length() ? netCfg.apPass.c_str() : nullptr);
  apMode = true;
  wifiState = W_AP;
}

static void startScan() {
  joinSsid = "";
  WiFi.scanNetworks(true);
  wifiState = W_SCAN;
}

// After all known networks were tried: next round, or the own AP once the search takes too long.
static void searchOn() {
  if (millis() - searchStartMs < WIFI_FALLBACK_MS) {
    startScan();
  } else {
    fallbackAp = true;
    startAp();
  }
}

static void onDisconnect(arduino_event_id_t, arduino_event_info_t info) {
  uint8_t r = info.wifi_sta_disconnected.reason;
  if (r != WIFI_REASON_ASSOC_LEAVE) failReason = r;  // ASSOC_LEAVE: own disconnect()
}

static String reasonText(uint8_t r) {
  switch (r) {
    case 0: return "no answer";  // e.g. no address from DHCP
    case WIFI_REASON_NO_AP_FOUND: return "not found (5 GHz?)";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "wrong password?";
    case WIFI_REASON_AUTH_EXPIRE: return "auth timeout";
    case WIFI_REASON_ASSOC_FAIL:
    case WIFI_REASON_CONNECTION_FAIL: return "connect failed";
  }
  return "error " + String(r);
}

static void resetTried() {
  for (bool& t : tried) t = false;
}

// Joins the strongest known network of the scan not tried yet, else a known
// network that wasn't in the scan. All tried: next round.
static void joinNext(int found) {
  int best = -1, slot = -1;
  for (int i = 0; i < found; i++) {
    if (best >= 0 && WiFi.RSSI(i) <= WiFi.RSSI(best)) continue;
    for (int k = 0; k < WIFI_NETWORKS; k++) {
      if (!tried[k] && netCfg.wifiSsid[k].length() && WiFi.SSID(i) == netCfg.wifiSsid[k]) {
        best = i;
        slot = k;
        break;
      }
    }
  }
  for (int k = 0; slot < 0 && k < WIFI_NETWORKS; k++) {
    if (!tried[k] && netCfg.wifiSsid[k].length()) slot = k;
  }
  if (slot >= 0) {
    tried[slot] = true;
    joinSsid = netCfg.wifiSsid[slot];
    failReason = 0;
    if (best >= 0) WiFi.begin(joinSsid.c_str(), netCfg.wifiPass[slot].c_str(), WiFi.channel(best), WiFi.BSSID(best));
    else WiFi.begin(joinSsid.c_str(), netCfg.wifiPass[slot].c_str());  // probes for a hidden network
    connectStartMs = millis();
    wifiState = W_CONNECT;
  }
  WiFi.scanDelete();
  if (slot < 0) {
    resetTried();
    searchOn();
  }
}

static void updateWifi() {
  switch (wifiState) {
    case W_SCAN: {
      int n = WiFi.scanComplete();
      if (n != WIFI_SCAN_RUNNING) joinNext(n);  // n < 0: scan failed, nothing found
      break;
    }
    case W_CONNECT:
      if (WiFi.status() == WL_CONNECTED) {
        wifiState = W_CONNECTED;
        resetTried();
      } else if (millis() - connectStartMs >= WIFI_CONNECT_MS) {
        failText = joinSsid + ": " + reasonText(failReason);
        failures++;
        log_w("WiFi %s", failText.c_str());
        WiFi.disconnect();
        startScan();  // the next network
      }
      break;
    case W_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {  // lost: the core reconnects, else search again
        searchStartMs = connectStartMs = millis();
        wifiState = W_CONNECT;
      }
      break;
    case W_AP:
      break;
  }
}

namespace nmeaserver {

void begin(const Config& cfg) {
  end();
  netCfg = cfg;
  port = cfg.nmeaPort;
  apMode = apStarted = fallbackAp = false;
  if (cfg.wifiClient) {
    static bool eventAdded = false;
    if (!eventAdded) {
      WiFi.onEvent(onDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
      eventAdded = true;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    resetTried();
    searchStartMs = millis();
    startScan();
  } else {
    startAp();
  }
  server.begin(port);
  server.setNoDelay(true);
  web::begin();
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
  web::end();
  udp.stop();
  WiFi.scanDelete();
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
  updateWifi();
  web::update();
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

bool apActive() { return isRunning && apMode; }

bool apFallback() { return isRunning && fallbackAp; }

String lastFailure() { return failText; }

uint32_t failureCount() { return failures; }

String networkName() {
  if (!isRunning) return "";
  return apMode ? netCfg.apSsid : joinSsid;
}

bool networkUp() { return isRunning && (apMode ? apStarted : WiFi.status() == WL_CONNECTED); }

String stateText() {
  if (!isRunning) return "off";
  if (apMode) return !apStarted ? "AP start failed" : fallbackAp ? "no network: own AP" : "running";
  switch (wifiState) {
    case W_SCAN: return "searching...";
    case W_CONNECT: return "connecting...";
    default: return "connected";
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
