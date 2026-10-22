#include "gps.h"

static const uint32_t AUTO_BAUDS[] = {115200, 9600, 38400, 57600, 4800, 19200, 230400};
static const uint8_t AUTO_BAUD_COUNT = sizeof(AUTO_BAUDS) / sizeof(AUTO_BAUDS[0]);
static const uint32_t AUTO_BAUD_TRY_MS = 2500;
static const uint32_t SATS_MAX_AGE_MS = 10000;

const char* const GNSS_NAMES[SYS_COUNT] = {"GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "SBAS"};

void GpsReader::begin(int rx, int tx, uint32_t baud) {
  end();
  _rx = rx;
  _tx = tx;
  _auto = (baud == 0);
  _locked = !_auto;
  _baudIdx = 0;
  _running = true;
  openPort(_auto ? AUTO_BAUDS[0] : baud);
}

void GpsReader::end() {
  if (_running) _serial.end();
  _running = false;
}

void GpsReader::openPort(uint32_t baud) {
  _serial.end();
  _serial.setRxBufferSize(8192);  // ~0.7 s at 115200 baud: SD writes and screenshots block the loop
  _serial.begin(baud, SERIAL_8N1, _rx, _tx);
  _baud = baud;
  _portOpenedMs = millis();
  _passedAtOpen = _gps.passedChecksum();
}

void GpsReader::update() {
  if (!_running) return;
  while (_serial.available()) {
    char c = _serial.read();
    _gps.encode(c);
    if (c == '$') {
      _lineLen = 0;
    } else if (c == '\r' || c == '\n') {
      if (_lineLen) parseLine();
      _lineLen = 0;
    } else if (_lineLen < sizeof(_line) - 1) {
      _line[_lineLen++] = c;
    }
    if (_rawSink && _locked) _rawSink(c);
  }
  if (_gps.passedChecksum() != _passedAtOpen) {
    _lastDataMs = millis();
    if (!_locked) {
      _locked = true;
      log_i("GPS baud detected: %lu", (unsigned long)_baud);
    }
    _passedAtOpen = _gps.passedChecksum();
  }
  if (_auto && !_locked && millis() - _portOpenedMs > AUTO_BAUD_TRY_MS) {
    _baudIdx = (_baudIdx + 1) % AUTO_BAUD_COUNT;
    openPort(AUTO_BAUDS[_baudIdx]);
  }
}

bool GpsReader::hasFix() {
  return _gps.location.isValid() && _gps.location.age() < 3000;
}

int GpsReader::satsInView() {
  return _viewMs && millis() - _viewMs < SATS_MAX_AGE_MS ? _viewCount : 0;
}

int GpsReader::satsInView(int sys) {
  if (!_viewMs || millis() - _viewMs >= SATS_MAX_AGE_MS) return 0;
  int n = 0;
  for (int i = 0; i < _viewCount; i++) n += _view[i].key / 512 == sys;
  return n;
}

int GpsReader::satsUsed(int sys) {
  if (!_usedMs || millis() - _usedMs >= SATS_MAX_AGE_MS) return 0;
  int n = 0;
  for (int i = 0; i < _usedCount; i++) n += _used[i].key / 512 == sys;
  return n;
}

bool GpsReader::isUsed(uint16_t key) {
  if (!_usedMs || millis() - _usedMs >= SATS_MAX_AGE_MS) return false;
  for (int i = 0; i < _usedCount; i++) {
    if (_used[i].key == key) return true;
  }
  return false;
}

int GpsReader::satellites(SatInfo* out, int max) {
  int n = 0;
  int count = satsInView();
  for (int sys = 0; sys < SYS_COUNT; sys++) {
    for (int i = 0; i < count && n < max; i++) {
      if (_view[i].key / 512 != sys) continue;
      out[n++] = {(uint8_t)sys, (uint16_t)(_view[i].key % 512), _view[i].snr, isUsed(_view[i].key)};
    }
  }
  return n;
}

// ---- GSV / GSA parsing

// Constellation from the talker ID; -1 = mixed (GN), -2 = not counted (e.g. NavIC)
static int talkerSystem(const char* t) {
  if (t[0] == 'G' && t[1] == 'P') return SYS_GPS;
  if (t[0] == 'G' && t[1] == 'L') return SYS_GLONASS;
  if (t[0] == 'G' && t[1] == 'A') return SYS_GALILEO;
  if ((t[0] == 'G' && t[1] == 'B') || (t[0] == 'B' && t[1] == 'D')) return SYS_BEIDOU;
  if ((t[0] == 'G' && t[1] == 'Q') || (t[0] == 'Q' && t[1] == 'Z')) return SYS_QZSS;
  if (t[0] == 'G' && t[1] == 'N') return -1;
  return -2;
}

// GPS and GN sentences may contain other systems, told apart by the PRN range
// (NMEA 4.0 / u-blox numbering).
static int prnSystem(int prn, int sys) {
  if (sys != SYS_GPS && sys != -1) return sys;
  if (prn >= 1 && prn <= 32) return SYS_GPS;
  if ((prn >= 33 && prn <= 64) || (prn >= 120 && prn <= 158)) return SYS_SBAS;
  if (prn >= 65 && prn <= 96) return SYS_GLONASS;
  if (prn >= 193 && prn <= 202) return SYS_QZSS;
  if (prn >= 211 && prn <= 246) return SYS_GALILEO;
  if (prn >= 401 && prn <= 437) return SYS_BEIDOU;
  return -2;
}

void GpsReader::addSat(int sys, int prn, int snr) {
  sys = prnSystem(prn, sys);
  if (sys < 0 || prn <= 0 || prn >= 512) return;
  uint16_t key = sys * 512 + prn;
  for (int i = 0; i < _curCount; i++) {
    if (_cur[i].key == key) {
      _cur[i].snr = max<int>(_cur[i].snr, snr);  // strongest signal of a multi-band satellite
      return;
    }
  }
  if (_curCount < MAX_SATS) _cur[_curCount++] = {key, (uint8_t)constrain(snr, 0, 99)};
}

void GpsReader::finishBlock() {
  if (_block == 'V') {
    memcpy(_view, _cur, _curCount * sizeof(Sat));
    _viewCount = _curCount;
    _viewMs = millis();
  } else if (_block == 'A') {
    memcpy(_used, _cur, _curCount * sizeof(Sat));
    _usedCount = _curCount;
    _usedMs = millis();
  }
  _curCount = 0;
}

void GpsReader::parseLine() {
  _line[_lineLen] = '\0';
  char* star = strchr(_line, '*');
  if (!star) return;
  uint8_t sum = 0;
  for (char* p = _line; p < star; p++) sum ^= *p;
  if (sum != strtol(star + 1, nullptr, 16)) return;
  *star = '\0';

  char* f[24];
  int n = 0;
  for (char* p = _line; n < 24;) {
    f[n++] = p;
    p = strchr(p, ',');
    if (!p) break;
    *p++ = '\0';
  }

  const char* type = strlen(f[0]) == 5 ? f[0] + 2 : "";
  char block = strcmp(type, "GSV") == 0 ? 'V' : strcmp(type, "GSA") == 0 ? 'A' : 0;
  if (block != _block) {
    finishBlock();
    _block = block;
  }
  int sys = talkerSystem(f[0]);

  if (block == 'V') {
    // prn, elevation, azimuth, snr per satellite; NMEA 4.10 appends a signal ID
    for (int i = 4; i + 4 <= n; i += 4) addSat(sys, atoi(f[i]), atoi(f[i + 3]));
  } else if (block == 'A') {
    // NMEA 4.10: system ID after PDOP / HDOP / VDOP
    static const int8_t SYSTEM_IDS[] = {-2, SYS_GPS, SYS_GLONASS, SYS_GALILEO, SYS_BEIDOU, SYS_QZSS};
    if (n > 18 && *f[18]) {
      int id = atoi(f[18]);
      sys = id >= 1 && id <= 5 ? SYSTEM_IDS[id] : -2;
    }
    for (int i = 3; i <= 14 && i < n; i++) addSat(sys, atoi(f[i]), 0);
  }
}
