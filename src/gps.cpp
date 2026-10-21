#include "gps.h"

static const uint32_t AUTO_BAUDS[] = {115200, 9600, 38400, 57600, 4800, 19200, 230400};
static const uint8_t AUTO_BAUD_COUNT = sizeof(AUTO_BAUDS) / sizeof(AUTO_BAUDS[0]);
static const uint32_t AUTO_BAUD_TRY_MS = 2500;

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
  _serial.setRxBufferSize(2048);
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
  int total = 0;
  for (auto& g : _gsv) {
    if (g.isValid() && g.age() < 10000) total += atoi(g.value());
  }
  return total;
}
