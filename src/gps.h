#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

// Reads NMEA from a UART GPS module. With baud == 0 the common baud rates
// are tried in turn until valid sentences arrive.
class GpsReader {
 public:
  void begin(int rx, int tx, uint32_t baud);
  void end();
  void update();  // call often: drains the UART into the parser

  TinyGPSPlus& gps() { return _gps; }
  bool hasFix();                   // valid, recent location
  uint32_t baud() const { return _baud; }
  bool baudLocked() const { return _locked; }
  int satsInView();                // sum over all constellations (GSV)
  uint32_t lastDataMs() const { return _lastDataMs; }
  // Called with every received byte once the baud rate is known.
  void setRawSink(void (*sink)(char c)) { _rawSink = sink; }

 private:
  void openPort(uint32_t baud);

  HardwareSerial _serial{1};
  TinyGPSPlus _gps;
  int _rx = -1, _tx = -1;
  uint32_t _baud = 0;
  bool _auto = false;
  bool _locked = false;
  uint8_t _baudIdx = 0;
  uint32_t _portOpenedMs = 0;
  uint32_t _passedAtOpen = 0;
  uint32_t _lastDataMs = 0;
  bool _running = false;
  void (*_rawSink)(char c) = nullptr;

  // Satellites in view per talker: GPS, GLONASS, Galileo, BeiDou (BD/GB), QZSS
  TinyGPSCustom _gsv[6] = {
    {_gps, "GPGSV", 3}, {_gps, "GLGSV", 3}, {_gps, "GAGSV", 3},
    {_gps, "BDGSV", 3}, {_gps, "GBGSV", 3}, {_gps, "GQGSV", 3},
  };
};
