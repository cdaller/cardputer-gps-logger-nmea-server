#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

// Constellations counted on the satellites page.
enum GnssSystem { SYS_GPS, SYS_GLONASS, SYS_GALILEO, SYS_BEIDOU, SYS_QZSS, SYS_SBAS, SYS_COUNT };
extern const char* const GNSS_NAMES[SYS_COUNT];

struct SatInfo {
  uint8_t sys;   // GnssSystem
  uint16_t prn;
  uint8_t snr;   // dB-Hz, 0 = not tracked
  bool used;     // used in the fix
};

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
  int satsInView(int sys);         // unique satellites of one constellation (GSV)
  int satsUsed(int sys);           // satellites of one constellation used in the fix (GSA)
  // Satellites in view with signal strength, ordered by constellation; returns the count.
  int satellites(SatInfo* out, int max);
  uint32_t lastDataMs() const { return _lastDataMs; }
  // Called with every received byte once the baud rate is known.
  void setRawSink(void (*sink)(char c)) { _rawSink = sink; }

 private:
  void openPort(uint32_t baud);
  void parseLine();
  void addSat(int sys, int prn, int snr);
  void finishBlock();
  bool isUsed(uint16_t key);

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

  // Own line parser for GSV / GSA. A run of consecutive GSV (or GSA) sentences
  // is one epoch; it is published when another sentence type follows.
  // Satellites are keyed by sys * 512 + PRN and deduplicated (multi-band receivers
  // report the same satellite once per signal).
  struct Sat { uint16_t key; uint8_t snr; };
  static const int MAX_SATS = 64;
  char _line[128];
  uint8_t _lineLen = 0;
  char _block = 0;          // 'V' = in a GSV run, 'A' = in a GSA run
  Sat _cur[MAX_SATS];       // run being collected
  uint8_t _curCount = 0;
  Sat _view[MAX_SATS];      // last GSV run
  uint8_t _viewCount = 0;
  Sat _used[MAX_SATS];      // last GSA run (snr unused)
  uint8_t _usedCount = 0;
  uint32_t _viewMs = 0, _usedMs = 0;
};
