#pragma once
#include <Arduino.h>
#include "config.h"

// Serves the raw NMEA sentences from the GPS over WiFi, e.g. for the Garmin
// Boating app: TCP server and UDP broadcast on the same port. The WiFi is
// either an own access point or an existing network (client mode).
namespace nmeaserver {
void begin(const Config& cfg);
void end();
void update();      // call often: accepts new TCP clients
void put(char c);   // raw GPS bytes; each complete sentence is sent
String statusText();
bool running();
bool networkUp();  // AP started or connected to the WiFi
// TCP clients; in AP mode at least the number of devices joined to the AP
// (UDP listeners can't be seen otherwise).
int deviceCount();
}  // namespace nmeaserver
