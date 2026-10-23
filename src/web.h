#pragma once
#include <Arduino.h>

// Web page on port 80 while the WiFi is on: live GPS data and the track
// files on the SD card for download.
namespace web {
// Table rows (<tr>...</tr>) with the current GPS data, built by the caller.
void setStatusHtml(String (*fn)());
void begin();
void end();
void update();  // call often: handles one request at a time
}  // namespace web
