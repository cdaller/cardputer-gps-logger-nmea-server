#pragma once
#include <Arduino.h>

// Web page on port 80 while the WiFi is on: live GPS data, the track files on
// the SD card for download and a settings page.
namespace web {
// Table rows (<tr>...</tr>) with the current GPS data, built by the caller.
void setStatusHtml(String (*fn)());
// Settings page: text() returns the config file content, apply() gets the
// changed key=value lines and returns an error, empty if accepted.
void setConfigHandlers(String (*text)(), String (*apply)(const String& lines));
void begin();
void end();
void update();  // call often: handles one request at a time
}  // namespace web
