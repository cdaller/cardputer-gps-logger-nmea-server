# Cardputer GPS Logger

GPS track logger for the **M5Stack Cardputer ADV** (also runs on Cardputer v1.x).
Logs to micro SD as CSV and/or GPX, shows position, altitude, time and satellites
in large colored digits, and uses deep sleep between points to save battery.

## Features

- Large display: latitude, longitude, altitude, local time, satellites (used / in view)
- Status bar: recording state + point count, GPS state, SD state, battery level
- Details page: speed, course, HDOP, fix age, NMEA statistics, log file, battery voltage
- Menu page: start/stop logging, settings, help, screen off / deep sleep - no need to
  remember key letters
- Logging to `/gpslogger/tracks/YYYYMMDD_HHMMSS.csv` / `.gpx` (UTC time stamps).
  The GPX file is closed after every point, so it stays valid if power is lost.
- Optional raw NMEA log (`.nmea` next to the track): all sentences from the GPS,
  buffered in RAM and written in 4 KB chunks (up to 4 KB are lost on power loss)
- Configurable log interval (1 s … 1 h), format, time zone, brightness, timeouts
- Deep sleep between points while logging (interval >= 15 s)
- GPS module presets plus custom RX/TX pins and baud rate, or automatic baud detection
- Keyboard control when a keyboard is detected (matrix keyboard on v1.x, TCA8418 on ADV);
  without keyboard the G0 button still works
- Configuration stored as a text file on the SD card (`/gpslogger/config.txt`), so it
  survives installing other firmware via the Launcher and can be edited on a PC

## Supported GPS modules

| Preset            | RX | TX | Baud   | Notes                                    |
|-------------------|----|----|--------|------------------------------------------|
| CAP LoRa-1262     | 15 | 13 | 115200 | Cardputer ADV only (EXT header), default on ADV |
| Unit GPS v1.1     | 1  | 2  | 115200 | AT6668, Grove port, default on v1.x      |
| Unit GPS AT6558   | 1  | 2  | 9600   | older M5 GPS unit, Grove port            |
| Grove NMEA        | 1  | 2  | auto   | NEO-6M / NEO-M8N / other NMEA modules     |
| Custom            | any| any| any    | set pins and baud in the settings         |

RX/TX are seen from the Cardputer (Cardputer RX <- GPS TX). Any module that outputs
standard NMEA (GGA/RMC, GSV for satellites in view) over UART should work.
On Cardputer v1.x, pins 13/15 belong to the keyboard matrix - use the Grove port there.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```sh
pio run                 # build
pio run -t upload       # flash directly via USB
pio device monitor      # serial log
```

Every build writes two files to `dist/`:

- `GPS-Logger.bin` - application image for the **Launcher**
- `GPS-Logger-full.bin` - merged image to flash at offset `0x0` (esptool, M5Burner, web flasher)

## Start via the Launcher

1. Copy `dist/GPS-Logger.bin` to the SD card.
2. On the Cardputer start [Launcher](https://github.com/bmorcelli/Launcher), choose
   *SD*, select `GPS-Logger.bin` and install it.
3. **Important for deep sleep:** in the Launcher settings untick
   **"DeepSleep starts Launcher"**. Otherwise every wake-up from deep sleep opens the
   Launcher instead of returning to the logger.

## Keys

| Key (keyboard)     | Action                                  |
|--------------------|-----------------------------------------|
| `Enter` / `R`      | start / stop logging                    |
| `Tab`, `/` , `,`   | switch page: position -> details -> menu |
| `;` `.` + `Enter`  | menu page: select item (up/down arrow keys) and run it |
| `S`                | settings (`;` `.` select, `,` `/` change, `` ` `` / `Del` save & exit) |
| `H`                | help                                    |
| `Z`                | screen off now (or deep sleep, if active) |
| **G0 click**       | next page / leave settings / help       |
| **G0 hold**        | start / stop logging                    |

Any key turns the screen back on. G0 wakes the device from deep sleep.

## Configuration

Changed in the settings page or by editing `/gpslogger/config.txt` on the SD card
(it is created with defaults on first start):

```ini
gps_module=cap_lora1262   # cap_lora1262 | unit_gps_v11 | unit_gps_at6558 | grove_nmea | custom
gps_rx=15
gps_tx=13
gps_baud=115200           # or auto
log_interval=5            # seconds
log_format=CSV+GPX        # CSV | GPX | CSV+GPX
nmea_log=0                # 1 = also log raw NMEA to .nmea
deep_sleep=0
screen_timeout=60         # seconds, 0 = never
fix_timeout=60            # max. seconds to wait for a fix after deep sleep wake-up
tz_offset=0               # hours added to UTC for the display
auto_start=0              # start logging after power on
brightness=128
```

## Deep sleep

With `deep_sleep=1`, logging active and an interval of at least 15 s, the device goes
to deep sleep after `screen_timeout` seconds without input. It then wakes up by timer for
each point, reads the GPS (display stays off), writes the point and sleeps again. If no fix
is found within `fix_timeout` seconds, that point is skipped. Press **G0** to get the UI back.

The GPS module itself stays powered so it keeps a hot fix; it consumes far more power than
the sleeping ESP32 (typically 20-40 mA), so longer intervals save mainly the display and CPU
power. With intervals below 15 s only the screen is switched off.
