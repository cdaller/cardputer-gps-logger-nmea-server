# Cardputer GPS Logger / NMEA Server

GPS track logger and **WiFi NMEA server** for the **M5Stack Cardputer ADV** (also runs on
Cardputer v1.x).

- **Logger:** logs to micro SD as CSV and/or GPX, shows position, altitude, speed, time and
  satellites in large colored digits, and uses deep sleep between points to save battery.
- **NMEA server:** turns the Cardputer into a wireless GPS for phones, tablets and PCs: the
  NMEA sentences are sent over WiFi (TCP + UDP) to chart and navigation apps like Garmin
  Boating - in its own WiFi network or in an existing one. Works at the same time as
  logging (deep sleep is off while the server runs). See [NMEA server](#nmea-server-for-eg-garmin-boating-app) below.

## Features

- NMEA server over WiFi (TCP + UDP) for chart apps like Garmin Boating, either with its
  own WiFi network or in an existing one
- Large display: latitude, longitude, altitude, local time, satellites (used / in view)
- Status bar: recording state + point count, satellite reception (FIX / NO FIX, used / in
  view), NMEA server (`↑ NMEA`), missing SD card, battery level (percentage left out if the
  bar is full)
- Big page: speed, direction (degrees and N / NE / E ...) and altitude in large letters
- Details page: speed, course, HDOP with a rough accuracy estimate (HDOP x 5 m), fix age,
  NMEA statistics, log file, battery voltage
- Satellites page: used / in view per constellation (GPS, GLONASS, Galileo, BeiDou, QZSS,
  SBAS) as bars, green = used in the fix
- Signal page: signal strength (SNR, dB-Hz) of every satellite in view as a bar chart,
  green = used in the fix; letters below the bars: G GPS, R GLONASS, E Galileo,
  C BeiDou, J QZSS, S SBAS
- Menu page: start/stop logging, start/stop NMEA server, settings, help, screen off /
  deep sleep - no need to remember key letters
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

## Screenshots

| | |
|---|---|
| ![Position page](docs/screen_main.png) | ![Position page while recording](docs/screen_main_recording.png) |
| **Position:** latitude, longitude, altitude, satellites and local time | **Recording:** blinking `REC` with point count, `↑ NMEA` while the NMEA server runs |
| ![Details page](docs/screen_info.png) | ![Satellites page](docs/screen_satellites.png) |
| **Details:** speed, course, HDOP and accuracy, GPS module, log and battery | **Satellites:** used / in view per constellation |
| ![Signal page](docs/screen_sat_signals.png) | ![NMEA server page](docs/screen_nmea_server.png) |
| **Signal:** SNR of every satellite in view, green = used in the fix | **NMEA server:** network, address and port for the app, connected devices |
| ![Menu page](docs/screen_menu.png) | |
| **Menu:** logging, NMEA server, settings, help, screen off | |

## Supported hardware

| Device                         | Notes                                                     |
|--------------------------------|-----------------------------------------------------------|
| M5Stack Cardputer ADV          | main target; keyboard via TCA8418, CAP LoRa-1262 GPS on the EXT header |
| M5Stack Cardputer v1.0 / v1.1  | matrix keyboard; GPS on the Grove port                    |

The model is detected at runtime, one firmware image runs on all of them. The firmware is
built for the ESP32-S3 (StampS3, 8 MB flash) and uses the Cardputer's display (240 x 135),
SD card slot, battery ADC and keyboard pins, so it does not run on other M5Stack or
ESP32 devices without changes. A micro SD card is needed for logging and for saving the
settings.

## Supported GPS modules

| Preset            | RX | TX | Baud   | Notes                                    |
|-------------------|----|----|--------|------------------------------------------|
| CAP LoRa-1262     | 15 | 13 | 115200 | Cardputer ADV only (EXT header), default on ADV |
| Unit GPS v1.1     | 1  | 2  | 115200 | AT6668, Grove port, default on v1.x      |
| Unit GPS AT6558   | 1  | 2  | 9600   | older M5 GPS unit, Grove port            |
| Grove NMEA        | 1  | 2  | auto   | NEO-6M / NEO-M8N / other NMEA modules     |
| Custom            | any| any| any    | set pins and baud in the settings         |

RX/TX are seen from the Cardputer (Cardputer RX <- GPS TX). Any module that outputs
standard NMEA (GGA/RMC, GSV for satellites in view, GSA for satellites used) over UART
should work.
On Cardputer v1.x, pins 13/15 belong to the keyboard matrix - use the Grove port there.

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```sh
pio run                 # build
pio run -t upload       # flash directly via USB
pio device monitor      # serial log
```

Every build writes two files to `dist/`:

- `GPS-NMEA-Logger.bin` - application image for the **Launcher**
- `GPS-NMEA-Logger-full.bin` - merged image to flash at offset `0x0` (esptool, M5Burner, web flasher)

The firmware version is set in `src/version.h`, shown on the help page (`H`) and written
to the `creator` attribute of GPX files.

## Start via the Launcher

1. Copy `dist/GPS-NMEA-Logger.bin` to the SD card.
2. On the Cardputer start [Launcher](https://github.com/bmorcelli/Launcher), choose
   *SD*, select `GPS-NMEA-Logger.bin` and install it.
3. **Important for deep sleep:** in the Launcher settings untick
   **"DeepSleep starts Launcher"**. Otherwise every wake-up from deep sleep opens the
   Launcher instead of returning to the logger.

## Keys

| Key (keyboard)     | Action                                  |
|--------------------|-----------------------------------------|
| `Enter` / `R`      | start / stop logging                    |
| `Tab`, `/` , `,`   | switch page: position -> big -> details -> satellites -> signal -> WiFi -> menu (WiFi only with NMEA server on) |
| `1` ... `7`        | jump to page: 1 position, 2 big, 3 details, 4 satellites, 5 signal, 6 WiFi, 7 menu |
| `;` `.` + `Enter`  | menu page: select item (up/down arrow keys) and run it |
| `S`                | settings (`;` `.` select, `,` `/` change, `` ` `` / `Del` save & exit) |
| `H`                | help                                    |
| `Z`                | screen off now (or deep sleep, if active) |
| `P`                | screenshot of the current page as BMP to `/gpslogger/screenshots/` |
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
tz_offset=0               # added to UTC for the display: hours or hours:minutes (+5:30)
auto_start=0              # start logging after power on
brightness=128
nmea_server=0             # 1 = send NMEA over WiFi
wifi_mode=ap              # ap = own network, client = join wifi_ssid
wifi_ssid=
wifi_pass=
ap_ssid=GPS-NMEA-Logger
ap_pass=k7rmx2qhwa        # random on first start; empty = open network, else min. 8 characters
nmea_port=2000            # TCP server and UDP broadcast port
```

WiFi names and passwords can also be typed in the settings page: select the entry, press
`Enter`, type, `Enter` to accept (`Del` deletes, G0 cancels). They are stored in plain text
in `config.txt`.

## NMEA server (for e.g. Garmin Boating app)

With `nmea_server=1` the Cardputer sends every NMEA sentence from the GPS over WiFi:
to all TCP clients connected to `nmea_port` (up to 4) and as UDP broadcast to the same port.

- **Own network** (`wifi_mode=ap`): join the WiFi `GPS-NMEA-Logger` with the iPad, using the
  password shown on the WiFi page (generated randomly on first start, stored as `ap_pass`).
  The Cardputer has the address `192.168.4.1`.
- **Existing network** (`wifi_mode=client`): the Cardputer joins `wifi_ssid`; its address
  is shown on the WiFi page.

The NMEA sentences contain your position: every device that can reach the port gets it,
and in client mode the UDP broadcast sends it to every device in that network. Use the own
network, or a WiFi you trust.

In the Garmin Boating app: *Menu > Paired devices > + > Add device*, enter
host = the address above, port = `nmea_port` (default 2000) and TCP or UDP.

While the server runs, the status bar shows **↑ NMEA**: orange = no network yet,
grey = no device, green = connected devices (TCP clients; in AP mode also
devices that joined the network - pure UDP listeners in client mode can't be detected).
The WiFi page (after the signal page) shows network name, AP password or connection
status, address, port and connected devices. Deep sleep is disabled while the server is
on, and WiFi increases the power consumption considerably.

### Testing from a computer

Join the same WiFi as the Cardputer, then:

```sh
# TCP (use the address from the WiFi page in client mode)
nc 192.168.4.1 2000

# UDP broadcast (brew install socat)
socat -u UDP-RECV:2000 -
```

Don't use `nc -ul 2000` for UDP: macOS netcat ties its socket to the first sender and its
own address, so it drops the following broadcast packets after a few lines.

## Deep sleep

With `deep_sleep=1`, logging active and an interval of at least 15 s, the device goes
to deep sleep after `screen_timeout` seconds without input. It then wakes up by timer for
each point, reads the GPS (display stays off), writes the point and sleeps again. If no fix
is found within `fix_timeout` seconds, that point is skipped. Press **G0** to get the UI back.

The GPS module itself stays powered so it keeps a hot fix; it consumes far more power than
the sleeping ESP32 (typically 20-40 mA), so longer intervals save mainly the display and CPU
power. With intervals below 15 s only the screen is switched off.
