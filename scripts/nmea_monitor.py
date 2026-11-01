#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Shows position, altitude and speed from the GPS logger's USB serial NMEA output
(setting "NMEA serial" / nmea_serial=1), plus acceleration if calibrated.

    ./scripts/nmea_monitor.py [port]     # port default: first /dev/cu.usbmodem* or /dev/ttyACM*

Needs uv (https://docs.astral.sh/uv/), which installs pyserial on the first start.
"""
import glob
import sys

import serial


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No serial port found, give it as argument (e.g. COM5)")
    return ports[0]


def checksum_ok(line):
    if "*" not in line:
        return False
    body, cs = line[1:].rsplit("*", 1)
    x = 0
    for c in body:
        x ^= ord(c)
    return cs[:2].upper() == f"{x:02X}"


def degrees(value, hemi):
    if not value:
        return None
    dot = value.index(".")
    deg = float(value[:dot - 2]) + float(value[dot - 2:]) / 60
    return -deg if hemi in ("S", "W") else deg


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    state = {}
    with serial.Serial(port, 115200, timeout=1) as ser:
        print(f"Reading {port}, Ctrl+C ends\n")
        while True:
            line = ser.readline().decode("ascii", "replace").strip()
            if not line.startswith("$") or not checksum_ok(line):
                continue
            f = line.split("*")[0].split(",")
            kind = f[0][3:]
            if kind == "GGA" and len(f) > 9:
                state.update(time=f[1][:6], fix=f[6] != "0", sats=f[7],
                             lat=degrees(f[2], f[3]), lon=degrees(f[4], f[5]), alt=f[9])
            elif kind == "RMC" and len(f) > 7:
                state["kmh"] = float(f[7]) * 1.852 if f[7] else None
            elif kind == "XDR" and "ACCLON" in f:
                state["acc"] = {f[i + 3]: float(f[i + 1]) for i in range(1, len(f) - 3, 4)}
            elif kind != "GGA":
                continue
            show(state)


def show(s):
    t = s.get("time", "")
    t = f"{t[0:2]}:{t[2:4]}:{t[4:6]} UTC" if len(t) == 6 else "--"
    if s.get("fix") and s.get("lat") is not None:
        pos = f"{s['lat']:.6f}, {s['lon']:.6f}"
        alt = f"{s['alt']} m"
    else:
        pos, alt = "no fix", "--"
    kmh = s.get("kmh")
    lines = [
        f"Time      {t}   sats {s.get('sats', '--')}",
        f"Position  {pos}",
        f"Altitude  {alt}",
        f"Speed     {kmh:.1f} km/h" if kmh is not None else "Speed     --",
    ]
    acc = s.get("acc")
    if acc:
        lines.append(f"Accel     lon {acc.get('ACCLON', 0):+.2f} g  lat {acc.get('ACCLAT', 0):+.2f} g"
                     f"  vert {acc.get('ACCVERT', 0):+.2f} g")
    # Redraw in place: cursor up over the previous block, clear each line
    sys.stdout.write(f"\033[{show.height}F" if show.height else "")
    sys.stdout.write("".join(f"\033[2K{l}\n" for l in lines))
    sys.stdout.flush()
    show.height = len(lines)


show.height = 0

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
