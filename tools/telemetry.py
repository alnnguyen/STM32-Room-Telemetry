#!/usr/bin/env python3
"""Log the board's UART readings to CSV, and plot a CSV.

    python3 tools/telemetry.py log            # auto-detects /dev/cu.usbmodem*
    python3 tools/telemetry.py log --port /dev/cu.usbmodem3103
    python3 tools/telemetry.py plot data/telemetry-20261028-141500.csv

Logging uses only the standard library (macOS/Linux). Plotting needs
matplotlib:  python3 -m pip install --user matplotlib

Only one program can open the serial port: quit `screen` first.
"""

import argparse
import csv
import datetime
import glob
import os
import re
import select
import sys
import termios

# Matches the firmware's report_sample() line, e.g.
#   Sample 12: T=71.81 F RH=54.62 % CRC=OK ALERT
SAMPLE_RE = re.compile(
    r"Sample (\d+): T=(-?\d+\.\d+) F RH=(\d+\.\d+) % CRC=OK( ALERT)?")


def open_serial(port):
    """Open the port raw at 115200 8N1 and return the file descriptor."""
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag: no input processing
    attrs[1] = 0  # oflag: no output processing
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0  # lflag: raw, no echo
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_lines(fd):
    """Yield complete text lines from the port as they arrive."""
    pending = b""
    while True:
        ready, _, _ = select.select([fd], [], [], 1.0)
        if not ready:
            continue
        chunk = os.read(fd, 256)
        if not chunk:
            continue
        pending += chunk
        while b"\n" in pending:
            line, pending = pending.split(b"\n", 1)
            yield line.decode("ascii", "replace").strip()


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("No /dev/cu.usbmodem* port found. Is the board plugged in?")
    return ports[0]


def cmd_log(args):
    port = args.port or find_port()
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(args.out_dir, f"telemetry-{stamp}.csv")

    try:
        fd = open_serial(port)
    except OSError as err:
        sys.exit(f"Could not open {port}: {err}\n"
                 "If it says 'Resource busy', quit screen (Ctrl-A, K, Y).")

    print(f"Logging {port} -> {path}  (Ctrl-C to stop)")
    rows = 0
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "sample", "temp_f", "humidity_pct",
                         "alert"])
        try:
            for line in read_lines(fd):
                match = SAMPLE_RE.search(line)
                if not match:
                    if line.startswith("Sample"):
                        print(f"  (skipped: {line})")  # e.g. a sensor error
                    continue
                now = datetime.datetime.now().isoformat(timespec="seconds")
                sample, temp, rh, alert = match.groups()
                writer.writerow([now, sample, temp, rh, 1 if alert else 0])
                f.flush()  # keep the file current if the script is killed
                rows += 1
                print(f"{now}  #{sample}  {temp} F  {rh} %"
                      f"{'  ALERT' if alert else ''}")
        except KeyboardInterrupt:
            pass
        finally:
            os.close(fd)
    print(f"\nSaved {rows} readings to {path}")
    if rows:
        print(f"Plot it with:  python3 tools/telemetry.py plot {path}")


def cmd_plot(args):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.dates as mdates
    except ImportError:
        sys.exit("Plotting needs matplotlib:\n"
                 "  python3 -m pip install --user matplotlib")

    times, temps, hums = [], [], []
    with open(args.csv) as f:
        for row in csv.DictReader(f):
            times.append(datetime.datetime.fromisoformat(row["timestamp"]))
            temps.append(float(row["temp_f"]))
            hums.append(float(row["humidity_pct"]))
    if not times:
        sys.exit(f"{args.csv} has no readings")

    fig, (ax_t, ax_h) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax_t.plot(times, temps, color="tab:red")
    ax_t.set_ylabel("Temperature (°F)")
    ax_t.grid(True, alpha=0.3)
    ax_h.plot(times, hums, color="tab:blue")
    ax_h.set_ylabel("Humidity (%RH)")
    ax_h.grid(True, alpha=0.3)
    ax_h.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    fig.suptitle(f"Room telemetry: {os.path.basename(args.csv)} "
                 f"({len(times)} readings)")
    fig.tight_layout()

    png = os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(png, dpi=120)
    print(f"Saved {png}")
    if not args.no_show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    log = sub.add_parser("log", help="record readings to a CSV file")
    log.add_argument("--port", help="serial port (default: auto-detect)")
    log.add_argument("--out-dir", default="data",
                     help="folder for CSV files (default: data)")
    log.set_defaults(func=cmd_log)

    plot = sub.add_parser("plot", help="plot a CSV file and save a PNG")
    plot.add_argument("csv")
    plot.add_argument("--no-show", action="store_true",
                      help="only save the PNG, don't open a window")
    plot.set_defaults(func=cmd_plot)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
