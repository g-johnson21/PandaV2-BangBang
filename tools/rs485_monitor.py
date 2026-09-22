#!/usr/bin/env python3
"""
RS-485 receive monitor for PandaV2.

Opens the board's serial port and prints everything it sends, with a timestamp
and a line counter. Read-only: nothing is ever transmitted, so this is safe to
run against an armed stand.

Usage:
    python rs485_monitor.py                  # COM5 at 460800
    python rs485_monitor.py --port COM7
    python rs485_monitor.py --baud 115200
    python rs485_monitor.py --raw            # show bytes as they arrive, no line assembly
    python rs485_monitor.py --hex            # add a hex dump of every line

Dependencies:
    pip install pyserial

Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed — run: pip install pyserial")

DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 460800

# Seconds between reconnect attempts when the port is missing or drops.
RETRY_S = 2.0
# Blocking read timeout; also how often we notice Ctrl+C while idle.
READ_TIMEOUT_S = 0.2

# Inverted-signal detection. The firmware sets TXINV on both LPUARTs to cancel
# the V2 PCB's swapped Y/Z pair. If the receiving end does not invert as well,
# every byte is sampled off its true bit boundaries and arrives as garbage that
# cannot be recovered in software (a 18-character line comes back as 13 bytes).
# These byte values are what an inverted "p0.00000,..." telemetry row decodes
# to on a non-inverting receiver; several of them are impossible in this
# firmware's plain-ASCII output.
INVERSION_MARKERS = {0xF4, 0xA3, 0xF6, 0x9F, 0xEB, 0xD1, 0xDA, 0xA7, 0xED, 0xB4}
DETECT_AFTER_BYTES = 256


def printable(data: bytes) -> str:
    """Decode for display, showing non-text bytes as escapes rather than hiding them."""
    return data.decode("utf-8", errors="backslashreplace")


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class Monitor:
    def __init__(self, port: str, baud: int, raw: bool, show_hex: bool):
        self.port = port
        self.baud = baud
        self.raw = raw
        self.show_hex = show_hex
        self.lines = 0
        self.rx_bytes = 0
        self.buf = bytearray()
        self.started = time.monotonic()
        self.sample = bytearray()
        self.warned = False

    # ── output ──────────────────────────────────────────────────────────
    def stamp(self) -> str:
        return f"{time.monotonic() - self.started:9.3f}"

    def note(self, msg: str):
        print(f"[{self.stamp()}] [monitor] {msg}", flush=True)

    def emit_line(self, data: bytes):
        self.lines += 1
        text = printable(data)
        print(f"[{self.stamp()}] {self.lines:6d} | {text}", flush=True)
        if self.show_hex:
            print(f"{'':10}        | {hexdump(data)}", flush=True)

    def check_inversion(self, chunk: bytes):
        """Warn once if the stream looks like an inverted signal, not ASCII."""
        if self.warned or len(self.sample) >= DETECT_AFTER_BYTES:
            return
        self.sample.extend(chunk)
        if len(self.sample) < DETECT_AFTER_BYTES:
            return

        printable_ratio = sum(
            1 for b in self.sample if 32 <= b < 127 or b in (10, 13)
        ) / len(self.sample)
        marker_ratio = sum(1 for b in self.sample if b in INVERSION_MARKERS) / len(
            self.sample
        )
        if printable_ratio > 0.8 or marker_ratio < 0.15:
            self.warned = True
            return

        self.warned = True
        self.note("")
        self.note("*** This looks like an INVERTED signal, not ASCII telemetry.")
        self.note(f"    printable={printable_ratio:.0%}, inversion markers={marker_ratio:.0%}")
        self.note("    The firmware sets TXINV on both RS-485 buses to cancel the")
        self.note("    V2 PCB's swapped Y/Z pair. Your receiver is not inverting,")
        self.note("    so bytes are sampled off their true bit boundaries. This")
        self.note("    CANNOT be undone in software - the extra bits are gone.")
        self.note("    Fix either end:")
        self.note("      - swap A/B (or Y/Z) at the adapter: inverts the pair back, or")
        self.note("      - drop the LPUART*_CTRL |= LPUART_CTRL_TXINV lines in setup().")
        self.note("    Swapping the wires is the one to try first - no reflash.")
        self.note("")

    # ── framing ─────────────────────────────────────────────────────────
    def feed(self, chunk: bytes):
        """Split on CR/LF the way the firmware frames its output."""
        self.rx_bytes += len(chunk)
        self.check_inversion(chunk)
        if self.raw:
            sys.stdout.write(printable(chunk))
            sys.stdout.flush()
            return

        self.buf.extend(chunk)
        while True:
            idx = min(
                (i for i in (self.buf.find(b"\n"), self.buf.find(b"\r")) if i >= 0),
                default=-1,
            )
            if idx < 0:
                break
            line = bytes(self.buf[:idx])
            del self.buf[: idx + 1]
            if line:  # skip the empty half of a \r\n pair
                self.emit_line(line)

    def flush_partial(self):
        """Print whatever is buffered without a terminator, so nothing is lost."""
        if self.buf:
            self.emit_line(bytes(self.buf) + b"  <-- no line terminator")
            self.buf.clear()

    # ── main loop ───────────────────────────────────────────────────────
    def run(self):
        self.note(f"listening on {self.port} @ {self.baud} (Ctrl+C to stop)")
        while True:
            try:
                with serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_S) as ser:
                    self.note(f"connected to {ser.name}")
                    last_data = time.monotonic()
                    while True:
                        chunk = ser.read(4096)
                        if chunk:
                            self.feed(chunk)
                            last_data = time.monotonic()
                        else:
                            # Quiet line: flush a partial line so a board that
                            # stopped mid-message still shows what it sent.
                            if self.buf and time.monotonic() - last_data > 0.5:
                                self.flush_partial()
            except serial.SerialException as exc:
                self.flush_partial()
                self.note(f"{exc}")
                self.note(f"retrying in {RETRY_S:.0f}s")
                time.sleep(RETRY_S)


def main():
    ap = argparse.ArgumentParser(description="Print everything PandaV2 sends over RS-485")
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    ap.add_argument("--raw", action="store_true", help="stream bytes as they arrive instead of assembling lines")
    ap.add_argument("--hex", dest="show_hex", action="store_true", help="also hex-dump each line")
    args = ap.parse_args()

    mon = Monitor(args.port, args.baud, args.raw, args.show_hex)
    try:
        mon.run()
    except KeyboardInterrupt:
        mon.flush_partial()
        elapsed = time.monotonic() - mon.started
        mon.note(f"stopped — {mon.lines} lines, {mon.rx_bytes} bytes in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
