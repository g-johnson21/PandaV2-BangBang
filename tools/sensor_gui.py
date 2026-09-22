#!/usr/bin/env python3
"""
PandaV2 Sensor Stream GUI
All sensor groups on one scrollable page, serial terminal below.
Runs without a Teensy — shows error banner and retries every 5 s.

Usage:
    python sensor_gui.py                        # auto-detect port
    python sensor_gui.py --port COM3
    python sensor_gui.py --port COM3 --baud 460800

Dependencies:
    pip install pyserial
    (tkinter ships with standard Python on Windows/macOS/Linux)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk
from typing import Optional
import serial
import serial.tools.list_ports

# ── Calibration file ─────────────────────────────────────────────────────────

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibrations.json")

NUM_PT = 16

def _default_pt_calib() -> list[dict]:
    return [{"scale": 1.0, "offset": 0.0} for _ in range(NUM_PT)]

def load_calibrations() -> list[dict]:
    """Load per-channel PT calibrations from disk. Returns defaults on any error."""
    try:
        with open(CALIB_FILE, "r") as f:
            data = json.load(f)

        pt_result = _default_pt_calib()
        for i, entry in enumerate(data.get("pt", [])[:NUM_PT]):
            pt_result[i]["scale"]  = float(entry.get("scale",  1.0))
            pt_result[i]["offset"] = float(entry.get("offset", 0.0))

        return pt_result
    except Exception:
        return _default_pt_calib()

def save_calibrations(pt_calib: list[dict]):
    """Persist per-channel PT calibrations to disk."""
    data = {
        "pt": [{"scale": c["scale"], "offset": c["offset"]} for c in pt_calib],
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(data, f, indent=2)

# ── Conversion helpers ───────────────────────────────────────────────────────

# PT: per-channel, loaded from file at startup and updated by the calibration panel
_pt_calib = load_calibrations()

LC_SCALE      = 1.0
LC_OFFSET     = 0.0
CURRENT_SCALE = 1.0 / 20.0

def convert_pt(v: float, ch: int) -> float:
    c = _pt_calib[ch]
    return (v - c["offset"]) * c["scale"]

def convert_lc(v: float) -> float:
    return (v - LC_OFFSET) * LC_SCALE

def convert_cur(v: float) -> float:
    return v * CURRENT_SCALE

# ── Configuration ────────────────────────────────────────────────────────────

BAUD          = 460800
TIMEOUT_S     = 2.0
RECONNECT_S   = 5.0
TERMINAL_ROWS = 200
UPDATE_MS     = 100

# Colour palette
BG       = "#1e1e2e"
BG_ALT   = "#181825"
BG_PANEL = "#11111b"
BG_ERR   = "#311324"
BG_GRP   = "#1e1e2e"
BG_CALIB = "#1a1a2e"
FG       = "#cdd6f4"
FG_DIM   = "#6c7086"
FG_HEAD  = "#89b4fa"
FG_GRP   = "#cba6f7"
FG_GOOD  = "#a6e3a1"
FG_WARN  = "#f38ba8"
FG_TERM  = "#a6e3a1"
FG_CALIB = "#fab387"
BTN_BG   = "#313244"
BTN_ACT  = "#45475a"

# ── Connection state ─────────────────────────────────────────────────────────

class ConnState:
    DISCONNECTED = "disconnected"
    CONNECTED    = "connected"
    ERROR        = "error"

# ── Data model ───────────────────────────────────────────────────────────────

class ChannelState:
    __slots__ = ("raw", "converted", "ok")
    def __init__(self):
        self.raw       = math.nan
        self.converted = math.nan
        self.ok        = False

class AppState:
    def __init__(self):
        self._lock      = threading.Lock()
        self.pt         = [ChannelState() for _ in range(16)]
        self.lc         = [ChannelState() for _ in range(8)]
        self.cur        = [ChannelState() for _ in range(16)]
        self.board_temp: float         = math.nan
        self.t_ms:       Optional[int]  = None
        self.err:        int            = 0
        self.conn:       str            = ConnState.DISCONNECTED
        self.status:     str            = "Not connected"
        self.next_retry: Optional[float] = None
        self._term_lines: list[str]     = []
        self._term_dirty: bool          = False

    def ingest(self, frame: dict, raw_line: str):
        with self._lock:
            self.t_ms = frame.get("t")
            self.err  = frame.get("err", 0)
            brd       = frame.get("brd")
            self.board_temp = brd if brd is not None else math.nan

            def _load(group, key, converter):
                vals = frame.get(key, [])
                for i, ch in enumerate(group):
                    v = vals[i] if i < len(vals) else None
                    if v is None or v != v:
                        ch.raw = ch.converted = math.nan
                        ch.ok = False
                    else:
                        ch.raw = v
                        ch.converted = converter(v)
                        ch.ok = True

            # PT uses per-channel converter
            pt_vals = frame.get("pt", [])
            for i, ch in enumerate(self.pt):
                v = pt_vals[i] if i < len(pt_vals) else None
                if v is None or v != v:
                    ch.raw = ch.converted = math.nan
                    ch.ok = False
                else:
                    ch.raw = v
                    ch.converted = convert_pt(v, i)
                    ch.ok = True

            _load(self.lc,  "lc",  convert_lc)
            _load(self.cur, "cur", convert_cur)


            self._push_term(raw_line)

    def get_pt_raw(self, ch: int) -> float:
        """Thread-safe read of a single PT raw voltage."""
        with self._lock:
            return self.pt[ch].raw

    def get_all_pt_raw(self) -> list[float]:
        """Thread-safe snapshot of all PT raw voltages."""
        with self._lock:
            return [c.raw for c in self.pt]

    def reapply_pt_calib(self):
        """Recompute converted values for all PT channels using current _pt_calib."""
        with self._lock:
            for i, ch in enumerate(self.pt):
                if not math.isnan(ch.raw):
                    ch.converted = convert_pt(ch.raw, i)

    def set_conn(self, state: str, msg: str, next_retry: Optional[float] = None,
                 term_line: Optional[str] = None):
        with self._lock:
            self.conn       = state
            self.status     = msg
            self.next_retry = next_retry
            if term_line:
                self._push_term(term_line)

    def push_raw_line(self, line: str):
        with self._lock:
            self._push_term(line)

    def _push_term(self, line: str):
        self._term_lines.append(line)
        if len(self._term_lines) > TERMINAL_ROWS:
            self._term_lines = self._term_lines[-TERMINAL_ROWS:]
        self._term_dirty = True

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "pt":         [(c.raw, c.converted, c.ok) for c in self.pt],
                "lc":         [(c.raw, c.converted, c.ok) for c in self.lc],
                "cur":        [(c.raw, c.converted, c.ok) for c in self.cur],
                "brd":        self.board_temp,
                "t_ms":       self.t_ms,
                "err":        self.err,
                "conn":       self.conn,
                "status":     self.status,
                "next_retry": self.next_retry,
            }

    def take_term_lines(self) -> Optional[list[str]]:
        with self._lock:
            if not self._term_dirty:
                return None
            self._term_dirty = False
            return list(self._term_lines)

# ── Port detection ────────────────────────────────────────────────────────────

def find_port(preferred: Optional[str] = None) -> Optional[str]:
    all_ports = serial.tools.list_ports.comports()
    if preferred:
        return preferred if any(p.device == preferred for p in all_ports) else None
    candidates = [
        p.device for p in all_ports
        if "teensy"     in (p.description or "").lower()
        or "usb serial" in (p.description or "").lower()
        or "usbmodem"   in p.device.lower()
        or "ttyACM"     in p.device
    ]
    if candidates:
        return candidates[0]
    return all_ports[0].device if all_ports else None

# ── Serial reader ─────────────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    def __init__(self, preferred_port: Optional[str], baud: int, state: AppState):
        super().__init__(daemon=True, name="SerialReader")
        self._preferred = preferred_port
        self._baud      = baud
        self._state     = state
        self._stop      = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            port = find_port(self._preferred)
            if port is None:
                retry_at = time.monotonic() + RECONNECT_S
                self._state.set_conn(ConnState.DISCONNECTED,
                                     "No device found — retrying in 5 s…",
                                     next_retry=retry_at)
                self._wait(RECONNECT_S)
                continue
            try:
                with serial.Serial(port, self._baud, timeout=TIMEOUT_S) as ser:
                    self._state.set_conn(ConnState.CONNECTED,
                                         f"Connected — {port} @ {self._baud}",
                                         term_line=f"[gui] connected to {port}")
                    ser.reset_input_buffer()
                    for raw_bytes in ser:
                        if self._stop.is_set():
                            return
                        line = raw_bytes.decode("ascii", errors="replace").rstrip()
                        if not line:
                            continue
                        try:
                            frame = json.loads(line)
                        except json.JSONDecodeError:
                            self._state.push_raw_line(line)
                            continue
                        if "status" in frame and "t" not in frame:
                            self._state.set_conn(ConnState.CONNECTED,
                                                 frame["status"], term_line=line)
                        else:
                            self._state.ingest(frame, line)
            except serial.SerialException as exc:
                retry_at = time.monotonic() + RECONNECT_S
                self._state.set_conn(ConnState.ERROR,
                                     f"Connection lost — retrying in 5 s  ({exc})",
                                     next_retry=retry_at,
                                     term_line=f"[gui] serial error: {exc}")
                self._wait(RECONNECT_S)

    def _wait(self, seconds: float):
        deadline = time.monotonic() + seconds
        while not self._stop.is_set() and time.monotonic() < deadline:
            time.sleep(0.1)

# ── Sensor table widget ───────────────────────────────────────────────────────

class SensorTable(tk.Frame):
    """Group header + one row per channel: CH | Raw | Converted."""

    def __init__(self, parent, title: str, channels: int,
                 raw_unit: str, conv_unit: str, ch_prefix: str, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)

        hdr_font  = tkfont.Font(family="Consolas", size=8, weight="bold")
        grp_font  = tkfont.Font(family="Consolas", size=9, weight="bold")
        row_font  = tkfont.Font(family="Consolas", size=9)

        tk.Label(self, text=title, bg=BG_GRP, fg=FG_GRP, font=grp_font,
                 anchor="w", padx=6, pady=3).grid(
                     row=0, column=0, columnspan=3, sticky="ew")

        for col, (text, w) in enumerate([
            ("CH", 5), (f"Raw ({raw_unit})", 13), (f"Conv ({conv_unit})", 13)
        ]):
            tk.Label(self, text=text, bg=BG_ALT, fg=FG_HEAD, font=hdr_font,
                     width=w, anchor="center").grid(
                         row=1, column=col, sticky="nsew", padx=1, pady=1)

        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)

        self._raw_vars  = []
        self._conv_vars = []
        self._raw_lbls  = []
        self._conv_lbls = []

        for i in range(channels):
            bg = BG_PANEL if i % 2 == 0 else BG_ALT
            rv = tk.StringVar(value="—")
            cv = tk.StringVar(value="—")
            self._raw_vars.append(rv)
            self._conv_vars.append(cv)

            tk.Label(self, text=f"{ch_prefix}{i}", bg=bg, fg=FG_DIM, font=row_font,
                     width=5, anchor="center").grid(row=i+2, column=0, sticky="nsew", padx=1)

            rl = tk.Label(self, textvariable=rv, bg=bg, fg=FG, font=row_font,
                          width=13, anchor="e", padx=4)
            rl.grid(row=i+2, column=1, sticky="nsew", padx=1)
            self._raw_lbls.append(rl)

            cl = tk.Label(self, textvariable=cv, bg=bg, fg=FG, font=row_font,
                          width=13, anchor="e", padx=4)
            cl.grid(row=i+2, column=2, sticky="nsew", padx=1)
            self._conv_lbls.append(cl)

    def update_row(self, i: int, raw: float, conv: float, ok: bool):
        fg = FG_GOOD if ok else FG_WARN
        if math.isnan(raw):
            self._raw_vars[i].set("NaN")
            self._conv_vars[i].set("NaN")
        else:
            self._raw_vars[i].set(f"{raw:>12.5f}")
            self._conv_vars[i].set(f"{conv:>12.4f}")
        self._raw_lbls[i].config(fg=fg)
        self._conv_lbls[i].config(fg=fg)

# ── Calibration panel ─────────────────────────────────────────────────────────

class CalibrationPanel(tk.Frame):
    """
    PT calibration workflow:
      1. Select channel (PT0–PT15 or All)
      2. Enter PT range (informational)
      3. Reading 1 — enter known PSI (default 0), take voltage snapshot
      4. Reading 2 — enter known PSI, take voltage snapshot
         → scale/offset computed from these two points
      5. Apply & Save — updates live conversion and writes sensor_calibrations.json
      6. Reading 3 (verification) — enter known PSI, take voltage snapshot
         → residual (measured − expected) displayed per channel
    """

    _CH_OPTIONS = ["All"] + [f"PT{i}" for i in range(NUM_PT)]

    def __init__(self, parent, state: AppState, on_calib_applied, **kwargs):
        super().__init__(parent, bg=BG_CALIB, **kwargs)
        self._state    = state
        self._on_apply = on_calib_applied

        # Per-channel voltage snapshots; None = not yet taken
        self._r1: list[Optional[float]] = [None] * NUM_PT
        self._r2: list[Optional[float]] = [None] * NUM_PT
        self._r3: list[Optional[float]] = [None] * NUM_PT

        mono9  = tkfont.Font(family="Consolas", size=9)
        mono9b = tkfont.Font(family="Consolas", size=9, weight="bold")
        mono8  = tkfont.Font(family="Consolas", size=8)

        # ── Title ─────────────────────────────────────────────────────
        tk.Label(self, text="Pressure Transducer Calibration",
                 bg=BG_CALIB, fg=FG_CALIB, font=mono9b,
                 anchor="w", padx=8, pady=6).pack(fill="x")
        tk.Frame(self, bg=FG_DIM, height=1).pack(fill="x", padx=8)

        # ── Top controls: channel + PT range ─────────────────────────
        ctrl = tk.Frame(self, bg=BG_CALIB)
        ctrl.pack(fill="x", padx=8, pady=8)

        tk.Label(ctrl, text="Channel:", bg=BG_CALIB, fg=FG, font=mono9).grid(
            row=0, column=0, sticky="w", padx=(0, 4))
        self._ch_var = tk.StringVar(value="All")
        ch_menu = ttk.Combobox(ctrl, textvariable=self._ch_var,
                               values=self._CH_OPTIONS, state="readonly", width=8,
                               font=mono9)
        ch_menu.grid(row=0, column=1, sticky="w", padx=(0, 24))
        ch_menu.bind("<<ComboboxSelected>>", self._on_channel_change)

        tk.Label(ctrl, text="PT range (PSI):", bg=BG_CALIB, fg=FG, font=mono9).grid(
            row=0, column=2, sticky="w", padx=(0, 4))
        self._range_var = tk.StringVar(value="")
        tk.Entry(ctrl, textvariable=self._range_var, width=10,
                 bg=BTN_BG, fg=FG, insertbackground=FG, font=mono9,
                 relief="flat").grid(row=0, column=3, sticky="w")

        # ── Three reading rows ────────────────────────────────────────
        readings_frame = tk.Frame(self, bg=BG_CALIB)
        readings_frame.pack(fill="x", padx=8, pady=(0, 6))

        def _reading_row(parent, row, label, psi_var, btn_text, btn_cmd, note=""):
            tk.Label(parent, text=label, bg=BG_CALIB, fg=FG_CALIB, font=mono9b,
                     width=12, anchor="w").grid(row=row, column=0, sticky="w", pady=3)
            tk.Label(parent, text="Known PSI:", bg=BG_CALIB, fg=FG, font=mono9).grid(
                row=row, column=1, sticky="w", padx=(8, 4))
            tk.Entry(parent, textvariable=psi_var, width=10,
                     bg=BTN_BG, fg=FG, insertbackground=FG, font=mono9,
                     relief="flat").grid(row=row, column=2, sticky="w", padx=(0, 12))
            tk.Button(parent, text=btn_text,
                      bg=BTN_BG, fg=FG, activebackground=BTN_ACT, activeforeground=FG,
                      font=mono9, relief="flat", padx=10, pady=3,
                      command=btn_cmd).grid(row=row, column=3, sticky="w", padx=(0, 8))
            if note:
                tk.Label(parent, text=note, bg=BG_CALIB, fg=FG_DIM, font=mono8).grid(
                    row=row, column=4, sticky="w")

        self._psi1_var = tk.StringVar(value="0")
        self._psi2_var = tk.StringVar(value="")
        self._psi3_var = tk.StringVar(value="")

        _reading_row(readings_frame, 0, "Reading 1",
                     self._psi1_var, "Take Reading 1", self._take_r1,
                     note="zero / reference point")
        _reading_row(readings_frame, 1, "Reading 2",
                     self._psi2_var, "Take Reading 2", self._take_r2,
                     note="calibration span point")
        _reading_row(readings_frame, 2, "Reading 3",
                     self._psi3_var, "Take Reading 3", self._take_r3,
                     note="verification (take after Apply)")

        # ── Action buttons ────────────────────────────────────────────
        btn_row = tk.Frame(self, bg=BG_CALIB)
        btn_row.pack(fill="x", padx=8, pady=(2, 6))

        tk.Button(btn_row, text="Apply & Save",
                  bg="#2d4a2d", fg=FG_GOOD,
                  activebackground="#3a5e3a", activeforeground=FG_GOOD,
                  font=mono9b, relief="flat", padx=12, pady=4,
                  command=self._apply).pack(side="left", padx=(0, 8))

        tk.Button(btn_row, text="Reset All Readings",
                  bg=BTN_BG, fg=FG_DIM, activebackground=BTN_ACT, activeforeground=FG,
                  font=mono9, relief="flat", padx=12, pady=4,
                  command=self._reset_readings).pack(side="left")

        # ── Status message ────────────────────────────────────────────
        self._msg_var = tk.StringVar(value="Enter known pressures and take readings 1 & 2, then Apply.")
        tk.Label(self, textvariable=self._msg_var,
                 bg=BG_CALIB, fg=FG_DIM, font=mono8,
                 anchor="w", padx=8).pack(fill="x")

        tk.Frame(self, bg=FG_DIM, height=1).pack(fill="x", padx=8, pady=(4, 0))

        # ── Results table ─────────────────────────────────────────────
        tbl_outer = tk.Frame(self, bg=BG_CALIB)
        tbl_outer.pack(fill="both", expand=True, padx=8, pady=8)

        self._tbl_canvas = tk.Canvas(tbl_outer, bg=BG_CALIB,
                                     highlightthickness=0, bd=0)
        tbl_vsb = tk.Scrollbar(tbl_outer, orient="vertical",
                                command=self._tbl_canvas.yview, bg=BG_ALT)
        self._tbl_canvas.configure(yscrollcommand=tbl_vsb.set)
        tbl_vsb.pack(side="right", fill="y")
        self._tbl_canvas.pack(side="left", fill="both", expand=True)

        self._tbl_inner = tk.Frame(self._tbl_canvas, bg=BG_CALIB)
        self._tbl_win = self._tbl_canvas.create_window(
            (0, 0), window=self._tbl_inner, anchor="nw")
        self._tbl_inner.bind("<Configure>", lambda _: self._tbl_canvas.configure(
            scrollregion=self._tbl_canvas.bbox("all")))
        self._tbl_canvas.bind("<Configure>", lambda e: self._tbl_canvas.itemconfig(
            self._tbl_win, width=e.width))

        # Columns: CH | R1(V) | R2(V) | Scale | Offset | R3(V) | Residual(PSI) | Status
        hdrs = [("CH", 6), ("R1 (V)", 13), ("R2 (V)", 13),
                ("Scale", 13), ("Offset", 13),
                ("R3 (V)", 13), ("Residual (PSI)", 15), ("Status", 16)]
        for col, (txt, w) in enumerate(hdrs):
            tk.Label(self._tbl_inner, text=txt, bg=BG_ALT, fg=FG_HEAD,
                     font=mono9b, width=w, anchor="center").grid(
                         row=0, column=col, sticky="nsew", padx=1, pady=1)

        self._row_vars: list[dict] = []
        self._residual_lbls: list[tk.Label] = []
        for i in range(NUM_PT):
            bg = BG_CALIB if i % 2 == 0 else BG_ALT
            rv = {k: tk.StringVar(value="—")
                  for k in ("r1", "r2", "scale", "offset", "r3", "residual", "status")}
            self._row_vars.append(rv)

            tk.Label(self._tbl_inner, text=f"PT{i}", bg=bg, fg=FG_DIM,
                     font=mono9, width=6, anchor="center").grid(
                         row=i+1, column=0, sticky="nsew", padx=1)
            for col, key in enumerate(("r1", "r2", "scale", "offset", "r3"), start=1):
                w = hdrs[col][1]
                tk.Label(self._tbl_inner, textvariable=rv[key],
                         bg=bg, fg=FG, font=mono9, width=w, anchor="e", padx=4).grid(
                             row=i+1, column=col, sticky="nsew", padx=1)

            # Residual gets its own label reference so we can colour it
            res_lbl = tk.Label(self._tbl_inner, textvariable=rv["residual"],
                               bg=bg, fg=FG, font=mono9, width=15, anchor="e", padx=4)
            res_lbl.grid(row=i+1, column=6, sticky="nsew", padx=1)
            self._residual_lbls.append(res_lbl)

            tk.Label(self._tbl_inner, textvariable=rv["status"],
                     bg=bg, fg=FG_DIM, font=mono9, width=16, anchor="w", padx=4).grid(
                         row=i+1, column=7, sticky="nsew", padx=1)

        self._refresh_table()

    # ── Internal helpers ─────────────────────────────────────────────

    def _selected_channels(self) -> list[int]:
        sel = self._ch_var.get()
        if sel == "All":
            return list(range(NUM_PT))
        return [int(sel[2:])]  # "PT3" → 3

    def _on_channel_change(self, _event=None):
        self._refresh_table()

    def _snap(self, store: list, label: str, next_hint: str):
        raws = self._state.get_all_pt_raw()
        chs  = self._selected_channels()
        for ch in chs:
            v = raws[ch]
            store[ch] = None if math.isnan(v) else v
        self._msg_var.set(f"{label} captured for {len(chs)} channel(s).  {next_hint}")
        self._refresh_table()

    def _take_r1(self):
        self._snap(self._r1, "Reading 1", "Now take Reading 2 at the span pressure.")

    def _take_r2(self):
        self._snap(self._r2, "Reading 2", "Click Apply & Save, then take Reading 3 to verify.")

    def _take_r3(self):
        self._snap(self._r3, "Reading 3", "Residuals shown in table.")

    def _apply(self):
        global _pt_calib
        try:
            psi1 = float(self._psi1_var.get())
        except ValueError:
            self._msg_var.set("Enter a valid numeric value for Reading 1 PSI.")
            return
        try:
            psi2 = float(self._psi2_var.get())
        except ValueError:
            self._msg_var.set("Enter a valid numeric value for Reading 2 PSI.")
            return
        if psi1 == psi2:
            self._msg_var.set("Reading 1 and Reading 2 PSI values must differ.")
            return

        chs     = self._selected_channels()
        updated = 0
        for ch in chs:
            r1 = self._r1[ch]
            r2 = self._r2[ch]
            if r1 is None or r2 is None:
                continue
            dv = r2 - r1
            if abs(dv) < 1e-9:
                continue
            # Two-point linear fit: PSI = scale * (V - offset)
            scale  = (psi2 - psi1) / dv
            offset = r1 - psi1 / scale
            _pt_calib[ch]["scale"]  = scale
            _pt_calib[ch]["offset"] = offset
            updated += 1

        if updated == 0:
            self._msg_var.set("No valid channel pairs found — check that readings were taken.")
            return

        save_calibrations(_pt_calib)
        self._state.reapply_pt_calib()
        self._on_apply()
        self._msg_var.set(
            f"Applied and saved calibration for {updated} channel(s).  "
            "Take Reading 3 to verify.")
        self._refresh_table()

    def _reset_readings(self):
        chs = self._selected_channels()
        for ch in chs:
            self._r1[ch] = None
            self._r2[ch] = None
            self._r3[ch] = None
        self._msg_var.set("Readings cleared. Ready to start again.")
        self._refresh_table()

    def _refresh_table(self):
        chs = set(self._selected_channels())
        try:
            psi3 = float(self._psi3_var.get())
        except (ValueError, AttributeError):
            psi3 = None

        for i in range(NUM_PT):
            rv  = self._row_vars[i]
            r1  = self._r1[i]
            r2  = self._r2[i]
            r3  = self._r3[i]
            cal = _pt_calib[i]

            rv["r1"].set(f"{r1:.6f}" if r1 is not None else "—")
            rv["r2"].set(f"{r2:.6f}" if r2 is not None else "—")
            rv["scale"].set(f"{cal['scale']:.5f}")
            rv["offset"].set(f"{cal['offset']:.6f}")
            rv["r3"].set(f"{r3:.6f}" if r3 is not None else "—")

            # Residual: actual converted value of R3 minus the known PSI at R3
            if r3 is not None and psi3 is not None:
                measured = convert_pt(r3, i)
                residual = measured - psi3
                rv["residual"].set(f"{residual:+.4f}")
                self._residual_lbls[i].config(
                    fg=FG_WARN if abs(residual) > 1.0 else FG_GOOD)
            else:
                rv["residual"].set("—")
                self._residual_lbls[i].config(fg=FG)

            if i not in chs:
                rv["status"].set("—")
            elif r1 is None:
                rv["status"].set("awaiting R1")
            elif r2 is None:
                rv["status"].set("awaiting R2")
            else:
                dv = r2 - r1
                if abs(dv) < 1e-9:
                    rv["status"].set("no Δ voltage")
                elif r3 is None:
                    rv["status"].set("ready / no verify")
                else:
                    rv["status"].set("verified")

# ── Main application ──────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self, preferred_port: Optional[str], baud: int):
        super().__init__()
        self.title("PandaV2 Sensor Monitor")
        self.configure(bg=BG)
        self.geometry("1400x900")
        self.minsize(900, 600)

        self._state  = AppState()
        self._reader = SerialReader(preferred_port, baud, self._state)

        self._build_ui()
        self._reader.start()
        self._schedule_update()

    # ── UI construction ──────────────────────────────────────────────

    def _build_ui(self):
        mono8  = tkfont.Font(family="Consolas", size=8)
        head12 = tkfont.Font(family="Consolas", size=12, weight="bold")

        # ── Top bar ───────────────────────────────────────────────────
        top = tk.Frame(self, bg=BG_ALT, height=32)
        top.pack(fill="x", side="top")
        tk.Label(top, text="PandaV2  Sensor Monitor", bg=BG_ALT, fg=FG_HEAD,
                 font=head12, padx=10).pack(side="left", pady=4)
        self._time_var = tk.StringVar(value="")
        tk.Label(top, textvariable=self._time_var, bg=BG_ALT, fg=FG_DIM,
                 font=mono8, padx=10).pack(side="right", pady=4)
        self._status_var = tk.StringVar(value="Connecting…")
        tk.Label(top, textvariable=self._status_var, bg=BG_ALT, fg=FG_DIM,
                 font=mono8, padx=10).pack(side="right", pady=4)
        tk.Label(top, text="Board:", bg=BG_ALT, fg=FG_DIM,
                 font=mono8).pack(side="right", pady=4, padx=(10, 2))
        self._brd_hdr_var = tk.StringVar(value="— °C")
        self._brd_hdr_lbl = tk.Label(top, textvariable=self._brd_hdr_var,
                                     bg=BG_ALT, fg=FG_DIM, font=mono8)
        self._brd_hdr_lbl.pack(side="right", pady=4, padx=(0, 8))

        # ── Error banner ──────────────────────────────────────────────
        self._banner = tk.Frame(self, bg=BG_ERR)
        tk.Label(self._banner, text="⚠", bg=BG_ERR, fg=FG_WARN,
                 font=tkfont.Font(family="Consolas", size=11, weight="bold"),
                 padx=8).pack(side="left")
        self._banner_msg = tk.Label(self._banner, text="", bg=BG_ERR, fg=FG_WARN,
                                    font=mono8, anchor="w")
        self._banner_msg.pack(side="left", fill="x", expand=True)
        self._banner_countdown = tk.Label(self._banner, text="", bg=BG_ERR, fg=FG_DIM,
                                          font=mono8, padx=10)
        self._banner_countdown.pack(side="right")
        self._banner.pack(fill="x", side="top")
        self._banner_visible = True

        # ── Tab notebook ──────────────────────────────────────────────
        style = ttk.Style(self)
        style.theme_use("default")
        style.configure("Dark.TNotebook",
                        background=BG, borderwidth=0, tabmargins=0)
        style.configure("Dark.TNotebook.Tab",
                        background=BG_ALT, foreground=FG_DIM,
                        padding=[12, 4], font=("Consolas", 9))
        style.map("Dark.TNotebook.Tab",
                  background=[("selected", BG)],
                  foreground=[("selected", FG_HEAD)])

        notebook = ttk.Notebook(self, style="Dark.TNotebook")
        notebook.pack(fill="both", expand=True, padx=4, pady=(4, 4))

        # ── Monitor tab ───────────────────────────────────────────────
        monitor_tab = tk.Frame(notebook, bg=BG)
        notebook.add(monitor_tab, text="  Monitor  ")
        self._build_monitor_tab(monitor_tab)

        # ── PT Calibration tab ────────────────────────────────────────
        calib_tab = tk.Frame(notebook, bg=BG_CALIB)
        notebook.add(calib_tab, text="  PT Calibration  ")
        self._calib_panel = CalibrationPanel(
            calib_tab, self._state,
            on_calib_applied=self._on_calib_applied)
        self._calib_panel.pack(fill="both", expand=True)

    def _build_monitor_tab(self, parent: tk.Frame):
        """Sensor grid + terminal, as before."""
        # ── Vertical pane: sensor area (top) + terminal (bottom) ──────
        paned = tk.PanedWindow(parent, orient="vertical", bg=BG,
                               sashwidth=6, sashrelief="flat",
                               sashpad=2, opaqueresize=True)
        paned.pack(fill="both", expand=True)

        # ── Scrollable sensor area ────────────────────────────────────
        sensor_outer = tk.Frame(paned, bg=BG)
        paned.add(sensor_outer, stretch="always", minsize=300)

        self._canvas = tk.Canvas(sensor_outer, bg=BG, highlightthickness=0, bd=0)
        vsb = tk.Scrollbar(sensor_outer, orient="vertical",
                           command=self._canvas.yview, bg=BG_ALT)
        self._canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)

        self._sensor_inner = tk.Frame(self._canvas, bg=BG)
        self._canvas_win = self._canvas.create_window(
            (0, 0), window=self._sensor_inner, anchor="nw")

        self._sensor_inner.bind("<Configure>", self._on_inner_resize)
        self._canvas.bind("<Configure>", self._on_canvas_resize)
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self._canvas.bind(seq, self._on_canvas_scroll)
            self._sensor_inner.bind(seq, self._on_canvas_scroll)

        self._build_sensor_grid()

        # ── Terminal ──────────────────────────────────────────────────
        mono8 = tkfont.Font(family="Consolas", size=8)
        term_frame = tk.Frame(paned, bg=BG_PANEL)
        paned.add(term_frame, stretch="never", minsize=120)

        tk.Label(term_frame, text="Serial Terminal", bg=BG_ALT, fg=FG_HEAD,
                 font=mono8, anchor="w", padx=6).pack(fill="x")

        self._term = tk.Text(
            term_frame, bg=BG_PANEL, fg=FG_TERM,
            font=("Consolas", 8), state="disabled",
            wrap="none", relief="flat", borderwidth=0,
            highlightthickness=0, insertbackground=FG,
        )
        tsb_y = tk.Scrollbar(term_frame, orient="vertical",
                              command=self._term.yview, bg=BG_ALT)
        tsb_x = tk.Scrollbar(term_frame, orient="horizontal",
                              command=self._term.xview, bg=BG_ALT)
        self._term.configure(yscrollcommand=tsb_y.set, xscrollcommand=tsb_x.set)
        tsb_y.pack(side="right",  fill="y")
        tsb_x.pack(side="bottom", fill="x")
        self._term.pack(fill="both", expand=True)
        self._term_autoscroll = True
        self._term.bind("<MouseWheel>", self._on_term_scroll)
        self._term.bind("<Button-4>",   self._on_term_scroll)
        self._term.bind("<Button-5>",   self._on_term_scroll)

        self.update_idletasks()
        paned.sash_place(0, 0, int(self.winfo_height() * 0.70))

    def _build_sensor_grid(self):
        pad = {"padx": 6, "pady": 6}

        for c in range(2):
            self._sensor_inner.columnconfigure(c, weight=1, uniform="col")

        self._pt_tbl = SensorTable(
            self._sensor_inner,
            title="Pressure  (×16)",
            channels=16, raw_unit="V", conv_unit="PSI", ch_prefix="PT")
        self._pt_tbl.grid(row=0, column=0, sticky="new", **pad)

        self._cur_tbl = SensorTable(
            self._sensor_inner,
            title="Current  (×16)",
            channels=16, raw_unit="V", conv_unit="A", ch_prefix="CUR")
        self._cur_tbl.grid(row=0, column=1, sticky="new", **pad)

        for tbl in (self._pt_tbl, self._cur_tbl):
            for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                tbl.bind(seq, self._on_canvas_scroll)

    # ── Calibration callback ─────────────────────────────────────────

    def _on_calib_applied(self):
        """Called by CalibrationPanel after a successful apply — nothing extra needed;
        reapply_pt_calib() already updated AppState and the next _update() tick
        will push fresh converted values to the sensor table."""
        pass

    # ── Canvas scroll helpers ────────────────────────────────────────

    def _on_inner_resize(self, _event):
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    def _on_canvas_resize(self, event):
        self._canvas.itemconfig(self._canvas_win, width=event.width)

    def _on_canvas_scroll(self, event):
        self._canvas.yview_scroll(
            -1 if (event.num == 4 or event.delta > 0) else 1, "units")

    # ── Periodic update ───────────────────────────────────────────────

    def _schedule_update(self):
        self._update()
        self.after(UPDATE_MS, self._schedule_update)

    def _update(self):
        data = self._state.snapshot()

        # Banner
        if data["conn"] == ConnState.CONNECTED:
            if self._banner_visible:
                self._banner.pack_forget()
                self._banner_visible = False
        else:
            if not self._banner_visible:
                self._banner.pack(fill="x", side="top")
                self._banner_visible = True
            self._banner_msg.config(text=data["status"])
            nr = data["next_retry"]
            secs = max(0.0, nr - time.monotonic()) if nr is not None else None
            self._banner_countdown.config(
                text=f"retry in {secs:.0f} s" if secs is not None else "")

        # Status bar
        self._status_var.set(data["status"])
        t_ms = data["t_ms"]
        self._time_var.set(f"t = {t_ms/1000:.2f} s" if t_ms is not None else "")

        # Tables
        for i, (raw, conv, ok) in enumerate(data["pt"]):
            self._pt_tbl.update_row(i, raw, conv, ok)
        for i, (raw, conv, ok) in enumerate(data["cur"]):
            self._cur_tbl.update_row(i, raw, conv, ok)

        brd = data["brd"]
        if math.isnan(brd):
            self._brd_hdr_var.set("— °C")
            self._brd_hdr_lbl.config(fg=FG_WARN)
        else:
            self._brd_hdr_var.set(f"{brd:.1f} °C")
            self._brd_hdr_lbl.config(fg=FG_GOOD)

        # Terminal
        lines = self._state.take_term_lines()
        if lines is not None:
            self._term.configure(state="normal")
            self._term.delete("1.0", "end")
            self._term.insert("end", "\n".join(lines))
            if self._term_autoscroll:
                self._term.see("end")
            self._term.configure(state="disabled")

    def _on_term_scroll(self, _event):
        self.after(50, self._check_term_bottom)

    def _check_term_bottom(self):
        self._term_autoscroll = self._term.yview()[1] >= 0.999

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PandaV2 Sensor Monitor")
    parser.add_argument("--port", default=None,
                        help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", default=BAUD, type=int,
                        help=f"Baud rate (default {BAUD})")
    args = parser.parse_args()

    App(args.port, args.baud).mainloop()


if __name__ == "__main__":
    main()
