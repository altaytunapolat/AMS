#!/usr/bin/env python3
# double_ams_fast_plot.py — Dual active shielding + optional live plot (+ HOLD mode)

import argparse, csv, datetime as dt, os, queue, re, sys, threading, time
from collections import deque
from typing import Optional

try:
    import serial  # pyserial
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ---------- Reader ----------
LINE_RE = re.compile(
    r"""
    ^\s*
    S1:\s*
        (?P<s1>(--|[-+]?(\d+(\.\d*)?|\.\d+)))        # number or --
        \s*(?:u?µ?T)?
        (?:\s+ph=(?P<ph1>[-+]?\d+))?
    .*?
    \|\s*S2:\s*
        (?P<s2>(--|[-+]?(\d+(\.\d*)?|\.\d+)))
        \s*(?:u?µT)?
        (?:\s+ph=(?P<ph2>[-+]?\d+))?
    """,
    re.IGNORECASE | re.VERBOSE,
)

def parse_line(line: str):
    m = LINE_RE.search(line)
    if not m: return None
    def to_float(x): return None if x is None or x == "--" else float(x)
    return {"t": dt.datetime.now().astimezone(), "s1": to_float(m.group("s1")), "s2": to_float(m.group("s2"))}

def reader_thread(port, baud, out_q: queue.Queue, stop_evt: threading.Event):
    try:
        with serial.Serial(port, baudrate=baud, timeout=0.2) as ser:
            time.sleep(0.2)
            buf = bytearray()
            while not stop_evt.is_set():
                chunk = ser.read(1024)
                if chunk:
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        s = line.decode("utf-8", errors="ignore")
                        d = parse_line(s)
                        if d:
                            out_q.put(d)
    except serial.SerialException as e:
        out_q.put(("__error__", str(e)))

# ---------- Minimal Genesys driver ----------
CR = b"\r"
class GenesysPSU:
    def __init__(self, port: str, addr: int, baud: int = 9600, timeout: float = 2.0, write_sleep: float = 0.003):
        self.port = port; self.addr = addr; self.write_sleep = write_sleep
        self.ser = serial.Serial(port=port, baudrate=baud,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, timeout=timeout, write_timeout=timeout)
        self._adr_selected = False
    def close(self): 
        try: self.ser.close()
        except Exception: pass
    def _flush_in(self):
        try: self.ser.reset_input_buffer()
        except Exception: pass
    def _write(self, s: str):
        if not s.endswith("\r"): s += "\r"
        self.ser.write(s.encode("ascii")); self.ser.flush()
        if self.write_sleep > 0: time.sleep(self.write_sleep)
    def _readline_cr(self) -> str:
        buf = bytearray(); t0 = time.monotonic()
        while True:
            b = self.ser.read(1)
            if not b: break
            if b == CR: break
            if b == b"\n": continue
            buf += b
            if len(buf) > 512 or time.monotonic() - t0 > (self.ser.timeout or 0) + 0.5: break
        return buf.decode("ascii", errors="replace").strip()
    def transact(self, cmd: str) -> str:
        self._flush_in(); self._write(cmd); return self._readline_cr()
    def ensure_adr(self):
        if self._adr_selected: return
        r = self.transact(f"ADR {self.addr}")
        if r.strip().upper() != "OK":
            raise RuntimeError(f"{self.port}: ADR {self.addr} failed (got {r!r})")
        self._adr_selected = True
    def remote(self): self.ensure_adr(); return self.transact("RMT 1")
    def local(self):  self.ensure_adr(); return self.transact("RMT 0")
    def set_pv(self, v: float): self.ensure_adr(); return self.transact(f"PV {v:.4f}")
    def set_pc(self, a: float): self.ensure_adr(); return self.transact(f"PC {a:.4f}")
    def out(self, on: bool):    self.ensure_adr(); return self.transact("OUT 1" if on else "OUT 0")
    def mc(self) -> float:      self.ensure_adr(); return float(self.transact("MC?") or "nan")
    def mv(self) -> float:      self.ensure_adr(); return float(self.transact("MV?") or "nan")

# ---------- Helpers ----------
class LPF:
    def __init__(self, alpha: float, init: Optional[float] = None):
        self.alpha = max(0.0, min(1.0, alpha)); self.y = init
    def update(self, x: Optional[float]) -> Optional[float]:
        if x is None: return self.y
        if self.y is None: self.y = x
        else: self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def slew_toward(curr: float, target: float, dI_dt: float, dt: float) -> float:
    if dI_dt <= 0: return target
    max_step = dI_dt * max(1e-4, dt)
    delta = target - curr
    if abs(delta) <= max_step: return target
    return curr + (max_step if delta > 0 else -max_step)
def now_local_iso(): return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")

# ---------- Optional live plot ----------
class LivePlot:
    def __init__(self, history_sec=30, refresh_hz=20):
        import matplotlib.pyplot as plt  # lazy import so headless users aren’t forced to install
        self.plt = plt
        self.history = history_sec
        self.dt_max = history_sec
        self.refresh_dt = 1.0 / max(1.0, refresh_hz)

        self.t0 = time.time()
        maxlen = int(history_sec * 100)  # generous; we trim by time window
        self.t = deque(maxlen=maxlen)
        self.b1 = deque(maxlen=maxlen); self.b2 = deque(maxlen=maxlen)
        self.i1 = deque(maxlen=maxlen); self.i2 = deque(maxlen=maxlen)

        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        self.l_b1, = self.ax1.plot([], [], label="S1 µT")
        self.l_b2, = self.ax1.plot([], [], label="S2 µT")
        self.ax1.set_ylabel("Magnetic field (µT)")
        self.ax1.legend(loc="upper right")
        self.l_i1, = self.ax2.plot([], [], label="I1 (A)")
        self.l_i2, = self.ax2.plot([], [], label="I2 (A)")
        self.ax1.grid()
        self.ax2.grid()
        self.ax2.set_ylabel("Current (A)")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.legend(loc="upper right")
        self.fig.tight_layout()
        self._last_draw = 0.0

    def push(self, b1: Optional[float], b2: Optional[float], i1: float, i2: float):
        t = time.time() - self.t0
        self.t.append(t)
        self.b1.append(float("nan") if b1 is None else b1)
        self.b2.append(float("nan") if b2 is None else b2)
        self.i1.append(i1); self.i2.append(i2)

        if (t - self._last_draw) >= self.refresh_dt:
            self._last_draw = t
            # keep only last history seconds (by time)
            while self.t and (t - self.t[0]) > self.history:
                self.t.popleft(); self.b1.popleft(); self.b2.popleft(); self.i1.popleft(); self.i2.popleft()

            self.l_b1.set_data(self.t, self.b1)
            self.l_b2.set_data(self.t, self.b2)
            self.l_i1.set_data(self.t, self.i1)
            self.l_i2.set_data(self.t, self.i2)

            # autoscale y; x fixed to window
            for ax in (self.ax1, self.ax2):
                ax.relim(); ax.autoscale_view(scalex=False, scaley=True)
            if self.t:
                self.ax1.set_xlim(max(0, self.t[-1]-self.history), self.t[-1])

            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)  # keep UI responsive

    def close(self):
        try:
            self.plt.ioff(); self.plt.show(block=False); self.plt.close(self.fig)
        except Exception:
            pass

# ---------- Stdin/Hotkey watcher (optional) ----------
def hotkey_thread(cmd_q: queue.Queue, stop_evt: threading.Event):
    """Windows-friendly hotkey watcher: H toggles HOLD, Q quits."""
    try:
        import msvcrt  # Windows non-blocking getch
        while not stop_evt.is_set():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ("h","H"):
                    cmd_q.put(("toggle_hold", None))
                elif ch in ("q","Q"):
                    cmd_q.put(("quit", None))
            time.sleep(0.03)
    except ImportError:
        # Fallback: line-based stdin on non-Windows (blocking read in a thread)
        while not stop_evt.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.1); continue
            s = line.strip().lower()
            if s in ("h", "hold", "resume", "toggle"):
                cmd_q.put(("toggle_hold", None))
            elif s in ("q", "quit", "exit"):
                cmd_q.put(("quit", None))

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Dual active shielding with optional live plot.")
    # Sensor stream
    ap.add_argument("--sensor-port", required=True); ap.add_argument("--sensor-baud", type=int, default=115200)
    # PSUs
    ap.add_argument("--psu1", default="COM20"); ap.add_argument("--psu2", default="COM21")
    ap.add_argument("--addr", type=int, default=6); ap.add_argument("--pv", type=float, default=12.0)
    ap.add_argument("--write-sleep-ms", type=float, default=2.0)
    # Plant model
    ap.add_argument("--k1", type=float, default=4.0); ap.add_argument("--k2", type=float, default=4.0)
    ap.add_argument("--sign1", type=int, choices=[-1,1], default=-1); ap.add_argument("--sign2", type=int, choices=[-1,1], default=-1)
    ap.add_argument("--soll1", type=float, default=0.0); ap.add_argument("--soll2", type=float, default=0.0)
    # Control
    ap.add_argument("--sk1", type=float, default=0.2); ap.add_argument("--sk2", type=float, default=0.2)
    ap.add_argument("--imax", type=float, default=7.0)
    ap.add_argument("--dirate", type=float, default=8.0)
    ap.add_argument("--lpf-alpha", type=float, default=0.2)
    ap.add_argument("--one_shot", action="store_true")
    # NEW: Hold / hotkeys
    ap.add_argument("--hold", action="store_true",
                    help="Start in HOLD (compensation paused). Keeps last currents on coils.")
    ap.add_argument("--hotkeys", action="store_true",
                    help="Enable console hotkeys: H=toggle HOLD/RESUME, Q=quit.")
    # Telemetry / logging
    ap.add_argument("--tel-period", type=float, default=0.3)
    ap.add_argument("--csv", help="CSV path")
    ap.add_argument("--dry-run", action="store_true")
    # Plot
    ap.add_argument("--plot", action="store_true", help="show live plot window")
    ap.add_argument("--plot-history", type=float, default=30.0, help="seconds of history")
    ap.add_argument("--plot-fps", type=float, default=20.0, help="target plot refresh rate")
    args = ap.parse_args()

    # Reader thread
    q = queue.Queue(); stop_evt = threading.Event()
    t_reader = threading.Thread(target=reader_thread, args=(args.sensor_port, args.sensor_baud, q, stop_evt), daemon=True)
    t_reader.start()

    # Optional hotkeys
    cmd_q = queue.Queue()
    t_hot = None
    if args.hotkeys:
        t_hot = threading.Thread(target=hotkey_thread, args=(cmd_q, stop_evt), daemon=True)
        t_hot.start()

    # PSUs
    psu1 = GenesysPSU(args.psu1, addr=args.addr, write_sleep=args.write_sleep_ms/1000.0)
    psu2 = GenesysPSU(args.psu2, addr=args.addr, write_sleep=args.write_sleep_ms/1000.0)
    try:
        if not args.dry_run:
            for psu in (psu1, psu2):
                psu.remote(); psu.set_pv(args.pv); psu.set_pc(0.0); psu.out(True)
        print(f"PSU1 {args.psu1} & PSU2 {args.psu2} ready (PV={args.pv} V).")
    except Exception as e:
        print(f"ERROR initializing PSUs: {e}", file=sys.stderr)
        stop_evt.set(); t_reader.join(timeout=1.0)
        try: psu1.close(); psu2.close()
        except Exception: pass
        sys.exit(1)

    # State
    lpf1 = LPF(args.lpf_alpha); lpf2 = LPF(args.lpf_alpha)
    I1 = 0.0; I2 = 0.0
    did_oneshot1 = did_oneshot2 = False
    mc1 = mv1 = mc2 = mv2 = float('nan'); last_tel = time.time()

    # HOLD state
    hold = args.hold
    if args.hotkeys:
        print("Hotkeys: H=toggle HOLD/RESUME, Q=quit")

    # CSV
    writer = None; f = None
    if args.csv:
        first = (not os.path.exists(args.csv)) or (os.path.getsize(args.csv) == 0)
        f = open(args.csv, "a", newline="", encoding="utf-8", buffering=1)
        writer = csv.writer(f)
        if first: writer.writerow(["local_time","S1_uT","S2_uT","I1_A","I2_A","MC1_A","MV1_V","MC2_A","MV2_V"])

    # Plot
    plot = LivePlot(history_sec=args.plot_history, refresh_hz=args.plot_fps) if args.plot else None

    print("Controller running. Ctrl+C to stop.")
    t_prev = time.time()
    try:
        while True:
            # process hotkey/commands
            try:
                while True:
                    cmd, payload = cmd_q.get_nowait()
                    if cmd == "toggle_hold":
                        hold = not hold
                        state = "HOLD (paused)" if hold else "RESUME"
                        print(f"[{now_local_iso()}] Hotkey: {state}")
                    elif cmd == "quit":
                        raise KeyboardInterrupt
            except queue.Empty:
                pass

            # newest sample from queue
            d = None
            try:
                d = q.get(timeout=1.0)
                while True:
                    d = q.get_nowait()
            except queue.Empty:
                pass

            now = time.time()
            dt_sec = max(1e-3, now - t_prev); t_prev = now
            s1_uT = d.get("s1") if isinstance(d, dict) else None
            s2_uT = d.get("s2") if isinstance(d, dict) else None

            # Control (respect HOLD)
            if not hold:
                if s1_uT is not None:
                    a0 = lpf1.update(s1_uT)
                    dI_ideal = (-args.sign1 * ((a0 or 0.0) - args.soll1)) / max(1e-9, args.k1)
                    if args.one_shot and not did_oneshot1:
                        I_des = clamp(dI_ideal, 0.0, args.imax); did_oneshot1 = True
                    else:
                        I_des = clamp(I1 + args.sk1 * dI_ideal, 0.0, args.imax)
                    I1 = slew_toward(I1, I_des, args.dirate, dt_sec)
                    if not args.dry_run: psu1.set_pc(I1)

                if s2_uT is not None:
                    a0 = lpf2.update(s2_uT)
                    dI_ideal = (-args.sign2 * ((a0 or 0.0) - args.soll2)) / max(1e-9, args.k2)
                    if args.one_shot and not did_oneshot2:
                        I_des = clamp(dI_ideal, 0.0, args.imax); did_oneshot2 = True
                    else:
                        I_des = clamp(I2 + args.sk2 * dI_ideal, 0.0, args.imax)
                    I2 = slew_toward(I2, I_des, args.dirate, dt_sec)
                    if not args.dry_run: psu2.set_pc(I2)
            # else: HOLD — keep last I1/I2 and do not write set_pc()

            # Telemetry (still read while HOLD)
            if (now - last_tel) >= args.tel_period and not args.dry_run:
                time.sleep(0.02)
                try: mc1, mv1 = psu1.mc(), psu1.mv()
                except Exception: mc1 = mv1 = float('nan')
                try: mc2, mv2 = psu2.mc(), psu2.mv()
                except Exception: mc2 = mv2 = float('nan')
                last_tel = now

            # Console
            stamp = now_local_iso()
            s1txt = f"{s1_uT:+.3f}µT" if s1_uT is not None else "--"
            s2txt = f"{s2_uT:+.3f}µT" if s2_uT is not None else "--"
            holdtxt = " [HOLD]" if hold else ""
            print(f"{stamp}{holdtxt}  S1={s1txt} → I1={I1:.3f}A (MC1={mc1 if mc1==mc1 else float('nan'):.3f}A, MV1={mv1 if mv1==mv1 else float('nan'):.2f}V)  "
                  f"S2={s2txt} → I2={I2:.3f}A (MC2={mc2 if mc2==mc2 else float('nan'):.3f}A, MV2={mv2 if mv2==mv2 else float('nan'):.2f}V)")

            # CSV
            if writer:
                writer.writerow([stamp,
                                 f"{s1_uT:.6f}" if s1_uT is not None else "",
                                 f"{s2_uT:.6f}" if s2_uT is not None else "",
                                 f"{I1:.6f}", f"{I2:.6f}", mc1, mv1, mc2, mv2])

            # Plot push
            if plot:
                plot.push(s1_uT, s2_uT, I1, I2)

    except KeyboardInterrupt:
        print("\nStopping (user).")
    finally:
        if plot: plot.close()
        stop_evt.set(); t_reader.join(timeout=1.0)
        try:
            if not args.dry_run:
                for psu in (psu1, psu2):
                    psu.set_pc(0.0); psu.out(False); psu.local()
        except Exception: pass
        try:
            if f: f.flush(); os.fsync(f.fileno()); f.close()
        except Exception: pass
        try:
            psu1.close(); psu2.close()
        except Exception: pass

if __name__ == "__main__":
    main()
