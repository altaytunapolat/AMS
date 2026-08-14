#!/usr/bin/env python3
# ams_5d_matrix_axisPI_Dy.py — 5-channel coupled AMS with axis-specific PI (X,Z,Y) and optional D on Y
# Sensors used: S1, S2, S3, S4, S5
# Coils driven:  1,  2,  3,  4,  5
# Notes:
# - X axis: S1 & S3  -> coils 1 & 3   (gainx, kix)
# - Z axis: S2       -> coil  2       (gainz, kiz)   (single-sensor partial axis)
# - Y axis: S4 & S5  -> coils 4 & 5   (gainy, kiy, optional kdy)
# - Cross-coupling handled by M_INV sub-matrix (rows/cols 1..5). No S6 yet.

import argparse, csv, datetime as dt, math, os, queue, re, sys, threading, time
from collections import deque
from typing import Optional, List

# ---------- I/O timing ----------
BUS_TIMEOUT_S       = 0.8
ADR_RETRIES         = 3
CMD_RETRIES         = 2
POST_WRITE_SETTLE_S = 0.15
CR                  = b"\r"

# ---------- Sensor parser (S1..S6 tolerant) ----------
LINE_RE = re.compile(
    r"""
    S1:\s*(?P<s1>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)? .*?
    S2:\s*(?P<s2>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)? .*?
    S3:\s*(?P<s3>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)? .*?
    S4:\s*(?P<s4>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)? .*?
    S5:\s*(?P<s5>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)?
    (?: .*? S6:\s*(?P<s6>--|[-+]?\d+(?:\.\d+)?|\.\d+)\s*(?:u?µ?T)?)?
    """,
    re.IGNORECASE | re.VERBOSE,
)

def _to_float(x: Optional[str]) -> Optional[float]:
    return None if x is None or x.strip() == "--" else float(x)

def parse_line(line: str):
    m = LINE_RE.search(line)
    if not m:
        return None
    return {
        "t": time.time(),
        "s1": _to_float(m.group("s1")),
        "s2": _to_float(m.group("s2")),
        "s3": _to_float(m.group("s3")),
        "s4": _to_float(m.group("s4")),
        "s5": _to_float(m.group("s5")),
        "s6": _to_float(m.group("s6")),
    }

def sensor_reader(port, baud, out_q: queue.Queue, stop_evt: threading.Event):
    try:
        import serial
        with serial.Serial(port, baudrate=baud, timeout=0.2) as ser:
            time.sleep(0.1)
            buf = bytearray()
            last_emit = None
            while not stop_evt.is_set():
                chunk = ser.read(256)
                if chunk:
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        s = line.decode("utf-8", errors="ignore")
                        d = parse_line(s)
                        if d: last_emit = d
                else:
                    time.sleep(0.01)
                if last_emit:
                    out_q.put(last_emit); last_emit = None
    except Exception as e:
        out_q.put(("__sensor_error__", repr(e)))

# ---------- Utils ----------
def now_iso(): return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class LPF:
    def __init__(self, alpha: float, init: Optional[float] = None):
        self.alpha = max(0.0, min(1.0, alpha)); self.y = init
    def update(self, x: Optional[float]) -> Optional[float]:
        if x is None: return self.y
        self.y = x if self.y is None else (self.alpha * x + (1 - self.alpha) * self.y)
        return self.y

class DerivLPF:
    """Band-limited derivative estimator with exponential smoothing."""
    def __init__(self, alpha: float = 0.25):
        self.alpha = max(0.0, min(1.0, alpha))
        self.prev = None
        self.dhat = 0.0
    def update(self, x: Optional[float], dt_sec: float) -> float:
        if x is None or dt_sec <= 0: return self.dhat
        if self.prev is None:
            self.prev = x
            return self.dhat
        raw = (x - self.prev) / dt_sec
        self.prev = x
        self.dhat = self.alpha * raw + (1 - self.alpha) * self.dhat
        return self.dhat

def slew_toward(curr: float, target: float, dI_dt: float, dt_sec: float) -> float:
    if dI_dt <= 0: return target
    max_step = dI_dt * max(1e-4, dt_sec)
    delta = target - curr
    if abs(delta) <= max_step: return target
    return curr + (max_step if delta > 0 else -max_step)

# ---------- Robust shared Genesys bus ----------
try:
    import serial
    from serial import SerialException, SerialTimeoutException
except ImportError:
    print("Missing dependency: pyserial. Install with:  pip install pyserial", file=sys.stderr)
    sys.exit(1)

class GenesysBus:
    """One serial port, multiple PSUs via ADR; cached ADR; retries; reopen; pacing."""
    def __init__(self, port: str, baud: int = 9600, timeout: float = BUS_TIMEOUT_S, write_sleep: float = 0.004,
                 adr_retries: int = ADR_RETRIES, cmd_retries: int = CMD_RETRIES):
        self.port = port; self.baud = baud; self.timeout = timeout
        self.write_sleep = max(0.0, write_sleep)
        self.adr_retries = max(0, adr_retries); self.cmd_retries = max(0, cmd_retries)
        self._ser = None; self._adr_selected = None; self._open()

    def _open(self):
        if self._ser and getattr(self._ser, "is_open", False):
            try: self._ser.close()
            except Exception: pass
        self._ser = serial.Serial(
            port=self.port, baudrate=self.baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout, write_timeout=self.timeout,
        )
        time.sleep(0.05); self._adr_selected = None

    def _ensure_open(self):
        if self._ser is None or not getattr(self._ser, "is_open", False): self._open()
    def close(self):
        try:
            if self._ser and self._ser.is_open: self._ser.close()
        except Exception: pass
        self._ser = None; self._adr_selected = None
    def _flush_in(self):
        try: self._ser.reset_input_buffer()
        except Exception: pass
    def _write(self, s: str):
        if not s.endswith("\r"): s += "\r"
        self._ser.write(s.encode("ascii")); self._ser.flush()
        if self.write_sleep: time.sleep(self.write_sleep)
    def _readline_cr(self) -> str:
        buf = bytearray(); t0 = time.monotonic()
        while True:
            b = self._ser.read(1)
            if not b: break
            if b == CR: break
            if b == b"\n": continue
            buf += b
            if len(buf) > 512 or (time.monotonic() - t0) > (self.timeout + 0.4): break
        return buf.decode("ascii", errors="replace").strip()
    def _xact_once(self, cmd: str) -> str:
        self._flush_in(); self._write(cmd); return self._readline_cr()
    def _ensure_adr(self, addr: int):
        if self._adr_selected == addr: return
        last_err = None
        for _ in range(self.adr_retries + 1):
            try:
                self._ensure_open()
                r = self._xact_once(f"ADR {addr}")
                if r.strip().upper() == "OK":
                    self._adr_selected = addr; return
                last_err = RuntimeError(f"{self.port}: ADR {addr} failed (got {r!r})")
            except (SerialException, SerialTimeoutException, OSError, AttributeError) as e:
                last_err = RuntimeError(f"{self.port}: ADR {addr} IO error: {e!r}")
                self._open(); time.sleep(0.02)
        raise last_err or RuntimeError(f"{self.port}: ADR {addr} failed")
    def transact(self, addr: int, cmd: str) -> str:
        last_err = None
        for _ in range(self.cmd_retries + 1):
            try:
                self._ensure_open(); self._ensure_adr(addr); return self._xact_once(cmd)
            except (SerialException, SerialTimeoutException, OSError, AttributeError) as e:
                last_err = RuntimeError(f"{self.port}: cmd {cmd!r} IO error: {e!r}")
                self._adr_selected = None; self._open(); time.sleep(0.02)
        raise last_err or RuntimeError(f"{self.port}: cmd {cmd!r} failed")

class GenesysPSU:
    def __init__(self, bus: GenesysBus, addr: int):
        self.bus = bus; self.addr = addr
    def remote(self): return self.bus.transact(self.addr, "RMT 1")
    def local(self):  return self.bus.transact(self.addr, "RMT 0")
    def set_pv(self, v: float): return self.bus.transact(self.addr, f"PV {v:.4f}")
    def set_pc(self, a: float): return self.bus.transact(self.addr, f"PC {a:.4f}")
    def out(self, on: bool):    return self.bus.transact(self.addr, "OUT 1" if on else "OUT 0")
    def mc(self) -> float:
        r = self.bus.transact(self.addr, "MC?");  return float(r) if r else float("nan")
    def mv(self) -> float:
        r = self.bus.transact(self.addr, "MV?");  return float(r) if r else float("nan")

class PSUCommandThrottler:
    def __init__(self, psu: GenesysPSU, eps: float = 0.02, min_interval: float = 0.10):
        self.psu = psu; self.eps = max(0.0, eps); self.min_interval = max(0.02, min_interval)
        self._last_set = 0.0; self._last_t = 0.0; self.last_write_t = 0.0
    def set_pc(self, target_a: float):
        t = time.monotonic()
        if abs(target_a - self._last_set) >= self.eps or (t - self._last_t) >= self.min_interval:
            self.psu.set_pc(target_a); self._last_set = target_a; self._last_t = t; self.last_write_t = t

# ---------- Live plot (optional) ----------
class LivePlot:
    def __init__(self, history_sec=60, refresh_hz=15):
        import matplotlib.pyplot as plt
        self.plt = plt; self.history = history_sec; self.refresh_dt = 1.0 / max(1.0, refresh_hz)
        self.t0 = time.time(); maxlen = int(history_sec * 100)
        self.t = deque(maxlen=maxlen)
        self.b1 = deque(maxlen=maxlen)  # S1
        self.b2 = deque(maxlen=maxlen)  # S2
        self.b3 = deque(maxlen=maxlen)  # S3
        self.b4 = deque(maxlen=maxlen)  # S4
        self.b5 = deque(maxlen=maxlen)  # S5
        self.i1 = deque(maxlen=maxlen); self.i2 = deque(maxlen=maxlen); self.i3 = deque(maxlen=maxlen)
        self.i4 = deque(maxlen=maxlen); self.i5 = deque(maxlen=maxlen)
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 8))
        self.l_b1, = self.ax1.plot([], [], label="S1 µT")
        self.l_b2, = self.ax1.plot([], [], label="S2 µT")
        self.l_b3, = self.ax1.plot([], [], label="S3 µT")
        self.l_b4, = self.ax1.plot([], [], label="S4 µT")
        self.l_b5, = self.ax1.plot([], [], label="S5 µT")
        self.ax1.set_ylabel("Field (µT)"); self.ax1.legend(loc="upper right"); self.ax1.grid()
        self.l_i1, = self.ax2.plot([], [], label="I1 (A)")
        self.l_i2, = self.ax2.plot([], [], label="I2 (A)")
        self.l_i3, = self.ax2.plot([], [], label="I3 (A)")
        self.l_i4, = self.ax2.plot([], [], label="I4 (A)")
        self.l_i5, = self.ax2.plot([], [], label="I5 (A)")
        self.ax2.set_ylabel("Current (A)"); self.ax2.set_xlabel("Time (s)")
        self.ax2.legend(loc="upper right"); self.ax2.grid()
        self.fig.tight_layout(); self._last_draw = 0.0
    def push(self, s1,s2,s3,s4,s5, i1,i2,i3,i4,i5):
        t = time.time() - self.t0
        self.t.append(t)
        self.b1.append(float("nan") if s1 is None else s1)
        self.b2.append(float("nan") if s2 is None else s2)
        self.b3.append(float("nan") if s3 is None else s3)
        self.b4.append(float("nan") if s4 is None else s4)
        self.b5.append(float("nan") if s5 is None else s5)
        self.i1.append(i1); self.i2.append(i2); self.i3.append(i3); self.i4.append(i4); self.i5.append(i5)
        if (t - self._last_draw) >= self.refresh_dt:
            self._last_draw = t
            while self.t and (t - self.t[0]) > self.history:
                self.t.popleft(); self.b1.popleft(); self.b2.popleft(); self.b3.popleft(); self.b4.popleft(); self.b5.popleft()
                self.i1.popleft(); self.i2.popleft(); self.i3.popleft(); self.i4.popleft(); self.i5.popleft()
            self.l_b1.set_data(self.t, self.b1); self.l_b2.set_data(self.t, self.b2); self.l_b3.set_data(self.t, self.b3)
            self.l_b4.set_data(self.t, self.b4); self.l_b5.set_data(self.t, self.b5)
            self.l_i1.set_data(self.t, self.i1); self.l_i2.set_data(self.t, self.i2); self.l_i3.set_data(self.t, self.i3)
            self.l_i4.set_data(self.t, self.i4); self.l_i5.set_data(self.t, self.i5)
            for ax in (self.ax1, self.ax2): ax.relim(); ax.autoscale_view()
            if self.t: self.ax1.set_xlim(max(0, self.t[-1]-self.history), self.t[-1])
            self.fig.canvas.draw_idle(); self.plt.pause(0.001)
    def close(self):
        try: self.plt.ioff(); self.plt.show(block=False); self.plt.close(self.fig)
        except Exception: pass

# ---------- Your 6×6 inverse matrix (µT/A)^-1 ----------
M_INV = [
    [-1.04863727,  0.81762905,  0.80188590, -0.83665581,  0.79769996, -0.78269065],
    [ 0.84199632, -1.03340832, -0.81697353,  0.83985358, -0.79952195,  0.77381819],
    [ 0.82903277, -0.81995219, -1.03038797,  0.83845299, -0.80046001,  0.78594077],
    [-0.83461625,  0.81345494,  0.81101593, -1.04300715,  0.78062754, -0.77997699],
    [ 0.84051140, -0.81296052, -0.81532356,  0.82194255, -1.00375882,  0.78584119],
    [-0.83968643,  0.80264323,  0.81138924, -0.82671785,  0.80705722, -0.98595438],
]

# Active submatrix: sensors [S1,S2,S3,S4,S5] -> cols [0,1,2,3,4]; coils [1..5] -> rows [0,1,2,3,4]
S_IDX = [0,1,2,3,4]
R_IDX = [0,1,2,3,4]

def matvec_5x5(e5: List[float]) -> List[float]:
    """y = - M_sub * e, where M_sub = M_INV[R_IDX][:, S_IDX]  (signed Amps)."""
    y = [0.0]*5
    for ri, r in enumerate(R_IDX):
        acc = 0.0
        for ci, s in enumerate(S_IDX):
            acc += M_INV[r][s] * e5[ci]
        y[ri] = -acc
    return y

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="AMS 5-channel coupled controller (coils 1..5 ; sensors 1..5).")
    ap.add_argument("--sensor-port", required=True)
    ap.add_argument("--sensor-baud", type=int, default=115200)
    ap.add_argument("--psu-port", required=True)
    ap.add_argument("--psu-baud", type=int, default=9600)

    # PSU addresses 1..5
    ap.add_argument("--addr1", type=int, default=1)
    ap.add_argument("--addr2", type=int, default=2)  # NEW: coil 2
    ap.add_argument("--addr3", type=int, default=3)
    ap.add_argument("--addr4", type=int, default=4)
    ap.add_argument("--addr5", type=int, default=5)
    ap.add_argument("--pv", type=float, default=20.0)

    # Polarity per coil (sign so that positive PC makes field consistent with matrix)
    ap.add_argument("--sign1", type=int, choices=[-1,1], default=1)
    ap.add_argument("--sign2", type=int, choices=[-1,1], default=1)  # NEW
    ap.add_argument("--sign3", type=int, choices=[-1,1], default=1)
    ap.add_argument("--sign4", type=int, choices=[-1,1], default=1)
    ap.add_argument("--sign5", type=int, choices=[-1,1], default=1)

    # Soll values (µT)
    ap.add_argument("--soll1", type=float, default=0.0)
    ap.add_argument("--soll2", type=float, default=0.0)  # NEW
    ap.add_argument("--soll3", type=float, default=0.0)
    ap.add_argument("--soll4", type=float, default=0.0)
    ap.add_argument("--soll5", type=float, default=0.0)

    # Filters & dynamics (axis-specific P/I; optional D on Y)
    ap.add_argument("--lpf-alpha", type=float, default=0.2)
    ap.add_argument("--gainx", type=float, default=0.22, help="P gain for X (S1,S3)")
    ap.add_argument("--kix",   type=float, default=0.003, help="I gain [1/s] for X")
    ap.add_argument("--gainz", type=float, default=0.20, help="P gain for Z (S2)")
    ap.add_argument("--kiz",   type=float, default=0.002, help="I gain [1/s] for Z")
    ap.add_argument("--gainy", type=float, default=0.16, help="P gain for Y (S4,S5)")
    ap.add_argument("--kiy",   type=float, default=0.001, help="I gain [1/s] for Y")
    ap.add_argument("--i-max-esum", type=float, default=0.25, help="Integral clamp per component [µT·s]")

    # Small D on Y only (optional)
    ap.add_argument("--kdy", type=float, default=0.0, help="D gain on Y (S4,S5); try 0.02–0.05")
    ap.add_argument("--dy-alpha", type=float, default=0.25, help="Derivative smoothing 0..1 (lower = more smoothing)")

    ap.add_argument("--imax",   type=float, default=20.0, help="|I| limit (A) in algorithm")
    ap.add_argument("--dirate", type=float, default=6.0,  help="slew limit A/s")
    ap.add_argument("--sensor-stale-sec", type=float, default=1.0)

    # Bus pacing / cadence
    ap.add_argument("--write-sleep-ms", type=float, default=0.1)
    ap.add_argument("--loop-period", type=float, default=0.12)
    ap.add_argument("--tel-period", type=float, default=0.25)
    ap.add_argument("--pc-min-interval", type=float, default=0.12)

    # UX
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--plot-history", type=float, default=60.0)
    ap.add_argument("--plot-fps", type=float, default=20.0)
    ap.add_argument("--csv")
    ap.add_argument("--hotkeys", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    try: sys.stdout.reconfigure(line_buffering=True)
    except Exception: pass

    # Sensor thread
    q = queue.Queue(); stop_evt = threading.Event()
    threading.Thread(target=sensor_reader, args=(args.sensor_port, args.sensor_baud, q, stop_evt), daemon=True).start()

    # Hotkeys
    cmd_q = queue.Queue()
    if args.hotkeys:
        def hotkey_thread(cmd_q, stop_evt):
            try:
                import msvcrt
                while not stop_evt.is_set():
                    if msvcrt.kbhit():
                        ch = msvcrt.getwch()
                        if ch in ("h","H"): cmd_q.put(("toggle_hold", None))
                        elif ch in ("q","Q"): cmd_q.put(("quit", None))
                    time.sleep(0.03)
            except ImportError:
                while not stop_evt.is_set():
                    line = sys.stdin.readline()
                    if not line:
                        time.sleep(0.1); continue
                    s = line.strip().lower()
                    if s in ("h","hold","resume","toggle"): cmd_q.put(("toggle_hold", None))
                    elif s in ("q","quit","exit"): cmd_q.put(("quit", None))
        threading.Thread(target=hotkey_thread, args=(cmd_q, stop_evt), daemon=True).start()

    # Bus + PSUs 1..5
    bus = GenesysBus(
        port=args.psu_port, baud=args.psu_baud, timeout=BUS_TIMEOUT_S,
        write_sleep=args.write_sleep_ms/1000.0, adr_retries=ADR_RETRIES, cmd_retries=CMD_RETRIES,
    )
    psu = {
        1: GenesysPSU(bus, args.addr1),
        2: GenesysPSU(bus, args.addr2),
        3: GenesysPSU(bus, args.addr3),
        4: GenesysPSU(bus, args.addr4),
        5: GenesysPSU(bus, args.addr5),
    }
    thr = {
        1: PSUCommandThrottler(psu[1], eps=0.02, min_interval=args.pc_min_interval),
        2: PSUCommandThrottler(psu[2], eps=0.02, min_interval=args.pc_min_interval),
        3: PSUCommandThrottler(psu[3], eps=0.02, min_interval=args.pc_min_interval),
        4: PSUCommandThrottler(psu[4], eps=0.02, min_interval=args.pc_min_interval),
        5: PSUCommandThrottler(psu[5], eps=0.02, min_interval=args.pc_min_interval),
    }

    try:
        if not args.dry_run:
            for k in (1,2,3,4,5):
                psu[k].remote(); psu[k].set_pv(args.pv); psu[k].set_pc(0.0); psu[k].out(True)
        print(f"PSUs on {args.psu_port} ready (ADDRs={args.addr1},{args.addr2},{args.addr3},{args.addr4},{args.addr5}, PV={args.pv:.1f} V).")
    except Exception as e:
        print(f"ERROR initializing PSUs: {e}", file=sys.stderr)
        stop_evt.set(); bus.close(); sys.exit(1)

    # State
    lpf = {k: LPF(args.lpf_alpha) for k in (1,2,3,4,5,6)}
    I = {1:0.0, 2:0.0, 3:0.0, 4:0.0, 5:0.0}
    last_sample_t = 0.0
    hold = False
    signs = {1: args.sign1, 2: args.sign2, 3: args.sign3, 4: args.sign4, 5: args.sign5}
    solls = {1: args.soll1, 2: args.soll2, 3: args.soll3, 4: args.soll4, 5: args.soll5}

    # Integral stores (µT·s) for S1..S5
    e_int = [0.0]*5

    # Derivative estimators for Y sensors (S4,S5)
    d_s4 = DerivLPF(alpha=args.dy_alpha)
    d_s5 = DerivLPF(alpha=args.dy_alpha)

    # CSV
    writer = None; f = None
    if args.csv:
        # ensure parent dir exists
        d = os.path.dirname(os.path.abspath(args.csv))
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)
        # open in "w" to truncate any old file
        f = open(args.csv, "w", newline="", encoding="utf-8", buffering=1)
        writer = csv.writer(f)
        writer.writerow([
            "local_time","S1_uT","S2_uT","S3_uT","S4_uT","S5_uT",
            "I1_A","I2_A","I3_A","I4_A","I5_A",
            "MC1_A","MV1_V","MC2_A","MV2_V","MC3_A","MV3_V","MC4_A","MV4_V","MC5_A","MV5_V"
        ])

    print("Controller running. Ctrl+C to stop. Hotkeys: H=hold/resume, Q=quit" if args.hotkeys else "Controller running. Ctrl+C to stop.")


    plot = LivePlot(history_sec=args.plot_history, refresh_hz=args.plot_fps) if args.plot else None

    print("Controller running. Ctrl+C to stop. Hotkeys: H=hold/resume, Q=quit" if args.hotkeys else "Controller running. Ctrl+C to stop.")
    next_tick = time.monotonic(); prev_wall = time.time(); last_tel = 0.0
    mc = {k: float("nan") for k in (1,2,3,4,5)}
    mv = {k: float("nan") for k in (1,2,3,4,5)}

    try:
        while True:
            # Hotkeys
            try:
                while True:
                    cmd, _ = cmd_q.get_nowait()
                    if cmd == "toggle_hold": hold = not hold; print(f"[{now_iso()}] {'HOLD' if hold else 'RESUME'}")
                    elif cmd == "quit": raise KeyboardInterrupt
            except queue.Empty:
                pass

            # Drain latest sensor sample
            d = None
            try:
                while True:
                    d = q.get_nowait()
            except queue.Empty:
                pass

            # Parse sensors (LPF)
            s = {k: None for k in (1,2,3,4,5,6)}
            if isinstance(d, dict):
                for k in (1,2,3,4,5,6):
                    raw = d.get(f"s{k}")
                    s[k] = lpf[k].update(raw)
                last_sample_t = d.get("t", last_sample_t)

            now_wall = time.time()
            dt_sec = max(1e-3, now_wall - prev_wall); prev_wall = now_wall
            fresh = (last_sample_t > 0.0) and ((now_wall - last_sample_t) <= args.sensor_stale_sec)

            # Control
            if fresh and not hold:
                # Error vector in sensor space (µT) ordered [S1,S2,S3,S4,S5]
                e5 = [
                    (s[1] or 0.0) - solls[1],
                    (s[2] or 0.0) - solls[2],
                    (s[3] or 0.0) - solls[3],
                    (s[4] or 0.0) - solls[4],
                    (s[5] or 0.0) - solls[5],
                ]

                # Integrator update (µT·s) + clamp
                for i in range(5):
                    e_int[i] += e5[i] * dt_sec
                    if e_int[i] >  args.i_max_esum: e_int[i] =  args.i_max_esum
                    if e_int[i] < -args.i_max_esum: e_int[i] = -args.i_max_esum

                # Axis-specific P/I scaling BEFORE matrix (preserves coupling)
                # X: indices 0 (S1), 2 (S3); Z: index 1 (S2); Y: indices 3 (S4), 4 (S5)
                e5_P = [
                    args.gainx*e5[0],
                    args.gainz*e5[1],
                    args.gainx*e5[2],
                    args.gainy*e5[3],
                    args.gainy*e5[4],
                ]
                e5_I = [
                    args.kix*e_int[0],
                    args.kiz*e_int[1],
                    args.kix*e_int[2],
                    args.kiy*e_int[3],
                    args.kiy*e_int[4],
                ]

                # Y-derivative (band-limited) on S4,S5 error
                de_s4 = d_s4.update(e5[3], dt_sec)
                de_s5 = d_s5.update(e5[4], dt_sec)
                e5_D = [0.0, 0.0, 0.0, args.kdy*de_s4, args.kdy*de_s5] if args.kdy != 0.0 else [0.0]*5

                # Map via matrix and sum components
                dI_P = matvec_5x5(e5_P)
                dI_I = matvec_5x5(e5_I)
                dI_D = matvec_5x5(e5_D) if args.kdy != 0.0 else [0.0]*5

                I_t = {
                    1: clamp(I[1] + dI_P[0] + dI_I[0] + dI_D[0], -args.imax, args.imax),
                    2: clamp(I[2] + dI_P[1] + dI_I[1] + dI_D[1], -args.imax, args.imax),
                    3: clamp(I[3] + dI_P[2] + dI_I[2] + dI_D[2], -args.imax, args.imax),
                    4: clamp(I[4] + dI_P[3] + dI_I[3] + dI_D[3], -args.imax, args.imax),
                    5: clamp(I[5] + dI_P[4] + dI_I[4] + dI_D[4], -args.imax, args.imax),
                }

                for k in (1,2,3,4,5):
                    I[k] = slew_toward(I[k], I_t[k], args.dirate, dt_sec)

            # Map signed I -> PSU PC via polarity and clamp to [0, imax]
            pc = {k: clamp(signs[k]*I[k], 0.0, args.imax) for k in (1,2,3,4,5)}

            # Writes
            if not args.dry_run:
                for k in (1,2,3,4,5):
                    try: thr[k].set_pc(pc[k])
                    except Exception as e: print(f"[WARN] PSU{k} set_pc: {e}")

            time.sleep(POST_WRITE_SETTLE_S)

            # Telemetry (gentle ADR)
            if args.tel_period > 0 and (now_wall - last_tel) >= args.tel_period and not args.dry_run:
                for k in (1,2,3,4,5):
                    try: mc[k], mv[k] = psu[k].mc(), psu[k].mv()
                    except Exception: mc[k]=mv[k]=float("nan")
                    time.sleep(max(0.0, args.write_sleep_ms/1000.0))
                last_tel = time.time()

            # Console
            def fmtf(x, n=3):
                if x is None: return "--"
                try:
                    if (x != x) or math.isinf(x): return "nan"
                    return f"{x:.{n}f}"
                except Exception:
                    return "--"

            stalemark = "" if fresh else " [STALE]"
            holdtxt = " [HOLD]" if hold else ""
            sat = {k: " [SAT]" if signs[k]*I[k] < 0 else "" for k in (1,2,3,4,5)}
            print(f"{now_iso()}{holdtxt}{stalemark}  "
                  f"S1={fmtf(s[1])}µT S2={fmtf(s[2])}µT S3={fmtf(s[3])}µT  S4={fmtf(s[4])}µT S5={fmtf(s[5])}µT  |  "
                  f"I1={I[1]:+.3f}A→PC{pc[1]:.3f}{sat[1]}  I2={I[2]:+.3f}A→PC{pc[2]:.3f}{sat[2]}  "
                  f"I3={I[3]:+.3f}A→PC{pc[3]:.3f}{sat[3]}  I4={I[4]:+.3f}A→PC{pc[4]:.3f}{sat[4]}  I5={I[5]:+.3f}A→PC{pc[5]:.3f}{sat[5]}  |  "
                  f"MC1={fmtf(mc[1])}A MV1={fmtf(mv[1],2)}V  MC2={fmtf(mc[2])}A MV2={fmtf(mv[2],2)}V  "
                  f"MC3={fmtf(mc[3])}A MV3={fmtf(mv[3],2)}V  MC4={fmtf(mc[4])}A MV4={fmtf(mv[4],2)}V  MC5={fmtf(mc[5])}A MV5={fmtf(mv[5],2)}V")

            # CSV
            if writer:
                writer.writerow([
                    now_iso(),
                    "" if s[1] is None else f"{s[1]:.6f}",
                    "" if s[2] is None else f"{s[2]:.6f}",
                    "" if s[3] is None else f"{s[3]:.6f}",
                    "" if s[4] is None else f"{s[4]:.6f}",
                    "" if s[5] is None else f"{s[5]:.6f}",
                    f"{I[1]:.6f}", f"{I[2]:.6f}", f"{I[3]:.6f}", f"{I[4]:.6f}", f"{I[5]:.6f}",
                    mc[1], mv[1], mc[2], mv[2], mc[3], mv[3], mc[4], mv[4], mc[5], mv[5],
                ])

            if plot:
                plot.push(s[1], s[2], s[3], s[4], s[5], I[1], I[2], I[3], I[4], I[5])

            # Cadence
            next_tick += args.loop_period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0: time.sleep(sleep_for)
            else: next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopping (user).")
    finally:
        if plot: plot.close()
        stop_evt.set()
        if not args.dry_run:
            for k in (1,2,3,4,5):
                try: psu[k].set_pc(0.0)
                except Exception: pass
                try: psu[k].out(False)
                except Exception: pass
                try: psu[k].local()
                except Exception: pass
        try:
            if writer and not f.closed:
                f.flush(); os.fsync(f.fileno()); f.close()
        except Exception: pass
        bus.close()

if __name__ == "__main__":
    main()
