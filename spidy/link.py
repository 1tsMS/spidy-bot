"""Text-line link to the Spidy firmware (firmware/spidy_fw) over USB serial or WiFi TCP.

Protocol (one command per line, '\n' terminated):
  P16,<16 ticks>   set all channels at once (0 = channel off). Streamed ~50 Hz, no reply.
  M,<ch>,<ticks>   set one PHYSICAL channel (raw, no calibration)      -> M_OK,ch,ticks
  OFF              all channels off (E-stop)                           -> OFF_OK
  PING,<n>         latency check                                       -> PONG,<n>
  IMU              read the MPU6050                                    -> IMU,ax,ay,az,gx,gy,gz
  STATE            last ticks sent to each channel                     -> STATE,<16 ticks>
  BOOT,<16 ticks>  store the pose the robot takes at power-on (flash)  -> BOOT_OK
  HELLO            identify                                            -> SPIDY,fw=<n>,ip=<ip>
Firmware also prints READY at boot and STALE if the P16 stream stops (it then HOLDS).

Reading runs on a background thread; every received line goes to on_line(line).
"""
import socket
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # the app still runs in sim-only mode
    serial = None

BAUD = 115200
TCP_PORT = 5000


def list_serial_ports():
    if serial is None:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


class Link:
    def __init__(self, on_line=None, on_status=None):
        self.on_line = on_line or (lambda line: None)
        self.on_status = on_status or (lambda connected, text: None)
        self._ser = None
        self._sock = None
        self._thread = None
        self._stop = threading.Event()
        self._wlock = threading.Lock()
        self.name = ""

    @property
    def connected(self):
        return self._ser is not None or self._sock is not None

    # ---- open / close
    def open_serial(self, port):
        self.close()
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self._ser = serial.Serial(port, BAUD, timeout=0.05)
        self.name = port
        self._start()

    def open_tcp(self, host, port=TCP_PORT, timeout=3.0):
        self.close()
        s = socket.create_connection((host, port), timeout=timeout)
        s.settimeout(0.05)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # don't batch small lines
        self._sock = s
        self.name = f"{host}:{port}"
        self._start()

    def close(self):
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        for h in (self._ser, self._sock):
            try:
                if h:
                    h.close()
            except OSError:
                pass
        was = self.connected
        self._ser = self._sock = self._thread = None
        if was:
            self.on_status(False, "disconnected")

    def _start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.on_status(True, f"connected: {self.name}")

    # ---- write
    def send(self, line):
        data = (line.strip() + "\n").encode()
        try:
            with self._wlock:
                if self._ser:
                    self._ser.write(data)
                elif self._sock:
                    self._sock.sendall(data)
                else:
                    return False
            return True
        except OSError as e:
            self.on_line(f"! write failed: {e}")
            self.close()
            return False

    def send_pulses(self, pulses16):
        return self.send("P16," + ",".join(str(int(p)) for p in pulses16))

    # ---- read (background thread)
    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                if self._ser:
                    chunk = self._ser.read(256)
                else:
                    try:
                        chunk = self._sock.recv(1024)
                    except socket.timeout:
                        chunk = b""
                    else:
                        if chunk == b"":                       # peer closed
                            raise OSError("connection closed by robot")
            except (OSError, Exception) as e:                  # serial errors are not OSError
                if not self._stop.is_set():
                    self.on_line(f"! link error: {e}")
                    threading.Thread(target=self.close, daemon=True).start()
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="ignore").strip()
                if line:
                    self.on_line(line)


def resolve_host(name, timeout=2.0):
    """'spidy.local' (mDNS) or an IP -> IP string. Windows 10+ resolves .local itself."""
    old = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout)
    try:
        return socket.gethostbyname(name)
    finally:
        socket.setdefaulttimeout(old)


class Pinger:
    """Measures round-trip latency with PING,n / PONG,n."""
    def __init__(self):
        self.sent = {}
        self.n = 0
        self.last_ms = None

    def make(self):
        self.n += 1
        self.sent[self.n] = time.perf_counter()
        return f"PING,{self.n}"

    def feed(self, line):
        if line.startswith("PONG,"):
            try:
                n = int(line.split(",")[1])
            except ValueError:
                return
            if n in self.sent:
                self.last_ms = (time.perf_counter() - self.sent.pop(n)) * 1000
