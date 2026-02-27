import serial
import threading
import time
from collections import deque

from stewart_control.config import DEBUG_LEVEL, LOG_EVERY_N, SERIAL_QUEUE_MAX


class SerialSender:
    def __init__(self, port="COM4", baud=115200, max_queue_size=SERIAL_QUEUE_MAX):
        self.port = port
        self.baud = baud
        self.max_queue_size = max_queue_size
        self.ser = None
        self.running = False
        self.read_thread = None
        self.write_thread = None
        self.connect_thread = None

        # Callback function for received lines
        self.on_receive_callback = None

        self._queue = deque()
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()
        self._connecting = False
        self._send_counter = 0

    def connect(self):
        if self._connecting:
            return
        if self.ser is not None and self.ser.is_open:
            return
        self._connecting = True
        self.connect_thread = threading.Thread(target=self._connect_worker, daemon=True)
        self.connect_thread.start()

    def _connect_worker(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # allow Arduino reset without blocking GUI

            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.write_thread = threading.Thread(target=self._write_loop, daemon=True)
            self.read_thread.start()
            self.write_thread.start()
            self._log(f"[SERIAL] Connected to {self.port} at {self.baud} baud", level=1)

        except Exception as e:
            self._log(f"[SERIAL ERROR] Could not connect: {e}", level=1)
            self.ser = None
        finally:
            self._connecting = False

    def disconnect(self):
        self.running = False
        self._queue_event.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self._log("[SERIAL] Disconnected", level=1)

    def send_command(self, cmd):
        self.enqueue_command(cmd, policy="fifo")

    def enqueue_command(self, cmd, policy="fifo"):
        if isinstance(cmd, str):
            cmd = cmd.encode()
        if not isinstance(cmd, (bytes, bytearray)):
            raise TypeError("Command must be bytes, bytearray, or str.")

        if self.ser is None or not self.ser.is_open:
            self._log("[SERIAL ERROR] Not connected", level=1)
            return

        with self._queue_lock:
            if policy == "latest":
                self._queue.clear()
            elif policy != "fifo":
                raise ValueError("policy must be 'fifo' or 'latest'")

            if len(self._queue) >= self.max_queue_size:
                self._queue.popleft()

            self._queue.append(bytes(cmd))

        self._queue_event.set()

    def set_receive_callback(self, callback_func):
        """
        callback_func should accept a single string argument (line received).
        """
        self.on_receive_callback = callback_func

    def _write_loop(self):
        while self.running:
            self._queue_event.wait(timeout=0.1)

            while self.running:
                with self._queue_lock:
                    if not self._queue:
                        self._queue_event.clear()
                        break
                    cmd = self._queue.popleft()

                if self.ser is None or not self.ser.is_open:
                    break

                t0 = time.perf_counter()
                try:
                    self.ser.write(cmd)
                except Exception as e:
                    self._log(f"[SERIAL ERROR] Send failed: {e}", level=1)
                    break
                t1 = time.perf_counter()
                self._send_counter += 1
                if DEBUG_LEVEL >= 2 and (self._send_counter % LOG_EVERY_N == 0):
                    self._log(f"Servo write: {(t1 - t0) * 1000:.2f} ms", level=2)

    def _read_loop(self):
        buffer = ""

        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(128).decode("utf-8", errors="ignore")

                if data:
                    buffer += data

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if line:
                            if self.on_receive_callback:
                                self.on_receive_callback(line)
                            else:
                                self._log(f"[SERIAL RX] {line}", level=2)

            except Exception as e:
                self._log(f"[SERIAL ERROR] Read failed: {e}", level=1)
                break

    @staticmethod
    def _log(msg, level=1):
        if DEBUG_LEVEL >= level:
            print(msg)
