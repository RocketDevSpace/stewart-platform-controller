import serial
import threading
import time


class SerialSender:
    def __init__(self, port="COM4", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.read_thread = None

        # Callback function for received lines
        self.on_receive_callback = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # allow Arduino reset
            print(f"[SERIAL] Connected to {self.port} at {self.baud} baud")

            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

        except Exception as e:
            print(f"[SERIAL ERROR] Could not connect: {e}")
            self.ser = None

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[SERIAL] Disconnected")

    def send_command(self, cmd):
        t0 = time.perf_counter()
        if self.ser is None or not self.ser.is_open:
            print("[SERIAL ERROR] Not connected")
            return

        try:
            self.ser.write(cmd)
        except Exception as e:
            print(f"[SERIAL ERROR] Send failed: {e}")
        t1 = time.perf_counter()
        print(f"Servo write: {(t1-t0)*100:.2f} ms")

    def set_receive_callback(self, callback_func):
        """
        callback_func should accept a single string argument (line received).
        """
        self.on_receive_callback = callback_func

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
                                print("[SERIAL RX]", line)

            except Exception as e:
                print(f"[SERIAL ERROR] Read failed: {e}")
                break
