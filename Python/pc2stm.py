"""
Star Sensor Emulator

Эмулирует звёздный датчик по COM3 @1 000 000 baud.

Сценарии (нажмите '1', '2' или 'q'):
  1 – отправить STM "AREYOULIVE?", ждать ответ "IAMALIVE!"
  2 – отправить STM "ISSENSOROK?", STM пошлёт запрос 0x04,
      эмулируем ответ, ждём "SENSOR_WORKS!"
"""

import threading
import struct
import serial
import time
import sys

# Константы пакета
PREAMBLE = 0xAA
CRC_POLY = 0x8005
CRC_INIT = 0xFFFF

class StarSensorEmulator:
    def __init__(self, port='COM3', baud=1_000_000):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.buf = bytearray()
        self.running = True
        self.lock = threading.Lock()
        self.current_scenario = None
        self._text_buf = ''
        print(f"Opened {port} @ {baud} baud")

    def start(self):
        threading.Thread(target=self.read_loop, daemon=True).start()
        try:
            while self.running:
                cmd = input("Enter scenario (1,2) or q to quit: ").strip()
                if cmd.lower() == 'q':
                    self.running = False
                elif cmd == '1':
                    self.current_scenario = 1
                    self.send_text("AREYOULIVE?")
                elif cmd == '2':
                    self.current_scenario = 2
                    self.send_text("ISSENSOROK?")
                else:
                    print("Unknown command.")
        except KeyboardInterrupt:
            pass
        finally:
            print("Shutting down...")
            self.ser.close()

    def read_loop(self):
        while self.running:
            n = self.ser.in_waiting or 1
            data = self.ser.read(n)
            if not data:
                continue

            # 1) Собираем и парсим бинарные фреймы
            self.buf.extend(data)
            self._parse_frames()

            # 2) Фильтруем и выводим текстовую часть
            try:
                txt = data.decode('utf-8')
            except UnicodeDecodeError:
                txt = data.decode('utf-8', errors='ignore')

            for line in txt.splitlines(keepends=True):
                if line.startswith("DBG:"):
                    # heartbeat
                    sys.stdout.write(line)
                elif all(32 <= ord(c) < 127 or c in '\r\n' for c in line):
                    # выводим только чистый ASCII
                    sys.stdout.write(line)
                    self._check_scenario_text(line)
            sys.stdout.flush()

    def _parse_frames(self):
        while True:
            if len(self.buf) < 8 or self.buf[0] != PREAMBLE:
                return
            length = self.buf[6] | (self.buf[7] << 8)
            total  = 8 + length + 2
            if len(self.buf) < total:
                return
            frame = bytes(self.buf[:total])
            del self.buf[:total]
            self.handle_frame(frame)

    def handle_frame(self, frame: bytes):
        hdr      = frame[:8]
        cmd      = hdr[5]
        length   = hdr[6] | (hdr[7] << 8)
        crc_recv = frame[8+length] | (frame[9+length] << 8)
        crc_calc = self.crc16(frame[:-2])

        if cmd == 0x04:
            print(f"\n[RX cmd=0x04] crc_ok={crc_calc==crc_recv}")
            # эмулируем задержку и ответ датчика
            time.sleep(0.065)
            x, y, zenith, azimuth, status = 1.23, 4.56, 78.9, 12.34, 0
            payload = struct.pack('<4fH', x, y, zenith, azimuth, status)
            self.send_frame(dest=hdr[2], src=hdr[1], cmd=cmd, data=payload)

    def send_text(self, text: str):
        packet = (text + '\n').encode('utf-8')
        print(f"\n>> send text: {text}")
        self.ser.write(packet)

    def send_frame(self, dest: int, src: int, cmd: int, data: bytes):
        header = bytearray([
            PREAMBLE,
            dest,
            src,
            0x00,  # reserved
            0x00,  # reserved
            cmd,
            len(data) & 0xFF,
            (len(data) >> 8) & 0xFF
        ])
        frame = header + data
        crc   = self.crc16(frame)
        frame += struct.pack('<H', crc)
        with self.lock:
            print(f"\n<< send frame: cmd=0x{cmd:02X}, len={len(data)}, crc=0x{crc:04X}")
            self.ser.write(frame)

    def _check_scenario_text(self, line: str):
        self._text_buf += line
        if len(self._text_buf) > 200:
            self._text_buf = self._text_buf[-200:]

        if self.current_scenario == 1 and "IAMALIVE!" in self._text_buf:
            print("\n> Scenario 1 OK")
            self.current_scenario = None
        elif self.current_scenario == 2 and "SENSOR_WORKS!" in self._text_buf:
            print("\n> Scenario 2 OK")
            self.current_scenario = None

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = CRC_INIT
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                crc = ((crc << 1) ^ CRC_POLY) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

if __name__ == '__main__':
    emulator = StarSensorEmulator(port='COM3', baud=1_000_000)
    emulator.start()