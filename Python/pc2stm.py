"""
Star Sensor Emulator
Emulates a star tracker sensor over serial COM3 @1 000 000 baud.

Сценарии (нажмите '1', '2', ... в консоли):
  1 – отправить STM текст "Работаешь?", ждать ответ "Работаю!"
  2 – отправить STM текст "А датчик работает?", STM запросит датчику команду 0x04,
      эмулируем ответ, ждем от STM текст "И датчик работает"
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
        print(f"Opened {port} @ {baud} baud")

    def start(self):
        # Запускаем фоновое чтение
        threading.Thread(target=self.read_loop, daemon=True).start()
        # Обработка ввода пользователя в основном потоке
        try:
            while self.running:
                cmd = input("Введите сценарий (1,2) или q для выхода: ").strip()
                if cmd.lower() == 'q':
                    self.running = False
                    break
                elif cmd == '1':
                    self.current_scenario = 1
                    self.send_text("Работаешь?")
                elif cmd == '2':
                    self.current_scenario = 2
                    self.send_text("А датчик работает?")
                else:
                    print("Неизвестная команда.")
        except KeyboardInterrupt:
            self.running = False
        finally:
            print("Shutting down...")
            self.ser.close()

    def read_loop(self):
        while self.running:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue
            self.buf.extend(data)
            # Парсим фреймы и текст
            while self.buf:
                if self.buf[0] == PREAMBLE and len(self.buf) >= 8:
                    # Как минимум заголовок
                    cmd = self.buf[5]
                    length = self.buf[6] | (self.buf[7] << 8)
                    frame_len = 8 + length + 2
                    if len(self.buf) < frame_len:
                        break  # ждем полный фрейм
                    frame = self.buf[:frame_len]
                    self.buf = self.buf[frame_len:]
                    self.handle_frame(frame)
                else:
                    # Текстовый байт
                    b = self.buf.pop(0)
                    try:
                        ch = bytes([b]).decode('utf-8', errors='ignore')
                        if ch:
                            sys.stdout.write(ch)
                            sys.stdout.flush()
                            self.check_scenario_text(ch)
                    except:
                        pass

    def handle_frame(self, frame: bytes):
        # Логируем принятый фрейм
        hdr = frame[:8]
        cmd = hdr[5]
        length = hdr[6] | (hdr[7] << 8)
        data = frame[8:8+length]
        crc_recv = frame[8+length] | (frame[9+length] << 8)
        crc_calc = self.crc16(frame[:-2])
        print(f"\nReceived frame: cmd=0x{cmd:02X}, len={length}, crc_ok={crc_calc==crc_recv}")
        print(frame.hex(' '))

        # Эмулируем только команду 0x04 (реальное фото + расчет)
        if cmd == 0x04:
            # Задержка, имитирующая время работы матрицы
            time.sleep(0.065)
            # Придуманные координаты
            x, y, zenith, azimuth = 1.23, 4.56, 78.9, 12.34
            status = 0
            payload = struct.pack('<4fH', x, y, zenith, azimuth, status)
            self.send_frame(dest=hdr[2], src=hdr[1], cmd=cmd, data=payload)

    def send_text(self, text: str):
        data = text.encode('utf-8')
        print(f"\n>> send text: {text}")
        self.ser.write(data)

    def send_frame(self, dest: int, src: int, cmd: int, data: bytes):
        length = len(data)
        header = bytearray([
            PREAMBLE,
            dest,
            src,
            0x00,  # резерв
            0x00,  # резерв
            cmd,
            length & 0xFF,
            (length >> 8) & 0xFF
        ])
        frame = header + data
        crc = self.crc16(frame)
        frame += struct.pack('<H', crc)
        with self.lock:
            print(f"\n<< send frame: cmd=0x{cmd:02X}, len={length}, crc=0x{crc:04X}")
            print(frame.hex(' '))
            self.ser.write(frame)

    def check_scenario_text(self, ch: str):
        """Отслеживаем вхождение ожидаемой фразы для текущего сценария."""
        # собираем последние 100 символов
        if not hasattr(self, '_text_buf'):
            self._text_buf = ''
        self._text_buf += ch
        if len(self._text_buf) > 200:
            self._text_buf = self._text_buf[-200:]
        if self.current_scenario == 1 and "Работаю!" in self._text_buf:
            print("\n> Сценарий 1 завершен успешно.")
            self.current_scenario = None
        elif self.current_scenario == 2 and "И датчик работает" in self._text_buf:
            print("\n> Сценарий 2 завершен успешно.")
            self.current_scenario = None

    @staticmethod
    def crc16(data: bytes) -> int:
        """CRC-16 (poly=0x8005, init=0xFFFF), no reflection."""
        crc = CRC_INIT
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

if __name__ == '__main__':
    emulator = StarSensorEmulator(port='COM3', baud=1_000_000)
    emulator.start()