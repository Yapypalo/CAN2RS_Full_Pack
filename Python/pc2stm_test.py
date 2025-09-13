#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading, struct, serial, time, sys

PREAMBLE = 0xAA
CRC_POLY = 0x8005
CRC_INIT = 0xFFFF

class Emulator:
    def __init__(self):
        self.ser = serial.Serial('COM3', 1000000, timeout=0.05)
        self.buf = bytearray()
        self.running = True
        self.scenario = None
        self.txtbuf = ''
        print("Opened COM3 @ 1000000 baud")

    def start(self):
        threading.Thread(target=self.read_loop, daemon=True).start()
        while self.running:
            cmd = input()  # берём всё, что ввёл пользователь
            if cmd.strip().lower() == 'q':
                self.running = False
                break
            # сразу шлём введённый текст
            self.send_text(cmd)
            # сохраняем сценарий только по точному совпадению
            if cmd.strip() == '1':
                self.scenario = 1
            elif cmd.strip() == '2':
                self.scenario = 2

    def read_loop(self):
        while self.running:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data: continue
            # печатаем текст
            sys.stdout.write(data.decode('utf-8', errors='ignore'))
            sys.stdout.flush()
            # накапливаем в буфер для сценариев
            self.txtbuf += data.decode('utf-8', errors='ignore')
            if self.scenario == 1 and "Работаю!" in self.txtbuf:
                print("\nScenario 1 OK"); self.scenario = None
            if self.scenario == 2 and "И датчик работает" in self.txtbuf:
                print("\nScenario 2 OK"); self.scenario = None
            # обрабатываем фреймы
            self.buf.extend(data)
            self.parse_frames()

    def parse_frames(self):
        while len(self.buf) >= 8 and self.buf[0] == PREAMBLE:
            length = self.buf[6] | (self.buf[7]<<8)
            tot = 8 + length + 2
            if len(self.buf) < tot: break
            frame = bytes(self.buf[:tot]); del self.buf[:tot]
            self.handle_frame(frame)

    def handle_frame(self, frame):
        cmd = frame[5]
        length = frame[6] | (frame[7]<<8)
        crc_recv = frame[8+length] | (frame[9+length]<<8)
        crc_calc = self.crc16(frame[:-2])
        print(f"\n[RX] cmd=0x{cmd:02X}, crc_ok={crc_calc==crc_recv}")
        if cmd == 0x04:
            time.sleep(0.065)
            payload = struct.pack('<4fH',1.23,4.56,78.9,12.34,0)
            self.send_frame(frame[2], frame[1], cmd, payload)

    def send_text(self, text):
        print(f"\n>> send_text: {repr(text)}")
        self.ser.write(text.encode('utf-8'))

    def send_frame(self, dst, src, cmd, data):
        header = bytearray([PREAMBLE, dst, src,0,0, cmd, len(data)&0xFF, len(data)>>8])
        frame = header + data
        crc = self.crc16(frame); frame += struct.pack('<H',crc)
        print(f"\n[TX] cmd=0x{cmd:02X}, crc=0x{crc:04X}")
        self.ser.write(frame)

    @staticmethod
    def crc16(d):
        crc=CRC_INIT
        for b in d:
            crc ^= b<<8
            for _ in range(8):
                crc = (crc<<1)^CRC_POLY if crc&0x8000 else crc<<1
                crc&=0xFFFF
        return crc

if __name__=='__main__':
    Emulator().start()
