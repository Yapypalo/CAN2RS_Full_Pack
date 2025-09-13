import serial
import time

# ⚙️ НАСТРОЙКИ — измени под свой порт и скорость
SERIAL_PORT = 'COM3'    # ← Замени на свой порт (например, 'COM3', '/dev/ttyACM0')
BAUD_RATE = 1000000     # ← Скорость должна совпадать с настройкой CDC в STM32

try:
    # Открываем порт
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ Подключено к {SERIAL_PORT} со скоростью {BAUD_RATE} бод")

    # Сбрасываем буферы на всякий случай
    ser.flushInput()
    ser.flushOutput()

    print("📡 Ожидание данных от STM32...\n")

    while True:
        if ser.in_waiting > 0:  # Есть данные в буфере?
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:  # Если строка не пустая
                print(f"Получено: {line}")

except serial.SerialException as e:
    print(f"❌ Ошибка подключения к порту {SERIAL_PORT}: {e}")

except KeyboardInterrupt:
    print("\n⚠️  Прервано пользователем")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("🔌 Порт закрыт")