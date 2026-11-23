import serial
import threading

PORT = "COM5"
BAUDRATE = 115200

stm32 = serial.Serial(PORT, BAUDRATE, timeout=1)

def read_stm32():
    while True:
        try:
            message = stm32.readline().decode(
                "utf-8", errors="replace"
            ).strip()

            if message:
                print("\n[STM32]:", message)

        except Exception as e:
            print("UART Error:", e)
            break

threading.Thread(
    target=read_stm32,
    daemon=True
).start()

print("QR Scanner Bridge Ready")
print("Scan your QR Code...")

try:
    while True:
        qr_code = input("QR: ").strip()

        if qr_code:
            stm32.write(
                (qr_code + "\r\n").encode("utf-8")
            )
            print("Sent to STM32:", qr_code)

except KeyboardInterrupt:
    print("Program stopped")

finally:
    stm32.close()
