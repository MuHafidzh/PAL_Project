import RPi.GPIO as GPIO
import time

pin_tes = [5, 6, 12, 16]

# a 12 b 5 c 16 d 6

GPIO.setmode(GPIO.BCM)
for pin in pin_tes:
    GPIO.setup(pin, GPIO.IN)

try:
    while True:
        for pin in pin_tes:
            state = GPIO.input(pin)
            print(f"GPIO {pin}: {'HIGH' if state else 'LOW'}")
        print('-' * 40)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()