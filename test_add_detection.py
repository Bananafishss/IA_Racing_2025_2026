import RPi.GPIO as GPIO
import time

PIN = 13

def callback(channel):
    print(f"Edge detected on channel {channel}")

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(PIN, GPIO.RISING, callback=callback)
print(f"Edge detection added to pin {PIN}. Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
