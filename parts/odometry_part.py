import RPi.GPIO as GPIO
import time
import math

class OdometryPart:
    def __init__(self, gpio_pin=13, ppr=20, wheel_radius_m=0.023, ema_alpha=0.3, timeout_s=0.5):
        self.gpio_pin = gpio_pin
        self.ppr = ppr
        self.wheel_radius_m = wheel_radius_m
        self.circumference_m = 2 * math.pi * wheel_radius_m
        self.m_per_pulse = self.circumference_m / float(ppr)

        self.ema_alpha = ema_alpha
        self.timeout_s = timeout_s

        self.pulse_count = 0
        self.speed_ms = 0.0
        self.last_update_time = time.time()
        self.last_pulse_time = time.time()
        self.total_distance_m = 0.0
        self.last_pin_state = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        try:
            self.last_pin_state = GPIO.input(self.gpio_pin)
        except:
            self.last_pin_state = 1

    def run(self):
        now = time.time()
        dt = now - self.last_update_time
        if dt <= 0:
            return self.speed_ms, 0.0

        try:
            current_pin_state = GPIO.input(self.gpio_pin)
        except:
            current_pin_state = self.last_pin_state if self.last_pin_state is not None else 1

        if self.last_pin_state == 1 and current_pin_state == 0:
            self.pulse_count += 1
            self.last_pulse_time = now

        self.last_pin_state = current_pin_state

        distance_m = self.pulse_count * self.m_per_pulse
        self.total_distance_m += distance_m

        speed_raw = distance_m / dt if dt > 0 else 0.0
        self.pulse_count = 0

        self.speed_ms = self.ema_alpha * speed_raw + (1.0 - self.ema_alpha) * self.speed_ms

        if (now - self.last_pulse_time) > self.timeout_s:
            self.speed_ms = 0.0

        self.last_update_time = now

        return self.speed_ms, 0.0

    def shutdown(self):
        try:
            GPIO.cleanup(self.gpio_pin)
        except:
            pass

