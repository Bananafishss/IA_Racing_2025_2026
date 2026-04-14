import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def set_angle(servo, angle):
    servo.angle = angle
    print(f'Servo angle: {angle}')

def set_pulse_width(servo, pulse_width):
    fraction = (pulse_width - 1000) / (2000 - 1000)
    servo.fraction = fraction

def test_position():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 40

    s = servo.Servo(
        pca.channels[1],
        min_pulse=1000,
        max_pulse=2000
    )
    
    angles = [90, 80, 70, 70, 80, 90]
    try:
        while True:
            for angle in angles:
                set_angle(s, angle)
                time.sleep(1)
    except KeyboardInterrupt:
        pca.deinit()
        print("Servo control stopped")

if __name__ == "__main__":
    test_position()
