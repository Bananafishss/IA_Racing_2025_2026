from donkeycar.parts.actuators import PWMThrottle
import time

motor = PWMThrottle(channel=0)
motor.update(0.25)
time.sleep(3)
motor.update(0.0)
motor.shutdown()
