import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import time
import RPi.GPIO as GPIO
from ina219 import INA219

i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

ina = INA219(addr=0x42)

def read_battery_voltage():
    bus_voltage = ina.getBusVoltage_V()
    voltage = bus_voltage
    return voltage * 2

def display_voltage():
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    
    try:
        while True:
            voltage = read_battery_voltage()
            draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
            voltage_text = f"Battery: {voltage:.2f}V"
            draw.text((0, 0), voltage_text, font=font, fill=255)
            oled.image(image)
            oled.show()
            time.sleep(1)
    except KeyboardInterrupt:
        pass

def stop_display():
    oled.fill(0)
    oled.show()
    GPIO.cleanup()
    oled.poweroff()

if __name__ == "__main__":
    try:
        display_voltage()
    finally:
        stop_display()
