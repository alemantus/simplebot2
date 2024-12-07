import time
import board
from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
ss = seesaw.Seesaw(i2c)

NEOPIXEL_PIN = 14  # Can be any pin
NEOPIXEL_NUM = 16  # No more than 60 pixels!

pixels = neopixel.NeoPixel(ss, NEOPIXEL_PIN, NEOPIXEL_NUM)
pixels.brightness = 0.3  # Not so bright!

color_offset = 0  # Start at red

# Cycle through all colors along the ring
while True:
    for i in range(NEOPIXEL_NUM):
        rc_index = (i * 256 // NEOPIXEL_NUM) + color_offset
        pixels[i] = colorwheel(rc_index & 255)
    color_offset += 1
    time.sleep(0.01)