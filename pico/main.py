import time
from neopixel import Neopixel

numpix = 11
strip = Neopixel(numpix, 0, 2, "GRB")

color = (10, 70, 255)
strip.show(color)
