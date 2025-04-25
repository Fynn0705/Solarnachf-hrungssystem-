# Angenommen, du hast den INA219-Treiber zur Verfügung
from ina219 import INA219
i2c = I2C(0, scl=Pin(5), sda=Pin(4))  # Verwende die entsprechenden Pins für ESP32
ina = INA219(i2c)