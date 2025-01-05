from machine import Pin, I2C
from bmp280 import BMP280I2C

class BMP280:
    def __init__(self, address=0x77):
        self.i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        self.bmp280 = BMP280I2C(address, self.i2c)

    @property
    def temperature(self):
        readout = self.bmp280.measurements
        return readout['t']
    
    @property
    def pressure(self):
        readout = self.bmp280.measurements
        return readout['p']
