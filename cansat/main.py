
from sensors import BMP280, RFM69HCW, MPU6050

bmp280 = BMP280()
radio = RFM69HCW()

while True:
    temp = bmp280.temperature
    press = bmp280.pressure

    radio.transmit(data=f"temperature: ")

    