from sensors import BMP280, RFM69HCW, MPU6050
import time


bmp_sensor = BMP280()
radio = RFM69HCW()

while True:
    temp = bmp_sensor.temperature
    press = bmp_sensor.pressure

    radio.transmit(data=f"temperature: ")

