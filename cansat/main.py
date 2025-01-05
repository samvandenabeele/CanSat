# from machine import Pin
# from utime import sleep

# pin = Pin("LED", Pin.OUT)

# print("LED starts flashing...")
# while True:
#     try:
#         pin.toggle()
#         sleep(1) # sleep 1sec
#     except KeyboardInterrupt:
#         break
# pin.off()
# print("Finished.")

from sensors import BMP280, RFM69HCW, MPU6050

bmp280 = BMP280()
radio = RFM69HCW()

while True:
    temp = bmp280.temperature
    press = bmp280.pressure

    radio.transmit(data=f"temperature: ")

    