from sensors import BMP280, RFM69HCW, MPU6050 #importing sensors and radio transmitter
import time


bmp_sensor = BMP280()
mpu_sensor = MPU6050()

radio = RFM69HCW()

start_time = time.time()

while True:
    current_time = time.time()
    elapsed_time_ms = (current_time - start_time) * 1000  # Elapsed time in milliseconds


    if 100 <= elapsed_time_ms < 150 :
        bmp_temp = bmp_sensor.temperature
        print(f"BMP280 Temp: {bmp_temp} C")

    if 175 <= elapsed_time_ms < 250:
        accel_data = mpu_sensor.accel
        gyro_data = mpu_sensor.gyro
        mpu_temp = mpu_sensor.temperature


        message = f"Accel (x,y,z): {accel_data[0]},{accel_data[1]},{accel_data[2]} | " f"Gyro (x,y,z): {gyro_data[0]},{gyro_data[1]},{gyro_data[2]} | "             f"IMU Temp: {mpu_temp} C"

        radio.transmit(message)
        print(f"Sent: {message}")


        start_time = time.time()

