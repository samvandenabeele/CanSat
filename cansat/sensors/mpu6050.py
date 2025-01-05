from imu import mpu6050
from time import sleep
from machine import Pin, I2C


i2c = I2C(0, sda=Pin(18), scl=Pin(19), freq=400000)
imu = mpu6050(i2c)

class MPU6050:
    def __init__(self):
        self.i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
        self.imu = mpu6050(self.i2c)

    @property
    def temperature(self):
        return self.imu.temperature
    
    @property
    def accel(self):
        return [self.imu.accel.x, self.imu.accel.y, self.imu.accel.z]
    
    @property
    def gyro(self):
        return [self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z]

# while True:
#    ax=round(imu.accel.x,2)
#    ay=round(imu.accel.y,2)
#    az=round(imu.accel.z,2)
#    gx=round(imu.gyro.x)
#    gy=round(imu.gyro.y)
#    gz=round(imu.gyro.z)
#    tem=round(imu.temperature,2)
#    print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\r")
#    sleep(0.2)
