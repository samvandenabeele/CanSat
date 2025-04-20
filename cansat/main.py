from adafruit_sgp30 import Adafruit_SGP30
from rfm69 import RFM69
from imu import MPU6050
from bme280 import BME280, BME280_I2CADDR
from machine import I2C, Pin, SPI, UART
from servo import Servo
import csv
import json
import time
from Kalman import KalmanFilter
from microGPS import MicropyGPS

# initialize the sensors

uart = UART(1, baudrate=115200, tx=Pin(21), rx=Pin(22))

i2c1 = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
i2c0 = I2C(1, scl=Pin(9), sda=Pin(8), freq=100000)

accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []
alt = []

sg90 = Servo(12)
sg90.move(0)
sgp30 = Adafruit_SGP30(i2c1)
mpu6050 = MPU6050(i2c0)
mpu6050.accel_range = 1
mpu6050.gyro_range = 1
bmp280 = BME280(i2c=i2c0, address=BME280_I2CADDR)
kf_ax = KalmanFilter()
kf_ay = KalmanFilter()
kf_az = KalmanFilter()
alt_baseline = 1032.0


spi = SPI(0, baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB)
nss = Pin( 5, Pin.OUT, value=True )
rst = Pin( 3, Pin.OUT, value=False )

FREQ           = 433.1
ENCRYPTION_KEY = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
NODE_ID        = 120 # ID of this node
BASESTATION_ID = 100 # ID of the node (base station) to be contacted

rfm = RFM69( spi=spi, nss=nss, reset=rst )
rfm.frequency_mhz = FREQ

gps = MicropyGPS()


def read_gps() -> tuple:
    timeout = time.ticks_add(time.ticks_ms(), 30000)  # 30 second timeout
    while time.ticks_diff(timeout, time.ticks_ms()) > 0:
        if uart.any():
            c = uart.read(1)
            try:
                gps.update(c.decode('utf-8'))
            except:
                continue

        if gps.latitude and gps.longitude and gps.fix_stat:
            lat = gps.latitude[0] + gps.latitude[1] / 60.0
            lon = gps.longitude[0] + gps.longitude[1] / 60.0
            if gps.latitude[2] == 'S':
                lat = -lat
            if gps.longitude[2] == 'W':
                lon = -lon
            return lat, lon

        time.sleep(0.1)

    raise TimeoutError("Failed to get GPS fix within timeout period.")

def mpu_add_data():
    accel_x.append(mpu6050.accel.x)
    accel_y.append(mpu6050.accel.y)
    accel_z.append(mpu6050.accel.z)
    gyro_x.append(mpu6050.gyro.x)
    gyro_y.append(mpu6050.gyro.y)
    gyro_z.append(mpu6050.gyro.z)

    ax_raw = sum(accel_x) / len(accel_x)
    ay_raw = sum(accel_y) / len(accel_y)
    az_raw = sum(accel_z) / len(accel_z)
    gx = sum(gyro_x) / len(gyro_x)
    gy = sum(gyro_y) / len(gyro_y)
    gz = sum(gyro_z) / len(gyro_z)

    ax = kf_ax.update(ax_raw)
    ay = kf_ay.update(ay_raw)
    az = kf_az.update(az_raw)


    if len(accel_x) > 10:
        accel_x.pop(0)
        accel_y.pop(0)
        accel_z.pop(0)
        gyro_x.pop(0)
        gyro_y.pop(0)
        gyro_z.pop(0)

    return ax, ay, az, gx, gy, gz, ax_raw, ay_raw, az_raw

def is_sorted(lst):
    return all(a <= b for a, b in zip(lst, lst[1:]))

def is_equal(lst):
    return all(a == b for a, b in zip(lst, lst[1:]))

def process_data():
    ax, ay, az, gx, gy, gz, ax_raw, ay_raw, az_raw = mpu_add_data()
    temp, press, hum = bmp280.raw_values
    alt = (alt_baseline - press) * 8.3
    sgp30.set_iaq_rel_humidity(hum, temp)
    iaq_data = sgp30.iaq_measure()
    lat, lon = read_gps()
    if iaq_data is not None:
        co2eq, tvoc = iaq_data
    else:
        co2eq, tvoc = 0, 0  # Default values if iaq_measure() returns None

    alt.append(alt)
    if len(alt) > 10:
        alt.pop(0)

    # print("Temperature: %0.1f C" % temp)
    # print("Pressure: %0.1f hPa" % press)
    # print("Altitude: %0.2f m" % alt)
    # print("Humidity: %0.1f %%" % hum)
    # print("CO2eq: %d ppm" % co2eq)
    # print("TVOC: %d ppb" % tvoc)
    # print("Accel: %0.2f, %0.2f, %0.2f" % (ax, ay, az))
    # print("Gyro: %0.2f, %0.2f, %0.2f" % (gx, gy, gz))
    # print("")

    with open("data.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow([temp, press, alt, hum, co2eq, tvoc, ax, ay, az, gx, gy, gz, ax_raw, ay_raw, az_raw, lat, lon])

    data = json.dumps({"temp": temp,
                       "press": press,
                       "alt": alt,
                       "hum": hum,
                       "co2eq": co2eq,
                       "tvoc": tvoc,
                       "ax": ax,
                       "ay": ay,
                       "az": az,
                       "gx": gx,
                       "gy": gy,
                       "gz": gz,
                       "lat": lat,
                       "lon": lon})

    rfm.send(node=NODE_ID, data=data.encode('utf-8'))
    return

while True:
    process_data()
    if is_sorted(alt):
        # turn servo to take air sample
        sg90.move(180)
        servo_time = time.time()
        break

while True:
    process_data()
    if time.time() - servo_time > 10:
        # turn servo to close air sample
        sg90.move(0)
        break  

while True:
    process_data()
    if is_equal(alt):
        break
    # print("Sent message")
    # print("")
