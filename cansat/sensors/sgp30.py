from machine import I2C, Pin
import time

class SGP30:
    def __init__(self, address=0x58):
        """
        Initialize the SGP30 sensor.

        :param i2c: Initialized I2C object.
        :param address: I2C address of the SGP30 (default: 0x58).
        """
        self.i2c = I2C(0, scl=Pin(1), sda=Pin(0), address=address)
        self.address = address
        self.initialize()

    def write_command(self, command, params=None):
        """
        Write a command to the sensor.

        :param command: The command to send (2 bytes).
        :param params: Optional parameters (list of bytes).
        """
        data = [command >> 8, command & 0xFF]
        if params:
            for param in params:
                data.append(param)
                data.append(self._crc8([param]))
        self.i2c.writeto(self.address, bytes(data))

    def read_data(self, num_bytes):
        """
        Read data from the sensor.

        :param num_bytes: Number of bytes to read.
        :return: List of bytes (without CRCs).
        """
        raw = self.i2c.readfrom(self.address, num_bytes * 3 // 2)
        result = []
        for i in range(0, len(raw), 3):
            if self._crc8(raw[i:i+2]) != raw[i+2]:
                raise ValueError("CRC mismatch")
            result.append((raw[i] << 8) | raw[i+1])
        return result

    def initialize(self):
        """
        Initialize the sensor and start air quality measurements.
        """
        self.write_command(0x2003)
        time.sleep(0.01)

    def read_air_quality(self):
        """
        Read TVOC and eCO2 values.

        :return: Tuple (eCO2, TVOC).
        """
        self.write_command(0x2008)
        data = self.read_data(2)
        return data[0], data[1]

    def set_humidity(self, absolute_humidity):
        """
        Set the absolute humidity for compensation.

        :param absolute_humidity: Humidity in mg/m^3 (integer).
        """
        params = [(absolute_humidity >> 8) & 0xFF, absolute_humidity & 0xFF]
        self.write_command(0x2061, params)

    def get_baseline(self):
        """
        Retrieve baseline values for TVOC and eCO2.

        :return: Tuple (eCO2_baseline, TVOC_baseline).
        """
        self.write_command(0x2015)
        data = self.read_data(2)
        return data[0], data[1]

    def set_baseline(self, eCO2_baseline, TVOC_baseline):
        """
        Set baseline values for TVOC and eCO2.

        :param eCO2_baseline: eCO2 baseline value.
        :param TVOC_baseline: TVOC baseline value.
        """
        params = [
            (eCO2_baseline >> 8) & 0xFF, eCO2_baseline & 0xFF,
            (TVOC_baseline >> 8) & 0xFF, TVOC_baseline & 0xFF
        ]
        self.write_command(0x201E, params)

    def measure_raw(self):
        """
        Measure raw H2 and ethanol signals.

        :return: Tuple (H2_signal, Ethanol_signal).
        """
        self.write_command(0x2050)
        data = self.read_data(2)
        return data[0], data[1]

    def soft_reset(self):
        """
        Perform a soft reset on the sensor.
        """
        self.write_command(0x0006)
        time.sleep(0.01)

    def _crc8(self, data):
        """
        Calculate the CRC8 checksum for the given data.

        :param data: List of bytes.
        :return: CRC8 checksum.
        """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

# Example Usage:
# from machine import Pin, I2C
# i2c = I2C(0, scl=Pin(22), sda=Pin(21))
# sgp30 = SGP30(i2c)
# eCO2, TVOC = sgp30.read_air_quality()
# print("eCO2:", eCO2, "ppm, TVOC:", TVOC, "ppb")
