import machine
import time

class TMP36:
    def __init__(self, pin=26):
        self.adc = machine.ADC(machine.Pin(pin))
        self.conversion_factor = 3.3 / 65535

    @property
    def temperature(self):
        reading = self.adc.read_u16() * self.conversion_factor
        temperature_c = (reading - 0.5) * 100
        return temperature_c

if __name__ == "__main__":
    sensor = TMP36(pin=26)
    while True:
        temp = sensor.temperature
        print("Temperature: {:.2f} C".format(temp))
        time.sleep(1)