#!/bin/bash
# Collecter les pilotes nécessaires
rm bme280.py
rm rfm69.py
wget https://raw.githubusercontent.com/mchobby/esp8266-upy/master/bme280-bmp280/bme280.py
wget https://raw.githubusercontent.com/mchobby/esp8266-upy/master/rfm69/lib/rfm69.py
