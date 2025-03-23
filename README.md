# CanSat Code
This is repo to store code for Raspberry Pi Pico 2 (satellite) and ESP8266 (ground station) in our CanSat competition for our team [SteinSat](https://www.instagram.com/steinsat2025/).
We are developing it using [Arduino IDE 2.x](https://github.com/arduino/arduino-ide) and its [libraries](#used-libraries). To display and store data we have [node-red dashboard](https://github.com/George-Doge/SteinSat_Node-RED).


## Project Directory Structure
- `ground_station` contains code used in our ground station using ESP8266
- `satellite` contains code used onboard our satellite using Raspberry Pi Pico 2
- `esp32_cam` contains code for ESP32 with camera
- `reset_counter` in the `esp32_cam` directory contains camera counter reset code
- `archive` contains code that is no longer used but might be useful for later or in case of reversing changes


## Used libraries
- [Adafruit BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Arduino LoRa](https://github.com/sandeepmistry/arduino-LoRa)
- [Arduino JSON](https://github.com/arduino-libraries/Arduino_JSON)

### Useful links
- [Raspberry Pi Pico 2 Arduino IDE setup guide](https://randomnerdtutorials.com/programming-raspberry-pi-pico-w-arduino-ide/)
- [Raspberry Pi Pico 2 Pinout](https://pico2.pinout.xyz/)
- [ESP8266 Arduino IDE setup guide](https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/)
- [ESP8266 Pinout](https://lastminuteengineers.com/esp8266-pinout-reference/)
- [ESP32 with Camera Arduino IDE setup guide](https://randomnerdtutorials.com/program-upload-code-esp32-cam/)
- [ESP32 with Camera Pinout](https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/) \
The setup guides also contain links to other useful libraries for boards
