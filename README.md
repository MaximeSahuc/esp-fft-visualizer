# ESP32/8266 FFT Visualizer

## Demo
<video width="300px" src="https://github.com/user-attachments/assets/7c88b3fd-4415-4396-ae51-c1a76146e295"></video>

## Libraries used
- Adafruit_SSD1306
- Adafruit_GFX
- arduinoFFT

## Hardware needed
- ESP32 or ESP8266
- SSD1306 based OLED display
- One 1K resistor
- One jack cable or splitter

## Hardware setup
1. Connect the OLED screen to the I2C pins of the microcontroller (21, 22 for the ESP32, or 4, 5 for the ESP8266 D1 Mini board).
2. Connect the jack cable's ground to the microcontroller's ground, and connect the left OR right channel to the analog input pin specified in the code.
3. Connect the 1K resistor between the ground and the audio channel you chose to prevent noise.
4. Flash the microcontroller with the code, making sure all the pin declarations are correct.
