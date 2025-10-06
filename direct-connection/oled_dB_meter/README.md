# OLED dB Meter (senseBox MCU S2)

This project provides a real-time sound level measurement and visualization tool using the senseBox MCU S2, capable of displaying both raw SPL and human-perceived loudness (dBA) on an OLED screen.
It measures, visualizes, and records loudness using a senseBox MCU S2, ICS43434 I2S microphone, DFRobot Sound Level Meter (SLM), and an SSD1306 OLED display.

The system captures two types of sound measurements:
- dBA: Human-perceived loudness from the DFRobot Sound Level Meter.
- dB SPL (Z-weighted): Raw sound pressure level derived from the ICS43434 microphone.

The ICS43434 does not apply A-weighting. To approximate dBA from the mic data, a digital A-weighting filter would need to be implemented.

## Setup
### Hardware
- senseBox MCU S2
- ICS-43434 microphone
- DFRobot Gravity Analog Sound Level Meter
- SSD1306 OLED display
- Micro SD Card

### Wiring
- ICS43434 mic in QWIIC adapters "GPIO A" and "GPIO B"
- Sound level meter in "GPIO C"
- OLED in first (left-side) "I2C" QWIIC connector

Note: The ICS mic has 4 data wires, so it uses two sockets. In my setup, I had only the "word select" wire connected to GPIO B and the rest to A. 
The SLM only uses only one data line.

## Usage
1. Set up Arduino IDE for the senseBox MCU. (https://github.com/sensebox/senseBox-MCU-S2-ESP32S2)
2. Install the Adafruit GFX and SSD1306 libraries.
3. Connect the hardware as described, and insert the SD card. 
4. Upload the code to the board.