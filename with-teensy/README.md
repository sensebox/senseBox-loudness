# Read dBA values measured by the Teensy DNMS board onto the senseBox MCU  

Upload the DNMS program onto the Teensy controller to continuosly calculate the current decibel values.
Use the senseBoc MCU to request these values at desired intervals. 

## Flashing the Teensy

Install the Teensy Loader from: https://www.pjrc.com/teensy/loader.html

Download the HEX file from: https://github.com/hbitter/DNMS/tree/master/Firmware/Teensy/Teensy4.0

Add a jumper to connect pins on J1 on the Teensy to allow power through USB connection. 

With the Teensy connected via USB and the Teensy Loader open:
- click on "Choose HEX file" and choose the downloaded file
- then double click button on Teensy to enter Boot Mode.

The program should automatically uploaded to the Teensy Board
