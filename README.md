# SCPI-Spectral-Sensor
A RP2040-based project for a spectral sensor that responds to SCPI commands via a socket.
## Parts
1X Wiznet w5500 pico-evb

1X TCA9548A I2C Multiplexer

3X Adafruit AS7262

4X STEMMA QT / Qwiic cables or JST connectors and wire.

## Dependencies
Mbed RP2040 core 

Adafruit_AS726x
## Commands
*IDN? -identify. Returns the SN, which is also the last byte of the MAC 

Color? X -responds with the color data (X=0,1,2). Each color is delimited by a comma ",". Colors are in this order: Violet (450nm), Blue (500nm), Green (550nm), Yellow (570nm), Orange (600nm),Red (650nm)

TEMP? -Reads the internal temp of the RP2040 returns in C

SYS XX -sets the last byte of the MAC address/SN based on the bit value of the byte(in ASCII) you send it. Don't change this often. Should be a setup-only task.

## Troubleshooting
The serial port via USB will print the IP Address at init. It will also print if it can't find one of the spectral sensors. The serial is running at 9600 Baud. The serial port does not respond to commands. The socket server is on port 5024.

![PCB](IMG20250424062745.jpg)
