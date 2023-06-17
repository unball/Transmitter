# Repository for the radio transmitter of the UnBall team

## Description 

The radio transmitter count with one ESP32 and one radio transmitter nrf24l01. The pinout configuration can be seen on the image below: 

![Pinout configuration Radio Transmitter NRF24L01 plus ESP32](/documentationImages/pinout-config.jpeg)

> Note that:
> - The team has 2 models for the ESP32 and the pinout configuration is different in each case
> - MISO is equal to RX
> - MOSI is equal to TX
> - The wire of MISO and MOSI have to be twisted
## Installation

This firmware is uploaded using the platformIO extention for VScode editor.
To install the VScode editor download the .deb file from [VScode](https://code.visualstudio.com/) and in the terminal use the following command: 
```bash
dpkg -i path/to/debFile/debFileName.deb 
```

Once the VScode editor is installed to install de platformIO extention follow the instrutions at [PlatformIO - installation](https://platformio.org/install/ide?install=vscode).

Use the platformIO extention options to build and upload the code.

You have to be careful with authorization with serial ports in ``Linux OS``. If you need to authorize a serial port, just type in terminal: 

> ```
> sudo chmod a+rw /dev/ttyUSB0
> ```

Be caregul, because if you disconnect and connect the usb again, the port can be ```/dev/ttyUSB1```. And an error could occur when uploading the code. 

Besides that, in your OS the serial port could be ```/deb/ttyACM0``` or ```/deb/ttyACM1``` (if the situation above happens). 



