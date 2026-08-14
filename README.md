# Repository for the radio transmitter of the UnBall team

## Description 

The radio transmitter uses ESP8266. 
The wifi transmitter uses ESP32.

## Installation

This firmware is uploaded using the platformIO extention for VScode editor.
To install the VScode editor download the .deb file from [VScode](https://code.visualstudio.com/) and in the terminal use the following command: 
```bash
dpkg -i path/to/debFile/debFileName.deb 
```

Once the VScode editor is installed to install de platformIO extention follow the instrutions at [PlatformIO - installation](https://platformio.org/install/ide?install=vscode).

Use the platformIO extention options to build and upload the code.

You have to be careful with authorization with serial ports in ``Linux OS``. If you need to authorize a serial port, just type in terminal: 

> ```bash
> sudo chmod a+rw /dev/ttyUSB0
> ```

Be caregul, because if you disconnect and connect the usb again, the port can be ```/dev/ttyUSB1```. And an error could occur when uploading the code. 

Besides that, in your OS the serial port could be ```/deb/ttyACM0``` or ```/deb/ttyACM1``` (if the situation above happens). 

### Caution 

If the plataform.io isn't showing any ttyUSB or ttyACM ports on ubuntu it probably is related to BRLTTY modules, then we gonna have to remove it. Don't worry this module is related this module is for acessibility related to braile display onto ubuntu.

To start it you gotta list all the devices, just to check if it's working

> ```bash
> ls /dev/ttyUSB* 
> #or 
> ls /dev/ttyACM*
> ```

if the port isn't showing there, the system hasn't loaded the board, then it probably is the module, to remove it:

> ```bash
> sudo apt-get remove --auto-remove brltty
> ```

Unplug and replug your board, type again in Terminal
> ```bash
> ls /dev/ttyUSB* 
> #or 
> ls /dev/ttyACM*
> ```

it probably fixed it for you, but if it hasn't fixed try the link below.

[Link to debug it or if you wanna see the original solution](https://forum.arduino.cc/t/ubuntu-arduino-ide-not-showing-any-ports/1043925/19)
