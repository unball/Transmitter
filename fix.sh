		sudo usermod -a -G dialout $(whoami)
		sudo chmod a+rw /dev/ttyACM0
		sudo chmod a+rw /dev/ttyUSB0

		echo ‘ATTRS{idVendor}=="2a03", ENV{ID_MM_DEVICE_IGNORE}="1"‘ | sudo tee /etc/udev/rules.d/77-arduino.rules
		sudo udevadm trigger
		sudo apt-get remove modemmanager

