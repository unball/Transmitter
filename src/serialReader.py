import serial

import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial("/dev/ttyUSB0", 115200)

data = np.array([[0,0,0,0,0]])
count = 0
sample = 1
points = 1000
poitstoremove = 15

while True:

    if data.size == points:
        plt.cla()
        plt.plot(data[:,0], data[:,1], color='k')
        plt.plot(data[:,0], data[:,2], color='r')
        plt.plot(data[:,0], data[:,4], color='b')
        plt.pause(0.00001)
        #data = [data[-1]]
    
    line = ser.readline()
    splitted = line.split(",")
    try:
        values = [float(x) for x in splitted]
    except:
        values = [0 for x in splitted]

    if(count % sample == 0):
        if data.size == points:
            data = data[poitstoremove:]
        data = np.concatenate((data,[values]))
    count += 1
    
