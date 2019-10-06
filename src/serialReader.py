import serial
import signal

#import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial("/dev/ttyUSB0", 115200)

data = np.array([[0,0,0,0,0]])
mainData = np.array([[0,0,0,0,0]])
points = 10000
poitstoremove = 50
stop = False

def write(signum, frame):
    global stop
    np.savetxt("output", mainData, delimiter=',')
    stop = True

signal.signal(signal.SIGINT, write)

while not stop:

    #if data.size == points:
    #    plt.cla()
    #    plt.plot(data[:,0], data[:,1], color='k')
    #    plt.plot(data[:,0], data[:,2], color='r')
    #    plt.plot(data[:,0], data[:,4], color='b')
    #    plt.pause(0.03)
    #    #data = [data[-1]]
    
    line = ser.readline()
    splitted = line.split(",")

    try:
        values = [float(x) for x in splitted]
        mainData = np.concatenate((mainData,[values]))
        if mainData.size > 700:
            np.savetxt("output", mainData, delimiter=',')
            stop = True
        
    except:
        exit()
        pass

    #if data.size == points:
    #    data = data[poitstoremove:]

    #data = np.concatenate((data,[values]))
    