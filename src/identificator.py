import serial
import signal

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

ser = serial.Serial("/dev/ttyUSB0", 115200)

data = np.array([[0,0,0,0,0]])
mainData = np.array([[0,0,0,0,0]])
points = 998
poitstoremove = 50
first = True
stop = False

def find_params(xdata, ydata, A):
    def response(x, k, tau):
        return A*k*(1-np.exp(-x/tau))                                       

    popt, pcov = curve_fit(response, xdata, ydata)
    return popt, response(xdata, *popt)

def write():
    global stop
    np.savetxt("output", data, delimiter=',')
    stop = True

while not stop:
    
    line = ser.readline().decode()
    splitted = line.replace('\n','').split(" ")

    try:
        if first:
            print("Recebendo...")
            first = False
        values = [float(x) for x in splitted]
        data = np.concatenate((data,[values]))
    except:
        pass

    if data[:,0].size % points == 0:
        write()
        data = data[1:]
        data[:,0] -= data[:,0][0]
        data[:,0] /= 1000000.0

        # Motor A
        
        # Gráfico inteiro
        plt.subplot(2,2,1)
        plt.plot(data[:,0], data[:,1], color='k')
        plt.plot(data[:,0], data[:,3], color='r')

        # Parte positiva
        final = np.argmin(data[:,1] > 0)
        datap = data[:final]

        plt.subplot(2,2,2)
        plt.plot(datap[:,0], datap[:,1], color='k')
        plt.plot(datap[:,0], datap[:,3], color='r')

        # Identifica
        pA, rA = find_params(datap[:,0], datap[:,3], datap[:,1].max())
        plt.plot(datap[:,0], rA, color='b')
        print(pA)

        # Motor B
        
        # Gráfico inteiro
        plt.subplot(2,2,3)
        plt.plot(data[:,0], data[:,2], color='k')
        plt.plot(data[:,0], data[:,4], color='r')

        # Parte positiva
        final = np.argmin(data[:,1] > 0)
        datap = data[:final]

        plt.subplot(2,2,4)
        plt.plot(datap[:,0], datap[:,2], color='k')
        plt.plot(datap[:,0], datap[:,4], color='r')

        # Identifica
        pB, rB = find_params(datap[:,0], datap[:,4], datap[:,2].max())
        plt.plot(datap[:,0], rB, color='b')
        print(pB)

        plt.show()

        break
    