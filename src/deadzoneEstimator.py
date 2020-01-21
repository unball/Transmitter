import serial
import signal

import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial("/dev/ttyUSB0", 115200)

data = np.array([[0,0,0,0,0]])
mainData = np.array([[0,0,0,0,0]])
points = 3450
poitstoremove = 50
stop = False

def write():
    global stop
    np.savetxt("output", data, delimiter=',')
    stop = True

def firstMax(samples,margin):
    currentmax = 0
    maxindex = 0
    for i,s in enumerate(samples):
        if s > currentmax:
            currentmax = s
            maxindex = i
        elif s < currentmax-margin:
            break
    return maxindex

def getPositiveSamples(samples):
    minindex = np.argmin(samples <= 0)
    maxindex = 0
    currentmax = 0
    for i,s in enumerate(samples[minindex:]):
        if s > currentmax:
            currentmax = s
            maxindex = i+minindex
        elif s < currentmax: break
    return minindex,maxindex

def getNegativeSamples(samples):
    maxindex = np.argmax(samples < 0)
    minindex = 0
    currentmin = 0
    for i,s in enumerate(samples[maxindex:]):
        if s < currentmin:
            currentmin = s
            minindex = i+maxindex
        elif s > currentmin: break
    return maxindex,minindex

def getBestSamples(samples, margin):
    currentmax = 0
    maxindex = 0
    # Encontra o primeiro máximo e para quando decrescer
    for i,s in enumerate(samples):
        if s > currentmax: 
            currentmax = s
            maxindex = i
        elif s < currentmax-margin: break

    currentmin = currentmax
    minindex = 0
    # Encontra o mínimo a partir do máximo e para quando crescer
    for i,s in enumerate(samples[maxindex:]):
        if s < currentmin: 
            currentmin = s
            minindex = i
        elif s > currentmin+margin: break

    return maxindex, minindex+maxindex


while not stop:

    #if data.size % points == 0:
    #    plt.cla()
    #    plt.subplot(2,1,1)
    #    plt.plot(data[:,0], data[:,1], color='k')
    #    plt.plot(data[:,0], data[:,3], color='r')
    #    plt.subplot(2,1,2)
    #    plt.plot(data[:,0], data[:,2], color='k')
    #    plt.plot(data[:,0], data[:,4], color='r')
    #    plt.pause(1)
    #    #data = [data[-1]]
    
    line = ser.readline().decode()
    splitted = line.replace('\n','').split(" ")

    try:
        values = [float(x) for x in splitted]
        data = np.concatenate((data,[values]))
    except:
        pass

    if data[:,0].size % points == 0:
        write()
        data[:,0] -= data[0,0]

        # Motor A
        
        # Gráfico inteiro
        plt.subplot(2,5,1)
        plt.plot(data[:,0], data[:,1], color='k')
        plt.plot(data[:,0], data[:,3], color='r')

        # Parte positiva no tempo
        ia, ib = getPositiveSamples(data[:,1])
        datap = data[ia:ib]

        plt.subplot(2,5,2)
        plt.plot(datap[:,0], datap[:,1], color='k')
        plt.plot(datap[:,0], datap[:,3], color='r')

        # Parte positiva vout x vin
        plt.subplot(2,5,3)
        plt.scatter(datap[:,1], datap[:,3], color='r', s=1)
        plt.plot(datap[:,1], datap[:,3], color='k')

        # Estima deadzone positiva
        deadp = datap[:,1][firstMax((datap[:,3][-1] / datap[:,1][-1]) * datap[:,1] - datap[:,3], 1)]
        print("Motor A, deadzone positiva: " + str(deadp))

        # Parte negativa no tempo
        ia, ib = getNegativeSamples(data[:,1])
        datan = data[ia:ib]

        plt.subplot(2,5,4)
        plt.plot(datan[:,0], datan[:,1], color='k')
        plt.plot(datan[:,0], datan[:,3], color='r')

        # Parte negativa vout x vin
        plt.subplot(2,5,5)
        plt.scatter(datan[:,1], datan[:,3], color='r', s=1)
        plt.plot(datan[:,1], datan[:,3], color='k')

        # Estima deadzone negativa
        deadn = datan[:,1][firstMax(-(datan[:,3][-1] / datan[:,1][-1]) * datan[:,1] + datan[:,3], 1)]
        print("Motor A, deadzone negativa: " + str(deadn))



        # Motor B
        
        # Gráfico inteiro
        plt.subplot(2,5,6)
        plt.plot(data[:,0], data[:,2], color='k')
        plt.plot(data[:,0], data[:,4], color='r')

        # Parte positiva no tempo
        ia, ib = getPositiveSamples(data[:,2])
        datap = data[ia:ib]

        plt.subplot(2,5,7)
        plt.plot(datap[:,0], datap[:,2], color='k')
        plt.plot(datap[:,0], datap[:,4], color='r')

        # Parte positiva vout x vin
        plt.subplot(2,5,8)
        plt.scatter(datap[:,2], datap[:,4], color='r', s=1)
        plt.plot(datap[:,2], datap[:,4], color='k')

        # Estima deadzone positiva
        deadp = datap[:,2][firstMax((datap[:,4][-1] / datap[:,2][-1]) * datap[:,2] - datap[:,4], 1)]
        print("Motor B, deadzone positiva: " + str(deadp))

        # Parte negativa no tempo
        ia, ib = getNegativeSamples(data[:,2])
        datan = data[ia:ib]

        plt.subplot(2,5,9)
        plt.plot(datan[:,0], datan[:,2], color='k')
        plt.plot(datan[:,0], datan[:,4], color='r')

        # Parte negativa vout x vin
        plt.subplot(2,5,10)
        plt.scatter(datan[:,2], datan[:,4], color='r', s=1)
        plt.plot(datan[:,2], datan[:,4], color='k')

        # Estima deadzone negativa
        deadn = datan[:,1][firstMax(-(datan[:,4][-1] / datan[:,2][-1]) * datan[:,2] + datan[:,4], 1)]
        print("Motor B, deadzone negativa: " + str(deadn))
        

        plt.show()
        stop = True
        break

    #    data = data[poitstoremove:]

    #data = np.concatenate((data,[values]))
    