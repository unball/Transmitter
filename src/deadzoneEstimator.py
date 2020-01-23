from serialReader import SerialReader

import matplotlib.pyplot as plt
import numpy as np

BUFFER_SIZE = 3500

# TODO: Usar scipy.optimize.curve_fit para estimar a deadzone

# Encontra o índice do primeiro máximo de um conjunto de amostras
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

# Encontra os índices que englobam o primeiro subconjunto de dados positivos de uma amostra
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

# Encontra os índices que englobam o primeiro subconjunto de dados negativos de uma amostra
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

# Estima a deadzone com base nos dados em data, onde data[:,0] é tempo, data[:,1] é velocidade e data[:,2] é encoder
def deadzoneEstimator(data):
    # Gráfico inteiro
    plt.subplot(2,5,1)
    plt.plot(data[:,0], data[:,1], color='k')
    plt.plot(data[:,0], data[:,2], color='r')

    # Parte positiva no tempo
    ia, ib = getPositiveSamples(data[:,1])
    datap = data[ia:ib]

    plt.subplot(2,5,2)
    plt.plot(datap[:,0], datap[:,1], color='k')
    plt.plot(datap[:,0], datap[:,2], color='r')

    # Parte positiva vout x vin
    plt.subplot(2,5,3)
    plt.scatter(datap[:,1], datap[:,2], color='r', s=1)
    plt.plot(datap[:,1], datap[:,2], color='k')

    # Estima deadzone positiva
    deadp = datap[:,1][firstMax((datap[:,2][-1] / datap[:,1][-1]) * datap[:,1] - datap[:,2], 1)]

    # Parte negativa no tempo
    ia, ib = getNegativeSamples(data[:,1])
    datan = data[ia:ib]

    plt.subplot(2,5,4)
    plt.plot(datan[:,0], datan[:,1], color='k')
    plt.plot(datan[:,0], datan[:,2], color='r')

    # Parte negativa vout x vin
    plt.subplot(2,5,5)
    plt.scatter(datan[:,1], datan[:,2], color='r', s=1)
    plt.plot(datan[:,1], datan[:,2], color='k')

    # Estima deadzone negativa
    deadn = datan[:,1][firstMax(-(datan[:,2][-1] / datan[:,1][-1]) * datan[:,1] + datan[:,2], 1)]

    return deadp, deadn

def main():
    # Recebe do rádio
    serial = SerialReader("/dev/ttyUSB0", 115200)

    # Recebe do direto do teensy via USB
    #serial = SerialReader("/dev/ttyACM0", 9600)

    # Dados a serem recebidos
    data = []

    # Recebe um buffer de dados
    while len(data) < BUFFER_SIZE-2:
        values = serial.read()

        # Concatena ao conjunto de dados lidos
        if values is not None:
            data.append(values)

    # Converte para numpy
    data = np.array(data)

    # Salva os dados em um arquivo
    np.savetxt("deadzonedata.dat", data, delimiter=',')

    # Corrige os dados para segundos
    data[:,0] -= data[:,0][0]
    data[:,0] /= 1000000.0

    # Estima a deadzone e plota os gráficos para análise do motor A
    deadp, deadn = deadzoneEstimator(np.array([data[:,0], data[:,1], data[:,3]]).T)
    print("Motor A: deadzone positiva: {0}, deadzone negativa: {1}".format(deadp, deadn))

    # Estima a deadzone e plota os gráficos para análise do motor B
    deadp, deadn = deadzoneEstimator(np.array([data[:,0], data[:,2], data[:,4]]).T)
    print("Motor B: deadzone positiva: {0}, deadzone negativa: {1}".format(deadp, deadn))

    plt.show()

if __name__ == "__main__":
    main()
