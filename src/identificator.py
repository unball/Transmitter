from serialReader import SerialReader
from scipy.optimize import curve_fit

import matplotlib.pyplot as plt
import numpy as np

BUFFER_SIZE = 1000

# Função que otimiza os parâmetros para uma resposta de primeira ordem
def find_params(xdata, ydata, A):
    def response(x, k, tau):
        return A*k*(1-np.exp(-x/tau))                                       

    popt, pcov = curve_fit(response, xdata, ydata)
    return popt, response(xdata, *popt)

# Estima os parâmetros da planta com base nos dados em data, onde data[:,0] é tempo, data[:,1] é velocidade e data[:,2] é encoder
def parameterEstimator(data):
        
    # Gráfico inteiro
    plt.subplot(2,2,1)
    plt.plot(data[:,0], data[:,1], color='k')
    plt.plot(data[:,0], data[:,2], color='r')

    # Parte positiva
    final = np.argmin(data[:,1] > 0)
    datap = data[:final]

    plt.subplot(2,2,2)
    plt.plot(datap[:,0], datap[:,1], color='k')
    plt.plot(datap[:,0], datap[:,2], color='r')

    # Identifica
    p, r = find_params(datap[:,0], datap[:,2], datap[:,1].max())
    plt.plot(datap[:,0], r, color='b')
    
    return p

def main():
    # Recebe do rádio
    serial = SerialReader("/dev/ttyUSB0", 115200)

    # Recebe do direto do teensy via USB
    #serial = SerialReader("/dev/ttyACM0", 9600)

    # Dados a serem recebidos
    data = []

    # Recebe um buffer de dados
    while data.size < BUFFER_SIZE-2:
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

    # Estima a planta do motor A
    p = parameterEstimator(np.array([data[:,0], data[:,1], data[:,3]]).T)
    print("Motor A:")
    print(p)

    # Estima a planta do motor B
    p = parameterEstimator(np.array([data[:,0], data[:,2], data[:,4]]).T)
    print("Motor B:")
    print(p)

    plt.show()

if __name__ == "__main__":
    main()
