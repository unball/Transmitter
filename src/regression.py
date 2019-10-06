import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import csv

AMPLITUDE = 64

class Idendify():
    def __init__(self):
        self.xdata = None
        self.pA = None 
        self.pB = None

    def readFile(self, filename='output'):
        if self.xdata == None:
            f = open(filename, "r")
            ini = False
            lines = f.readlines()
            data = np.array([[0,0,0,0,0]])
            for line in lines:
                newdata = list(map(lambda x: float(x), line.split(",")))
                if len(newdata) == 5 and (newdata[1]!=0 or ini):
                    data = np.concatenate((data, [newdata]))
                    ini = True

            data = data[1:]
            self.xdata = data[:,0]
            self.xdata /= 1000000.0
            self.xdata -= self.xdata[0]

            self.refA = data[:,1]
            self.motorA = data[:,2]
            self.refB = data[:,3]
            self.motorB = data[:,4]

    def saveParam(self, pA, pB):
        with open('motors.csv', 'w', newline='') as csvfile:
            fieldnames = ['k', 'tau']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow({'k': pA[0], 'tau': pA[1]})
            writer.writerow({'k': pB[0], 'tau': pB[1]})

    def readParm(self):
        #csv.register_dialect('tau', delimiter=':', quoting=csv.QUOTE_NONE)
        with open('motors.csv', newline='') as f:
            reader = csv.reader(f)
            parm = []
            for (indx, row) in enumerate(reader):
                if indx==0:
                    continue
                parm.append([float(x) for x in row])
            self.pA = parm[0]
            self.pB = parm[1]

    def find_params(self, xdata, ydata):
        def response(x, k, tau):
            return AMPLITUDE*k*(1-np.exp(-x/tau))                                       
        popt, pcov = curve_fit(response, xdata, ydata)
        return popt, response(xdata, *popt)

    def regression(self):
        self.readFile()
        self.pA, rA = self.find_params(xdata, motorA)
        self.pB, rB = self.find_params(xdata, motorB)
        self.plotData()
        self.saveParam(self.pA, self.pB)
        # print(pA)

    def plotData(self):
        self.readFile()
        plt.subplot(2,1,1)
        #plt.axis([8.4,11.5, -25,25])
        plt.plot(xdata, data[:,1], 'k-')
        #plt.plot(xdata, rA, 'r-')
        plt.plot(xdata, motorA, 'b-')

        plt.subplot(2,1,2)
        plt.plot(xdata, data[:,3], 'k-')
        #plt.axis([8.4,11.5, -25,25])
        #plt.plot(xdata, rB, 'r-')
        plt.plot(xdata, motorB, 'b-')
        plt.show()


if __name__ == "__main__":
    ident =  Idendify()
    #ident.saveParam([1.12,2.32], [3.12,2.2])
    ident.readParm()
    print(ident.pA)
    print(ident.pB)