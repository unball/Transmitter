import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

AMPLITUDE = 64

f = open("output", "r")
lines = f.readlines()

data = np.array([[0,0,0,0,0]])

#print(lines)
for line in lines:
    newdata = list(map(lambda x: float(x), line.split(",")))
    if len(newdata) == 5 : data = np.concatenate((data, [newdata]))

data = data[1:]
xdata = data[:,0]
xdata /= 1000000.0
xdata -= xdata[0]

motorA = data[:,2]
motorB = data[:,4]

def find_params(xdata, ydata):
    def response(x, k, tau):
        return AMPLITUDE*k*(1-np.exp(-x/tau))                                       

    popt, pcov = curve_fit(response, xdata, ydata)
    return popt, response(xdata, *popt)

# pA, rA = find_params(xdata, motorA)
# pB, rB = find_params(xdata, motorB)

# print(pA)

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


