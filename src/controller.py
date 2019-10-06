import numpy as np 

class projectController():
    def __init__(self,gain = 0.4/0.00811292, tau = 0.00811292):
        self.poles = np.array([-1/tau])
        self.zeros=np.array([])
        self.gain = gain

    def defineDesiredPoles(self, up, tp):
        zeta = -np.log(up)/np.sqrt(np.pi**2 + np.log(up)**2)    
        wn = np.pi/(tp*np.sqrt(1-zeta**2))
        self.so = wn*(-zeta + np.sqrt(1-zeta**2)*1j)

    def phaseCondition(self):
        return (-np.pi + np.sum(np.angle(self.so-self.poles)) - np.sum(np.angle(self.so-self.zeros)))

    def findZero(self, phase):
        self.zeros = np.append(self.zeros, self.so.real - self.so.imag/np.tan(phase))

    def magCondition(self):
        return np.absolute(np.prod(self.so-self.poles)/(self.gain*np.prod(self.so-self.zeros)))
    
    def definePIController(self, up=0.04, tp=0.035):
        self.poles = np.append(self.poles, 0.0)
        self.defineDesiredPoles(up, tp)
        self.findZero(self.phaseCondition())
        self.k = self.magCondition()
        

pi = projectController()
pi.definePIController()
print(pi.zeros)
print(pi.k)
# so = -1+1j

# poles = np.array([-1/tau])
# poles = np.append(poles,0.0)
# zeros = np.array([])




# defineDesiredPoles(0.04, 0.035)
# phase = phaseCondition(so, np.array([]), np.array([0+0j, -123.2590+0.0j]))
# zero = findZero(so,phase)
# np.append(zeros,zero)
# k = magCondition(so, zeros,poles,gain)
# print(k)