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
    
    def definePIController(self, up=0.03, tp=0.035):
        self.poles = np.append(self.poles, 0.0)
        self.defineDesiredPoles(up, tp)
        self.findZero(self.phaseCondition())
        self.k = self.magCondition()
        print("Continuos Controller:")
        print("u'(t) = " +  str(self.k) + " * (e'(t) + " + str(-self.zeros[0]) + "e(t))")
        
    def discretize(self, T):
        zero = np.exp(T*self.zeros[0])
        pole = 1
        s = np.pi*1/T*1j
        z = -1 #exp(-s*T)
        continuosGain = np.abs(self.k*(s-self.zeros[0])/s)
        InvdiscretGain = (z-pole)/(z-zero)
        discreteK = continuosGain*InvdiscretGain
        print("\nDiscrete Controller(mached):")
        print("u[k] = " +  str(discreteK) + " * (e[k] + " + str(-zero) + "e[k-1]) + u[k-1]")
        

if __name__ == "__main__":
    pi = projectController()
    pi.definePIController()
    pi.discretize(0.002)