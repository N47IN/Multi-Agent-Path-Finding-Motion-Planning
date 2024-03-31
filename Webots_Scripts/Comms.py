import numpy as np
from scipy.optimize import linear_sum_assignment

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
    
class comms():
    def __init__(self,agent1=None,agent2=None,agent3=None,Admin=None):
        self.agent1 = agent1
        self.agent2 = agent2
        self.agent3 = agent3
        self.Admin = Admin
        
    def getAgent2(self):
        if self.agent2.getQueueLength()>0:
            message2 = self.agent2.getBytes()
            coord3 = rnd(np.frombuffer(message2, dtype=np.float64)) 
            print("Agent2 :",coord3)
            return coord3
            
    def getAgent1(self):
        if self.agent1.getQueueLength()>0:
            message1 = self.agent1.getBytes()
            coord1 = rnd(np.frombuffer(message1, dtype=np.float64)) 
            #print("Agent1 :",coord1)
            return coord1
        else:
            pass
            
    def getAgent3(self):
        if self.agent3.getQueueLength()>0:
            message3 = self.agent3.getBytes()
            coord3 = rnd(np.frombuffer(message3, dtype=np.float64)) 
            #print("Agent3 :",coord3)
            return coord3
        else:
            pass
            
    def getAdmin(self):
        if self.Admin.getQueueLength()>0:
            message = self.Admin.getBytes()
            coord = rnd(np.frombuffer(message, dtype=np.float64)) 
            #print("Admin :",coord)
            return coord
            
        else:
            pass
    