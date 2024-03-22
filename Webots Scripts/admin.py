from controller import Robot, GPS, Motor, Keyboard
import numpy as np
from scipy.optimize import linear_sum_assignment

# create the Robot instance.
robot = Robot()
agent1 = robot.getDevice("receiver_agent1")
agent3 = robot.getDevice("receiver_agent3")
agent2 = robot.getDevice("receiver_agent2")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
agent1.enable(10)
agent2.enable(10)
agent3.enable(10)

iteration = 0
goals = np.asarray([[1,2],[5.3,6.7],[8,1.7]])

def getCostMatrix(goals,coords):
    cost = np.zeros((len(goals),len(goals)))
    for i in range(len(goals)):
        for j in range(len(goals)):
            cost[i,j]= np.linalg.norm(goals[j]-coords[i,0:1])
        
    return cost


def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)



while robot.step(timestep) != -1:
  if iteration ==10:
   coords = []
   if agent2.getQueueLength()>0:
    message2 = agent2.getBytes()
    coord3 = rnd(np.frombuffer(message2, dtype=np.float64)) 
    print("Agent2 :",coord3)
    coords.append(coord3)
       
   if agent3.getQueueLength()>0:
        message3 = agent3.getBytes()
        coord2 = rnd(np.frombuffer(message3, dtype=np.float64)) 
        print("Agent2 : ",coord2)
        coords.append(coord2)
        
   if agent3.getQueueLength()>0:
        message1 = agent1.getBytes()
        coord1 = rnd(np.frombuffer(message1, dtype=np.float64)) 
        print("Agent1 : ",coord1)
        coords.append(coord1)
        
   coords = np.asarray(coords)
   cost = getCostMatrix(goals,coords)
   r,c = linear_sum_assignment(cost)
   #print(cost)
   #print(r,c)
   for i, j in zip(r, c):
        print(f"Worker {i+1} assigned to Goal {j+1}")
  iteration+=1
   
        
  
# Enter here exit cleanup code.
