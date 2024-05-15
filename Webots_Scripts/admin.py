from controller import Robot, GPS, Motor, Keyboard
import numpy as np
from scipy.optimize import linear_sum_assignment
from Comms import comms

# create the Robot instance.
robot = Robot()


agent1 = robot.getDevice("receiver_agent1")
emitter = robot.getDevice("emitter")
agent3 = robot.getDevice("receiver_agent3")
agent2 = robot.getDevice("receiver_agent2")
comms = comms(agent1,agent2,agent3)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
agent1.enable(10)
agent2.enable(10)
agent3.enable(10)

iteration = 0
goals = np.asarray([[4,6],[5,7],[9,9]])

def getCostMatrix(goals,coords):
    cost = np.zeros((len(goals),len(goals)))
    for i in range(len(goals)):
        for j in range(len(goals)):
            cost[i,j]= np.linalg.norm(goals[j]-coords[i,0:1])
    return cost

def broadcast(data):
    message = rnd(np.array(data))
    message = message.tobytes()
    emitter.send(message)
    
def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

while robot.step(timestep) != -1:
  if iteration ==10:
    coords = []
    coord1 = comms.getAgent1()
    #print(coord1)
    coords.append(coord1[0:2])
    coord2 = comms.getAgent2()
    coords.append(coord2[0:2])
    #print(coord2)
    coord3 = comms.getAgent3()
    coords.append(coord3[0:2])
    #print(coord3)
    coords = np.asarray(coords)
    cost = getCostMatrix(goals,coords)
    r,c = linear_sum_assignment(cost)
    #print(cost)
    #print(r,c)
    ''' for i, j in zip(r, c):
        print(f"Worker {i+1} assigned to Goal {j+1}") '''
    goals = [goals[i] for i in c]
    ''' for i in range(len(goals)):
        list(goals[i]).append(float(i)) '''
    print(goals)
    broadcast(goals)
  iteration+=1

        

# Enter here exit cleanup code.