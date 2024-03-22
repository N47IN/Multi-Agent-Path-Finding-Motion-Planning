
from controller import Robot, GPS, Motor, Keyboard
import numpy as np
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

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
        
while robot.step(timestep) != -1:
    # Read the sensors:
   if agent2.getQueueLength()>0:
    message2 = agent2.getBytes()
    coord = rnd(np.frombuffer(message2, dtype=np.float64)) 
    print("Agent3 :",coord)
   
       
   if agent3.getQueueLength()>0:
        message3 = agent3.getBytes()
        coord = rnd(np.frombuffer(message3, dtype=np.float64)) 
        print("Agent2 : ",coord)
        
   if agent3.getQueueLength()>0:
        message1 = agent1.getBytes()
        coord = rnd(np.frombuffer(message1, dtype=np.float64)) 
        print("Agent1 : ",coord)

# Enter here exit cleanup code.
