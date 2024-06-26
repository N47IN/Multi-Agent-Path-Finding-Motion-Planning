# Talks at channel 2
from controller import Robot, GPS, Motor, Keyboard
import numpy as np
#from Comms import comms
#from global_planner import RRT_planner

# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent1 = robot.getDevice("receiver_agent1")
agent3 = robot.getDevice("receiver_agent3")
emitter = robot.getDevice("emitter")
admin = robot.getDevice("receiver_admin")
admin.enable(10)

gps = robot.getDevice("gps")
gps.enable(10)
keyboard.enable(10)
agent1.enable(10)
agent3.enable(10)

right_motor.setPosition(float('inf'))
left_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)
timestep = 10

data = [20]
timestep = int(robot.getBasicTimeStep())

#comms = comms(agent1= agent1,agent3=agent3, Admin=admin)

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
    position = gps.getValues()
    broadcast(position)
    #right_motor.setVelocity(1)
    #left_motor.setVelocity(1)
    
    
# Enter here exit cleanup code.