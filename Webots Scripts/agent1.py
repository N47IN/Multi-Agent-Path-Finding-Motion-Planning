# Talks at channel 1
from controller import Robot, GPS, Motor, Keyboard
import numpy as np
# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent2 = robot.getDevice("receiver_agent2")
agent3 = robot.getDevice("receiver_agent3")
emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps")
gps.enable(10)
keyboard.enable(10)
agent2.enable(10)
agent3.enable(10)

right_motor.setPosition(100)
left_motor.setPosition(100)
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)
timestep = 10

command ={                 
   ord("W"): [3,3],
   ord("A"):[3,0],
   ord("D"):[0,3]
   }
   
def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

   message = rnd(np.array(gps.getValues()))
   message = message.tobytes()
   emitter.send(message)
# Enter here exit cleanup code.
