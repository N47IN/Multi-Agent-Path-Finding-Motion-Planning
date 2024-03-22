# Talks at channel 2
from controller import Robot, GPS, Motor, Keyboard
import numpy as np
# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent1 = robot.getDevice("receiver_agent1")
agent3 = robot.getDevice("receiver_agent3")
emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps")
gps.enable(10)
keyboard.enable(10)
agent1.enable(10)
agent3.enable(10)

right_motor.setPosition(100)
left_motor.setPosition(100)
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)
timestep = 10

data = [20]
timestep = int(robot.getBasicTimeStep())

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

while robot.step(timestep) != -1:
    data = rnd(np.array(gps.getValues()))
    message = data.tobytes()
    emitter.send(message)

    pass

# Enter here exit cleanup code.
