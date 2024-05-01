# Talks at channel 1
from turtle import pos
from controller import Robot, GPS, Motor, Keyboard
from controller import InertialUnit
from global_planner import RRT_star_planner
import numpy as np
from Comms import comms
from ccma import CCMA
from pure_pursuit import PP
# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

w_ma = 4
w_cc = 2

Yaw = robot.getDevice("inertial unit")
right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent2 = robot.getDevice("receiver_agent2")
agent3 = robot.getDevice("receiver_agent3")
admin = robot.getDevice("receiver_admin")
emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps")
gps.enable(10)
keyboard.enable(10)
Yaw.enable(10)
admin.enable(10)
agent2.enable(10)
agent3.enable(10)

comms = comms(agent2= agent2,agent3=agent3, Admin=admin)
g_planner = RRT_star_planner("Binary_Mask.png",0.5)
ccma = CCMA(w_ma, w_cc, distrib="hanning")

right_motor.setPosition(float('inf'))
left_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)
timestep = 10


path = False

def broadcast(data):
    message = rnd(np.array(data))
    message = message.tobytes()
    emitter.send(message)
    
def steer(data,v):
    speed = data*0.10/(0.5*0.8)
    right_motor.setVelocity(-8*v -3*speed)
    left_motor.setVelocity(-8*v +3*speed)

    
def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
    
start_time = robot.getTime()
v = 0
time = 0
velocity = 0

while robot.step(timestep) != -1:
    position = gps.getValues()
    

    #print(position)
    broadcast(position)
    yaw = Yaw.getRollPitchYaw()
    yaw = yaw[2] 

    
    if admin.getQueueLength()>0 and path==False:
        goal = comms.getAdmin()[0:2]
        print("current position Agent1 :",position[0:2])
        print("goal Agent1 :",goal)
        g_planner.setStart(position[0:2])
        g_planner.setGoal([ goal[0], goal[1]])
        global_path = np.asarray(g_planner.RRT())
        g_plan_smoothed = ccma.filter(global_path, cc_mode=False)
        path = True
        tracker = PP(g_plan_smoothed,yaw)
        #print(g_plan)
        
    if path == True:
        if np.linalg.norm(position[0:2] - goal) > 0.05 :
            time2 = robot.getTime() - start_time

            try :
                velocity, steering_angle, time = tracker.execute(xpos = position[0], ypos = position[1], yaw = yaw, v = velocity ,time = time)
                steer(steering_angle,velocity)
            except:
                steer(0,0)

        