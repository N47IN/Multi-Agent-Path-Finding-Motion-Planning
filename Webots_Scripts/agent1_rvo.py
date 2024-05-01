# Talks at channel 1
from turtle import pos
from controller import Robot, GPS, Motor, Keyboard
from controller import InertialUnit
from global_planner_rrtstar import RRT_star_planner
import numpy as np
from velocity_obstacle_webots import RVO
from Comms import comms
from ccma import CCMA
from pure_pursuit import PP
import math
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
    
def getDist(pos,obs):
    return np.linalg.norm(pos[0:2]-obs[0:2])
    
def steer(data,v):
    speed = data*0.10/(0.5*0.8)
    right_motor.setVelocity(-8*v -6*speed)
    left_motor.setVelocity(-8*v +6*speed)

    
def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
    
start_time = robot.getTime()
v = 0
time = 0
velocity = 1

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
        agent2 = comms.getAgent2()
        agent3 = comms.getAgent3()
        agent1 = [position[0],position[1],velocity,yaw]
        rvo = RVO(goal,agent1,agent2,agent3)
        if np.linalg.norm(position[0:2] - goal) > 0.05 :
            time2 = robot.getTime() - start_time
            broadcast(agent1)
            
            if getDist(agent2,position) < 0.5 or getDist(agent3,position)< 0.5:
                velocity, steering_angle = rvo.simulate(agent1)
                steer(5*steering_angle,2*velocity)
                print(steering_angle,velocity)
                print("obsooo")
            else:
                steer(0,0.2/8)
                

        