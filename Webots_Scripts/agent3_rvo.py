# Talks at channel 2
from turtle import pos
from controller import Robot, GPS, Motor, Keyboard
from controller import InertialUnit
from global_planner_rrtstar import RRT_star_planner
from velocity_obstacle_webots import RVO
import numpy as np
from Comms import comms
from ccma import CCMA
from pure_pursuit import PP
import math

# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

Yaw = robot.getDevice("inertial unit")

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent1 = robot.getDevice("receiver_agent1")
agent2 = robot.getDevice("receiver_agent2")
emitter = robot.getDevice("emitter")
admin = robot.getDevice("receiver_admin")
admin.enable(10)
w_ma = 4
w_cc = 2
gps = robot.getDevice("gps")
gps.enable(10)
Yaw.enable(10)

keyboard.enable(10)
agent1.enable(10)
agent2.enable(10)
#emitter.enable(10)

right_motor.setPosition(float('inf'))
left_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)
timestep = 10

data = [20]
timestep = int(robot.getBasicTimeStep())

g_planner = RRT_star_planner("Binary_Mask.png",0.5,"/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/image1.png")
comms = comms(agent1= agent1,agent2=agent2, Admin=admin)
ccma = CCMA(w_ma, w_cc, distrib="hanning")

def broadcast(data):
    message = rnd(np.array(data))
    message = message.tobytes()
    emitter.send(message)
    
def getDist(pos,obs):
    return np.linalg.norm(pos[0:2]-obs[0:2])

def steer(data,v):
    speed = data*0.10/(0.5*0.8)
    right_motor.setVelocity(-8*v -3*speed)
    left_motor.setVelocity(-8*v +3*speed)

def plan(position,goal):
    g_planner.setStart(position[0:2])
    g_planner.setGoal([ goal[0], goal[1]])
    print(position)
    global_path, nodes = g_planner.RRT()
    g_plan_smoothed = ccma.filter(np.asarray(global_path), cc_mode=False)
    return g_plan_smoothed

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
path = False

start_time = robot.getTime()
v = 0
time = 0
velocity = 1

while robot.step(timestep) != -1:
    position = gps.getValues()
    yaw = Yaw.getRollPitchYaw()
    yaw = yaw[2]
    agent1 = comms.getAgent1()
    agent2 = comms.getAgent2()
    #print(agent2,agent1)

    agent3 = [position[0],position[1],velocity, yaw]
    broadcast(agent3)
    
    if admin.getQueueLength()>0 and path==False:
        goal = comms.getAdmin()[4:6]
        # print("current position Agent3 :",position[0:2])
        # print("goal Agent3 :",goal)
        g_plan_smoothed = plan(position,goal)
        path = True
        Local = False
        tracker = PP(g_plan_smoothed,yaw)
        rvo = RVO(goal,agent3,agent1,agent2)
        #print(g_plan)
        
    if path == True :
        
        if np.linalg.norm(position[0:2] - goal) > 0.05 :
            time2 = robot.getTime() - start_time
            print("dist is", getDist(agent2,position),getDist(agent1,position))
            if getDist(agent2,position) < 0.55 or getDist(agent1,position) < 0.55 :
                Local = True
                rvo.update(agent3,agent1,agent2)
                try:
                    velocity, steering_angle = rvo.simulate(agent3,agent1,agent2)
                except: 
                    steer(velocity,steering_angle)
                steer(steering_angle,velocity)
                print(steering_angle,velocity)
                print("Agent3 : RVO")
                
            else:
                try :
                    Local = False
                    velocity, steering_angle, time = tracker.execute(xpos = position[0], ypos = position[1], yaw = yaw, v = velocity ,time = time)
                    steer(steering_angle,velocity)
                    print("Agent3 : PP")
                except:
                    velocity = 0
                    steer(0,0)
                
            if getDist(agent2,position) > 0.6 and getDist(agent1,position) > 0.6 :
                if Local == True :
                    Local = False
                    g_plan_smoothed = plan(position,goal)
                    tracker = PP(g_plan_smoothed,yaw)
                
                
