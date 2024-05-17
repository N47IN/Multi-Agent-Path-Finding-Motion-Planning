# Talks at channel 1
from turtle import pos
from controller import Robot, GPS, Motor, Keyboard
from controller import InertialUnit
from velocity_obstacle_webots import RVO
from global_planner_rrtstar import RRT_star_planner
import numpy as np
from Comms import comms
from ccma import CCMA
from pure_pursuit import PP
from scipy.spatial import KDTree
# create the Robot instance.
robot = Robot()
keyboard= Keyboard()

w_ma = 4
w_cc = 2

dyna = robot.getDevice("receiver")
dyna.enable(10)
Yaw = robot.getDevice("inertial unit")
right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
agent1 = robot.getDevice("receiver_agent1")
agent2 = robot.getDevice("receiver_agent2")
admin = robot.getDevice("receiver_admin")
emitter = robot.getDevice("emitter")
gps = robot.getDevice("gps")
gps.enable(10)
keyboard.enable(10)
Yaw.enable(10)
admin.enable(10)
#emitter.enable(10)
agent1.enable(10)
agent2.enable(10)

comms = comms(agent1= agent1,agent2=agent2, Admin=admin, dyna=dyna)
g_planner = RRT_star_planner("Binary_Mask.png",0.5,"/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/image1.png")
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

def plan(position,goal):
    g_planner.setStart(position[0:2])
    g_planner.setGoal([ goal[0], goal[1]])
    global_path = np.asarray(g_planner.RRT())
    g_plan_smoothed = ccma.filter(global_path, cc_mode=False)
    return g_plan_smoothed

def rnd(number, precision=4):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)
    
    
def getLocalGoal(tree,position,window_size,nodes):
    window_boundary_x = np.linspace(position[0] - window_size, position[0] + window_size, 20)
    window_boundary_y = np.linspace(position[1] - window_size, position[1] + window_size, 20)
    boundary_points = [[position[0] - window_size, y] for y in window_boundary_y]
    boundary_points += [[position[0] + window_size, y] for y in window_boundary_y]
    boundary_points += [[x, position[1] - window_size] for x in window_boundary_x]
    boundary_points += [[x, position[1] + window_size] for x in window_boundary_x]
    nearest_nodes_indices = tree.query_ball_point(boundary_points, 10)
    nearest_node = nodes[np.argmin([nodes[i].cost for i in range(len(nearest_nodes_indices))])]
    print("COSTTTTTTT",nearest_node.cost)
    goal = nearest_node #pass this goal to local planners
    return goal
    
def check_window(position, bot_pos, obs_pos, window_size = 50): #window size in pixels
    window_x = [position[0] - window_size/2, position[0] + window_size/2]
    window_y = [position[1] - window_size/2, position[1] + window_size/2]
    print("win points",window_x,window_y)
    rvo_bots = []
    obstacles = []
    for i in range(len(bot_pos)):
        print("bot points",bot_pos[i][0],bot_pos[i][1])

        if window_x[0] < bot_pos[i][0] < window_x[1] and window_y[0] < bot_pos[i][1] < window_y[1]:
            rvo_bots.append(i)
            print("bot ", i)
    for i in range(len(obs_pos)):
        if window_x[0] < obs_pos[i][0] < window_x[1] and window_y[0] < obs_pos[i][1] < window_y[1]:
            obstacles.append(i)

    return rvo_bots, obstacles

start_time = robot.getTime()
v = 0
time = 0
velocity = 0

while robot.step(timestep) != -1:
    position = gps.getValues()
    yaw = Yaw.getRollPitchYaw()
    yaw = yaw[2] 
    agent3 = [position[0],position[1],velocity,yaw]
    broadcast(agent3)
    agent1 = comms.getAgent1()
    agent2 = comms.getAgent2()
    dynamic = comms.getDyna()
    print("dynaaaa", dynamic)


    #print(position)

    if admin.getQueueLength()>0 and path==False:
        goal = comms.getAdmin()[0:2]
        print("current position Agent3 :",position[0:2])
        print("goal Agent3 :",goal)
        g_planner.setStart(position[0:2])
        g_planner.setGoal([ goal[0], goal[1]])
        print("planning global")
        global_path, nodes = g_planner.RRT()
        global_path = np.asarray(global_path)
        print(global_path)
        g_plan_smoothed = ccma.filter(np.asarray(global_path), cc_mode=False)
        path = True
        rvo = RVO(goal,agent3,agent1,agent2)

        #print(g_plan)
        
    window_size = 1.2
    local_done = True
    local_active = False
    if path == True:
        if np.linalg.norm(position[0:2] - goal) > 0.05 :
            time2 = robot.getTime() - start_time
            
            #in every time step calc the distance between bots and obstacles and the robot itself
            if not(local_active):
                bot_pos = [agent1[0:2], agent2[0:2]]
                print("botssss",bot_pos)
                obstacle_pos = []
                rvo_bots, obstacles = check_window([position[0],position[1]], bot_pos, obstacle_pos, window_size)
                #if any of the local planners need to be activated, then activate them and re-define tracker
                print("Bot info",rvo_bots, obstacles)
                if rvo_bots or obstacles:
                    local_active = True
                    local_done = False
                    tree = KDTree([(node.x,node.y) for node in nodes])
                    local_goal = getLocalGoal(tree,position,window_size,nodes)
                    print("local goal is", local_goal)
                    new_global_path = g_planner.generate_path(local_goal)
                    #g_plan_smoothed = ccma.filter(np.asarray(new_global_path), cc_mode=False)

                if rvo_bots:
                    try:
                     velocity, steering_angle = rvo.simulate(agent3,agent1,agent2,[local_goal.x,local_goal.y,0,0])
                     steer(-steering_angle, velocity)
                    except:
                        steer(0,0)
                    
                   
                        
                elif obstacles: 
                    # d_star_planner([position[0],position[1]], velocity, steering_angle, obstacles)
                    #this will get the planned path, and then create a tracker for the local planner
                    pass
            
            if local_active:
                if np.linalg.norm(np.array(position[0:2]) - np.array([local_goal.x,local_goal.y])) > 0.05 :
                    print("Moving towards local goal!") 
                else:
                    local_done = True
                    local_active = False
            
            
            if local_done == True and local_active == False:  #resume global plan     
                path = True
                tracker = PP(g_plan_smoothed,yaw)
                local_done = False 

                try :
                    velocity, steering_angle, time = tracker.execute(xpos = position[0], ypos = position[1], yaw = yaw, v = velocity ,time = time)
                    steer(steering_angle,velocity)

                except:
                    steer(0,0)

