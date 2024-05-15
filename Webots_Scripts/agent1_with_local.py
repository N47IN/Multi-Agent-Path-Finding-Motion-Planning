# Talks at channel 1
from turtle import pos
from controller import Robot, GPS, Motor, Keyboard
from controller import InertialUnit
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
g_planner = RRT_star_planner("Binary_Mask.png",0.5, "image1.png")
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
    

def check_window(position, bot_pos, obs_pos, window_size = 100): #window size in pixels
    window_x = [position[0] - window_size//2, position[0] + window_size//2]
    window_y = [position[1] - window_size//2, position[1] + window_size//2]
    rvo_bots = []
    obstacles = []
    for i in range(len(bot_pos)):
        if window_x[0] < bot_pos[i][0] < window_x[1] and window_y[0] < bot_pos[i][1] < window_y[1]:
            rvo_bots.append(i)
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
    
    
    #print(position)
    broadcast(position)
    yaw = Yaw.getRollPitchYaw()
    yaw = yaw[2] 

    if admin.getQueueLength()>0 and path==False:
        goal = comms.getAdmin()[0:2]
        print("current position Agent1 :",position[0:2])
        print("goal Agent1 :",goal)
        g_planner.setGoal(position[0:2])
        g_planner.setStart([ goal[0], goal[1]])
        print(position[0:2])
        print([ goal[0], goal[1]])
        global_path, nodes = g_planner.RRT()
        global_path = np.asarray(global_path)
        print(global_path)
        g_plan_smoothed = ccma.filter(global_path, cc_mode=False)
        path = True
        tracker = PP(g_plan_smoothed,yaw)
        #print(g_plan)
        
    window_size = 100
    local_done = False
    local_active = False
    if path == True:
        if np.linalg.norm(position[0:2] - goal) > 0.05 :
            time2 = robot.getTime() - start_time
            
            #in every time step calc the distance between bots and obstacles and the robot itself
            if not(local_active):
                bot_pos = [comms.getAgent2()[0:2], comms.getAgent3()[0:2]]
                obstacle_pos = []
                rvo_bots, obstacles = check_window([position[0],position[1]], bot_pos, obstacle_pos)
                #if any of the local planners need to be activated, then activate them and re-define tracker
                if rvo_bots or obstacles:
                    tree = KDTree([(node.x,node.y) for node in nodes])
                    window_boundary_x = np.linspace(position[0] - window_size, position[0] + window_size, 20)
                    window_boundary_y = np.linspace(position[1] - window_size, position[1] + window_size, 20)
                    boundary_points = [[position[0] - window_size, y] for y in window_boundary_y]
                    boundary_points += [[position[0] + window_size, y] for y in window_boundary_y]
                    boundary_points += [[x, position[1] - window_size] for x in window_boundary_x]
                    boundary_points += [[x, position[1] + window_size] for x in window_boundary_x]
                    nearest_nodes_indices = tree.query_ball_point(boundary_points, 10)
                    nearest_node = nodes[np.argmin([nodes[i].cost for i in range(nearest_nodes_indices)])]
                    goal = nearest_node #pass this goal to local planners
                    
                    new_global_path = g_planner.generate_path(nearest_node)

                if rvo_bots:
                    # rvo_planner([position[0],position[1]], velocity, steering_angle, rvo_bots) 
                    #this will get the planned path, and then create a tracker for the local planner
                    pass        
                elif obstacles: 
                    # d_star_planner([position[0],position[1]], velocity, steering_angle, obstacles)
                    #this will get the planned path, and then create a tracker for the local planner
                    pass
            
            if local_active:
                #put local goal_check here to make sure local_done gets flagged once the local goal is reached
                #local_active = False
                pass
            
            if local_done:  #resume global plan     
                
                g_plan_smoothed = ccma.filter(new_global_path, cc_mode=False)
                path = True
                tracker = PP(g_plan_smoothed,yaw)
                local_done = False 

            try :
                velocity, steering_angle, time = tracker.execute(xpos = position[0], ypos = position[1], yaw = yaw, v = velocity ,time = time)
                steer(steering_angle,velocity)

            except:
                steer(0,0)

