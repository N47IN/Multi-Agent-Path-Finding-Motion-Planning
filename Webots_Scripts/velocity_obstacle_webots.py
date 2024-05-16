from multi_robot_plot import plot_robot_and_obstacles
from create_obstacles_webots import create_obstacle
from create_obst_from_img import create_obstacles_from_img
from control import compute_desired_velocity
import numpy as np
import math
import cv2

SIM_TIME = 5.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.1
VMAX = 2
VMIN = 0.2

class RVO:

    def __init__(self,goal,rob,obs1,obs2):
        self.goal = np.asarray([goal[0],goal[1],0,0])
        self.obs1 = obs1
        self.obs2 = obs2
        self.rob = rob
        image = cv2.imread("/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/Binary_Mask.png")
        
    
             
    def simulate(self,rob,obs1,obs2,goal,filename="MAPF"):
        image = cv2.imread("/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/Binary_Mask.png")
        #k = create_obstacles_from_img(image,SIM_TIME, NUMBER_OF_TIMESTEPS)
        self.obstacles = create_obstacle(rob,obs1,obs2,SIM_TIME, NUMBER_OF_TIMESTEPS)
        #print("obstacles are of the form", self.obstacles,"while", k)
        #self.obstacles = np.dstack((self.obstacles,k)) 
        start = np.array(rob)
        self.goal = goal
        robot_state = np.asarray([rob[0],rob[1],rob[2]*math.cos(rob[3]),rob[2]*math.sin(rob[3])])
        robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))
        v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX)
        v, theta = self.compute_velocity(
            robot_state, self.obstacles[:, :], v_desired)
        return v, theta
    
    def update(self,obs1,obs2,rob):
        self.obstacles = create_obstacle(rob,obs1,obs2,SIM_TIME, NUMBER_OF_TIMESTEPS)
        

    def compute_velocity(self,robot, obstacles, v_desired):
        pA = robot[:2]
        vA = robot[-2:]
        #print(np.shape(obstacles))
        # Compute the constraints for each velocity obstacles
        number_of_obstacles = np.shape(obstacles)[1]
        Amat = np.empty((number_of_obstacles * 2, 2))
        bvec = np.empty((number_of_obstacles * 2))
        for i in range(number_of_obstacles):
            obstacle = obstacles[:, i]

            pB = obstacle[:2]
            pB = [pB[0][0], pB[1][0]]
            vB = obstacle[2:]
            dispBA = pA - pB

            distBA = np.linalg.norm(dispBA)
            thetaBA = np.arctan2(dispBA[1], dispBA[0])

            if 2.2 * ROBOT_RADIUS > distBA:
                distBA = 2.2*ROBOT_RADIUS
            phi_obst = np.arcsin(2.2*ROBOT_RADIUS/distBA)

            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst
            # VO
            translation = vB

            translation = [translation[0],translation[1]]

            Atemp, btemp = self.create_constraints(translation, phi_left, "left")
            Amat[i*2, :] = Atemp
            bvec[i*2] = btemp
            Atemp, btemp = self.create_constraints(translation, phi_right, "right")
            Amat[i*2 + 1, :] = Atemp
            bvec[i*2 + 1] = btemp

        # Create search-space
        th = np.linspace(0, 2*np.pi, 20)
        vel = np.linspace(0, VMAX, 5)

        vv, thth = np.meshgrid(vel, th)

        vx_sample = (vv * np.cos(thth)).flatten()
        vy_sample = (vv * np.sin(thth)).flatten()

        v_sample = np.stack((vx_sample, vy_sample))

        v_satisfying_constraints = self.check_constraints(v_sample, Amat, bvec)

        # Objective function
        size = np.shape(v_satisfying_constraints)[1]
        diffs = v_satisfying_constraints - \
            ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
        norm = np.linalg.norm(diffs, axis=0)
        min_index = np.where(norm == np.amin(norm))[0][0]
        cmd_vel = (v_satisfying_constraints[:, min_index])
        theta = math.atan(cmd_vel[1]/cmd_vel[0])
        v = np.linalg.norm(cmd_vel)
        return v ,theta

    def check_constraints(self,v_sample, Amat, bvec):
        length = np.shape(bvec)[0]
        for i in range(int(length/2)):
            v_sample = self.check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])
        return v_sample

    def check_inside(self,v, Amat, bvec):
        v_out = []
        for i in range(np.shape(v)[1]):
            if not ((Amat @ v[:, i] < bvec).all()):
                v_out.append(v[:, i])
        return np.array(v_out).T

    def create_constraints(self,translation, angle, side):
        origin = np.array([0, 0, 1])
        point = np.array([np.cos(angle), np.sin(angle)])
        line = np.cross(origin, point)
        line = self.translate_line(line, translation)
        if side == "left":
            line *= -1

        A = line[:2]
        b = -line[2]
        return A, b

    def translate_line(self,line, translation):
        matrix = np.eye(3)
        matrix[2, 0] = -1*translation[0]
        matrix[2, 1] = -1*translation[1]
        return matrix @ line

    def update_state(self,x, v):
        new_state = np.empty((4))
        new_state[:2] = x[:2] + v * TIMESTEP
        new_state[-2:] = v
        return new_state

