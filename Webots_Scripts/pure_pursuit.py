
import numpy as np
import math
import matplotlib.pyplot as plt
from ccma import CCMA

# Parameters
k = 0.7  # look forward gain
Lfc = 0.05  # [m] look-ahead distance
Kp = 0.43  # speed proportional gain
dt = 0.032  # [s] time tick
WB = 0.13  # [m] wheel base of vehicle

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        
    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)
    return a

w_ma = 4
w_cc = 2

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            
            print(state.rear_x)
            print(state.rear_y)
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            print(ind)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        Lf = k * state.v + Lfc  # update look ahead distance
        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
        return ind, Lf

def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)
    if pind >= ind:
        ind = pind
    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    return delta, ind

class PP:
    def __init__(self,path,yaw):
        path =np.asarray(path)
        cx = path[:,0]
        cy = path[:,1]
        target_speed = 0.2  # [m/s]
        self.T = 100.0  # max simulation time
        self.state = State(x=cx[0], y=cy[1], yaw=yaw, v=0.0)
        self.lastIndex = len(cx) - 1
        self.time = 0.0
        self.states = States()
        self.states.append(self.time, self.state)
        self.target_course = TargetCourse(cx, cy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)
        
    def execute(self,xpos,ypos,yaw,v,time):
        self.state = State(x=xpos, y=ypos, yaw=yaw, v=v)    
        ai = proportional_control(0.2, self.state.v)
        di, self.target_ind = pure_pursuit_steer_control(
        self.state, self.target_course, self.target_ind)
        self.state.update(ai, di)  # Control vehicle      
        self.time += dt
        return self.state.v, di, time