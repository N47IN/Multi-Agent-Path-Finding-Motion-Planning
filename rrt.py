import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from scipy.spatial import KDTree


class Node:
    def __init__(self,x,y, parent):
        self.x = x
        self.y = y
        self.parent = parent

class Object:
    def __init__(self,centroid = (2,2), radius = 3, num_edges = 4):
        self.centroid = centroid
        self.num_edges = num_edges
        self.radius = radius
        self.vertices = []
        for i in range(self.num_edges):
            theta = (np.random() - 1)*np.pi
            self.vertices.append((self.centroid[0] + self.radius*np.cos(theta), self.centroid[1] + self.radius*np.sin(theta)))
        self.path = mplPath.Path(self.vertices)

class Map:
    def __init__(self):
        self.x_size = 10
        self.y_size = 10
        self.obstacles = [Object((3,3),1,4), Object((7,4),2,6),Object((4,6),3,3)]

def collision_check(x,y,x_new,y_new,map):
    line_x = np.arange(x,x_new,0.05)
    line_y = y + (line_x - x)*(y_new - y)/(x_new - x)
    collision = False
    for obj in map.obstacles:
        if obj.contains_point((x_new,y_new)):
            collision = True
            break
        if True in obj.contains_points(zip(line_x,line_y)):
            collision = True
            break
    return collision

def generate_node(parent,radius):
    angle = np.pi*(np.random() - 1)
    x_new = parent.x + radius*np.cos(angle)
    y_new = parent.x + radius*np.sin(angle)
    return Node(x_new,y_new,parent)

def goal_check(node, goal, goal_threshold = 0.01):
    if np.sqrt((goal[1] - node.y)**2 + (goal[0] - node.x)**2) < goal_threshold:
        return True
    else:
        return False

def RRT(start, goal,radius = 0.5):
    node_list = []
    node_list.append(start)
    curr_node = Node(start[0],start[1],parent=None)
    while not(goal_check(curr_node,goal)):
        tree = KDTree([(node.x,node.y) for node in node_list])
        _, new_parent_index = tree.query(goal)
        curr_node = node_list[new_parent_index]
        node_list.append(generate_node(curr_node,radius))

map = Map()
start = (1,1)
goal = (9,8)