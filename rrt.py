import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from scipy.spatial import KDTree
import cv2
import random

def readMap():
    # Read Map 
    img = cv2.imread("Maps/mapya.png")
    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    r_channel, g_channel, b_channel = cv2.split(image_rgb)
    r_thresh = 200
    g_thresh = 100
    b_thresh = 70
    # consider blue channel since b_brown = 0
    r_threshed = cv2.threshold(r_channel, r_thresh, 255, cv2.THRESH_BINARY)[1]
    g_threshed = cv2.threshold(g_channel, g_thresh, 255, cv2.THRESH_BINARY)[1]
    b_threshed = cv2.threshold(b_channel, b_thresh, 255, cv2.THRESH_BINARY)[1]
    # Denoise to remove grains
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    binaryMask = cv2.morphologyEx(b_threshed, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite("Binary_Mask.png", binaryMask)
    #cv2.waitKey(0)
    return binaryMask

# cartesian to pixel
def ctop(point):
    x = int(shape[0] - point[0]/10*shape[0])
    y = int(shape[1] - point[1]/10*shape[1])
    return x,y 

def displayPoints(Nodes,image):
    x_len = [shape[0] - a.x/10*shape[0] for a in Nodes]
    y_len = [shape[1] - a.y/10*shape[1] for a in Nodes]
    cv2.imwrite("Image.png",image)
    image1 = cv2.imread("Image.png")
    for i in range(len(x_len)):
        image1 = cv2.circle(image1,(int(x_len[i]), int(y_len[i])), 5, (0,255,0), -1)
    cv2.imshow("Nodes", image1)
    cv2.waitKey(500)

""" def displayGoal(goal,start,image):
    image = cv2.circle(image,(int(goal[0]), int(goal[1])), 10, (0,0,255), -1)
    image = cv2.circle(image,(int(start[0]), int(start[1])), 10, (255,0,0), -1)
    cv2.imshow("square_circle_opencv.jpg", image)
    cv2.waitKey(0) """

class Node:
    def __init__(self,x,y, parent):
        self.x = x
        self.y = y
        self.parent = parent

class Map:
    def __init__(self):
        self.x_size = 10
        self.y_size = 10
        self.goal = []

def collision_check(x,y,x_new,y_new):
    x,y = ctop([x,y])
    x_new , y_new = ctop([x_new,y_new])
    line_x = np.arange(x,x_new,0.05)
    line_y = y + (line_x - x)*(y_new - y)/(x_new - x)
    line_y = [int(x) for x in line_y]
    collision = False
    image1 = cv2.imread("Image.png")
    print(image[x_new-1,y_new-1])
    for i in range(len(line_x)-2):
        if all(image[int(line_x[i]),line_y[i]] == [0,0,0]):
            collision = True
            image1 = cv2.circle(image1,(int(line_x[i]), line_y[i]), 5, (0,255,0), -1)
            cv2.imshow("hii",image1)
            cv2.waitKey(5000)
            print(collision)
            break
    return collision

def generate_node(parent,radius):
    angle = np.pi*(np.random.random() * 2)
    x_new = max(min(parent.x + radius*np.cos(angle),10), 0)
    y_new = max(min(parent.y + radius*np.sin(angle),10), 0)
    return Node(x_new,y_new,parent)

def goal_check(node, goal, goal_threshold = 0.1):
    if np.sqrt((goal[1] - node.y)**2 + (goal[0] - node.x)**2) < goal_threshold:
        return True
    else:
        return False

def RRT(start, goal,image,radius = 1):
    node_list = []
    curr_node = Node(start[0],start[1],parent=None)
    node_list.append(curr_node)
    while not(goal_check(curr_node,goal)):
        tree = KDTree([(node.x,node.y) for node in node_list])
        _, new_parent_index = tree.query(goal)
        curr_node = node_list[new_parent_index]
        radius = min(0.5, 0.5*np.sqrt((goal[1] - curr_node.y)**2 + (goal[0] - curr_node.x)**2))
        #radius = 1.25
        child = generate_node(curr_node,radius)
        Xp,Yp = ctop([child.x,child.y])
        if collision_check(curr_node.x,curr_node.y,child.x,child.y,):
            continue
        elif Xp < 0 or Yp < 0 :
            continue
        else:
            node_list.append(child)
        displayPoints(node_list,image)
    print("Found Goal")



readMap()
image = cv2.imread("Binary_Mask.png")
shape = image.shape[:2]
start = [0.3,0.3]
goal = [6,7]
RRT(start,goal,image)
