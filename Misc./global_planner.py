import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from scipy.spatial import KDTree
import cv2
import random

class Node:
    def __init__(self,x,y, parent):
        self.x = x
        self.y = y
        self.parent = parent

class RRT_planner:
    def __init__(self, map_image_src):
        self.map_image = cv2.imread(map_image_src)
        self.shape = self.map_image.shape[:2]
    
    def ctop(self, point):
        y = min(self.shape[0]-1,int(self.shape[0] - point[0]/10*self.shape[0]))
        x = min(self.shape[1]-1,int(self.shape[1] - point[1]/10*self.shape[1]))
        return x,y 

    def displayPoints(self, Nodes):
        x_len = [self.shape[0] - a.x/10*self.shape[0] for a in Nodes]
        y_len = [self.shape[1] - a.y/10*self.shape[1] for a in Nodes]
        cv2.imwrite("Image.png",self.map_image)
        image1 = cv2.imread("Image.png")
        for i in range(len(x_len)):
            image1 = cv2.circle(image1,(int(x_len[i]), int(y_len[i])), 5, (0,255,0), -1)
            node = Nodes[i]
            if i>=1:
                line_start = self.ctop((node.x,node.y))[::-1]
                line_end = self.ctop((node.parent.x,node.parent.y))[::-1]
                cv2.line(image1, line_start, line_end, (0,0,0), 1)
        start_p = self.ctop(start)
        goal_p = self.ctop(goal)
        cv2.circle(image1,(start_p[1], start_p[0]), 5, (255,0,0), -1)
        cv2.circle(image1,(goal_p[1], goal_p[0]), 5, (0,0,255), -1)
        cv2.imshow("Nodes", image1)
        cv2.waitKey(10)

    def displayPath(self, path):
        cv2.imwrite("Image.png",self.map_image)
        image1 = cv2.imread("Image.png")
        for i in range(1,len(path)):
            line_start = self.ctop((path[i][0],path[i][1]))[::-1]
            line_end = self.ctop((path[i-1][0],path[i-1][1]))[::-1]
            cv2.line(image1, line_start, line_end, (0,0,0), 3)
        cv2.imshow("Nodes", image1)
        cv2.waitKey(3000)

    def collision_check(self, x,y,x_new,y_new):
        x,y = self.ctop([x,y])
        x_new , y_new = self.ctop([x_new,y_new])
        collision = False
        try:
            line_x = np.linspace(x,x_new,100)
            line_y = y + (line_x - x)*(y_new - y)/(x_new - x)
            avg_pixels = [int(x) for x in sum([self.map_image[int(line_x[i]), int(line_y[i])]/len(line_x) for i in range(len(line_x))])]
            if avg_pixels == [255, 255, 255]:
                collision = False
            else:
                # print(avg_pixels)
                collision = True
        except:
            collision = True
        return collision    

    def generate_node(self, sampled_pt):
        tree = KDTree([(node.x,node.y) for node in self.node_list])
        _, parent_index = tree.query(sampled_pt)
        parent = self.node_list[parent_index]
        x_init = self.node_list[parent_index].x
        y_init = self.node_list[parent_index].y
        angle = np.arctan2(sampled_pt[1]-y_init, sampled_pt[0]-x_init)
        x_new = x_init + np.cos(angle)*self.radius
        y_new = y_init + np.sin(angle)*self.radius
        return Node(x_new,y_new,parent)

    def goal_check(self, node, goal_threshold = 0.2):
        dist = np.sqrt((self.goal[1] - node.y)**2 + (self.goal[0] - node.x)**2)
        if dist < goal_threshold:
            # self.goal_found = True
            return True
        else:
            return False

    def generate_path(self, node):
        path = []
        while node != None:
            path.append([node.x,node.y])
            node = node.parent
        return path[::-1]
    
    def setStart(self,start):
        self.start = start
        
    def setGoal(self,goal):
        self.goal = goal

    def RRT(self,radius = 0.1, goal_bias = 0.05):
        self.radius = radius
        self.fast_goal = False
        self.node_list = [] 
        self.goal_found = False
        curr_node = Node(self.start[0],self.start[1],parent=None)
        self.node_list.append(curr_node)
        goal_found = False
        while(not(self.goal_check(self.node_list[-1]) or goal_found)):   #goal_found is added just for the corner case of goal lying between 2 nodes due to too large radius. 
            p = np.random.random()
            goal_sampled = False
            if  p > goal_bias:
                x = np.random.random() * 10
                y = np.random.random() * 10
                sampled_pt = (x,y)
            else:
                sampled_pt = goal
                goal_sampled = True
                # print('goal sampling')
            child = self.generate_node(sampled_pt)
            Xp,Yp = self.ctop([child.x,child.y])
            if self.collision_check(child.parent.x,child.parent.y,child.x,child.y):
                # print(child.parent.x,child.parent.y, child.x, child.y)
                # print("collido")
                continue
            elif Xp < 0 or Yp < 0 :
                # print("outside")
                continue
            else:
                self.node_list.append(child)
                if goal_sampled:
                    goal_found = self.goal_check(child)  #to check if the goal is less than 'radius' away. 
                    if self.fast_goal:    #Check if no obstacle to goal, then shoot into goal. 
                        if not(self.collision_check(self.goal[0], self.goal[1], child.x, child.y)):
                            goal_found = True
            self.displayPoints(self.node_list)
        self.node_list.append(Node(self.goal[0], self.goal[1], self.node_list[-1]))
        path = self.generate_path(self.node_list[-1])
        return path

# def RRT_explore(node_list,goal,image,radius = 1):
#         print("EXPLORING")
#         curr_node = node_list[-1]
#         # radius = min(1.25, 0.5*np.sqrt((goal[1] - curr_node.y)**2 + (goal[0] - curr_node.x)**2))
#         tree = KDTree([(node.x,node.y) for node in node_list])
#         _, new_parent_index = tree.query(child)
#         parent = node_list[new_parent_index]
#         child =Node(child[0],child[1],parent)
#         Xp,Yp = ctop([child.x,child.y])
#         if collision_check(curr_node.x,curr_node.y,child.x,child.y,):
#             print("collido")
#         elif Xp < 0 or Yp < 0 :
#             print("outside")
#         else:
#             node_list.append(child)
#         displayPoints(node_list,image)

if __name__ == '__main__':
    # map = readMap()
    planner = RRT_planner("Binary_Mask.png")
    start = [6,1]
    goal = [8,9]
    planner.setGoal(goal)
    planner.setStart(start)
    path = planner.RRT()
   
    planner.displayPath(path)
