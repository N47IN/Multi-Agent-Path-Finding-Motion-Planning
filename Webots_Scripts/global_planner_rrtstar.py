import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from scipy.spatial import KDTree
import cv2
import random
import time

class Node:
    def __init__(self,x,y, parent):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0
    def update_cost(self):
        if self.parent:
            self.cost = self.parent.cost + np.sqrt((self.y - self.parent.y)**2 + (self.x - self.parent.x)**2)
        else:
            self.cost = 0
    def distance(self, node):
        return np.sqrt((self.y - node.y)**2 + (self.x - node.x)**2)

class RRT_star_planner:
    def __init__(self, map_image_src, rewire_radius, new_img):
        self.map_image = cv2.imread(map_image_src)
        self.shape = self.map_image.shape[:2]
        self.rewire_radius = rewire_radius
        self.new_img = new_img
    
    def ctop(self, point):
        y = min(self.shape[0]-1,int(point[0]/10*self.shape[0]))
        x = min(self.shape[1]-1,int(self.shape[1] - point[1]/10*self.shape[1]))
        return x,y 

    def displayPoints(self, Nodes, path = None):
        x_len = [a.x/10*self.shape[0] for a in Nodes]
        y_len = [self.shape[1] - a.y/10*self.shape[1] for a in Nodes]
        cv2.imwrite(self.new_img,self.map_image)
        image1 = cv2.imread(self.new_img)
        for i in range(len(x_len)):
            image1 = cv2.circle(image1,(int(x_len[i]), int(y_len[i])), 2, (0,255,0), -1)
            node = Nodes[i]
            if i>=1:
                line_start = self.ctop((node.x,node.y))[::-1]
                line_end = self.ctop((node.parent.x,node.parent.y))[::-1]
                cv2.line(image1, line_start, line_end, (0,0,0), 1)
        start_p = self.ctop(self.start)
        goal_p = self.ctop([self.goal[0],self.goal[1]])
        cv2.circle(image1,(start_p[1], start_p[0]), 5, (255,0,0), -1)
        cv2.circle(image1,(goal_p[1], goal_p[0]), 5, (0,0,255), -1)
        bigger = cv2.resize(image1, (1050, 1610))

        if self.goal_found:
            for i in range(1,len(path)):
                line_start = self.ctop((path[i][0],path[i][1]))[::-1]
                line_end = self.ctop((path[i-1][0],path[i-1][1]))[::-1]
                cv2.line(image1, line_start, line_end, (0,0,0), 3)
        cv2.imshow("Nodes", image1)
        cv2.waitKey(10)

    def displayPath(self, path):
        cv2.imwrite(self.new_img,self.map_image)
        image1 = cv2.imread(self.new_img)
        for i in range(1,len(path)):
            line_start = self.ctop((path[i][0],path[i][1]))[::-1]
            line_end = self.ctop((path[i-1][0],path[i-1][1]))[::-1]
            cv2.line(image1, line_start, line_end, (0,0,0), 3)
        cv2.imshow("Nodes", image1)
        cv2.waitKey(10)

    def collision_check(self, x,y,x_new,y_new):
        x,y = self.ctop([x,y])
        x_new , y_new = self.ctop([x_new,y_new])
        collision = False
        eps = 0.005
        try:
            line_x = np.linspace(x,x_new,100)
            line_y = y + (line_x - x)*(y_new - y)/(x_new - x + eps)
            avg_pixels = [int(x) for x in sum([self.map_image[int(line_x[i]), int(line_y[i])]/len(line_x) for i in range(len(line_x))])]
            if avg_pixels == [255, 255, 255]:
                collision = False
            else:
                # print(avg_pixels)
                collision = True
        except:
            collision = True
        return collision    

    def rewire(self,new_node,tree):
        nearest_point_indices = tree.query_ball_point((new_node.x, new_node.y),self.rewire_radius)
        # points = [self.node_list[index] for index in nearest_point_indices]
        for i in nearest_point_indices:
            if self.node_list[i].cost >  new_node.cost + new_node.distance(self.node_list[i]):
                if not(self.collision_check(self.node_list[i].x,self.node_list[i].y,new_node.x,new_node.y)):
                    self.node_list[i].parent = new_node
                    self.node_list[i].update_cost()

    def generate_node(self, sampled_pt):
        tree = KDTree([(node.x,node.y) for node in self.node_list])
        _, parent_index = tree.query(sampled_pt)
        parent = self.node_list[parent_index]
        x_init = self.node_list[parent_index].x
        y_init = self.node_list[parent_index].y
        angle = np.arctan2(sampled_pt[1]-y_init, sampled_pt[0]-x_init)
        x_new = x_init + np.cos(angle)*self.radius
        y_new = y_init + np.sin(angle)*self.radius
        new_node = Node(x_new,y_new,parent)
        new_node.update_cost()
        self.rewire(new_node,tree)
        return new_node

    def goal_check(self, node, goal_threshold = 0.2):
        dist = np.sqrt((self.goal[1] - node.y)**2 + (self.goal[0] - node.x)**2)
        if dist < goal_threshold:
            # self.goal_found = True
            return True
        else:
            return False

    def generate_path(self, node):
        path = []
        #print(node)
        while node != None :
            path.append([node.x,node.y])
            node.update_cost()
            node = node.parent
        return path[::-1]
    
    def setStart(self,start):
        self.start = start
        
    def setGoal(self,goal):
        self.goal = goal

    def RRT(self,mode = False,radius = 0.1, goal_bias = 0.05, runTime = 2):
        self.radius = radius
        self.fast_goal = False
        self.node_list = [] 

        curr_node = Node(self.start[0],self.start[1],parent=None)
        self.node_list.append(curr_node)
        self.goal_found = False
        self.goal_done = False #this will make the planner stop 2 seconds after goal has been found
        self.goal_flag = True
        found_time = 0
        path_length_prev = 10000
        optimal_threshold = 0.01
        path_stagnant = False
        found_iters = 0
        flag = 0
        while(not(self.goal_found) or not(path_stagnant) or len(self.node_list) < 4000):   #goal_found is added just for the corner case of goal lying between 2 nodes due to too large radius. 
            # print(len(self.node_list))
            p = np.random.random()
            goal_sampled = False
            if  p > goal_bias:
                x = np.random.random() * 10
                y = np.random.random() * 10
                sampled_pt = (x,y)
            else:
                sampled_pt = self.goal
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
                # print(len(self.node_list))
                if goal_sampled:
                    self.goal_found = self.goal_check(child)  #to check if the goal is less than 'radius' away. 
                    if self.fast_goal:    #Check if no obstacle to goal, then shoot into goal. 
                        if not(self.collision_check(self.goal[0], self.goal[1], child.x, child.y)):
                            self.goal_found = True
            
            if self.goal_found: 
                print("found", len(self.node_list))
                if self.goal_flag:
                    found_time = time.time()
                    self.goal_flag = False

                if time.time() - found_time > runTime:
                    # print("found", time.time() - found_time)
                    self.goal_done = True

                tree = KDTree([(node.x,node.y) for node in self.node_list])
                nearest_point_indices = tree.query_ball_point(self.goal, self.radius)
                if nearest_point_indices:
                    min_path = 100
                    min_index = 0
                    for i in nearest_point_indices:
                        if self.node_list[i].cost < min_path:
                            min_path = self.node_list[i].cost
                            min_index = i
                    goal_node = Node(self.goal[0], self.goal[1], self.node_list[min_index])
                    goal_node.update_cost()
                else:
                    _,parent_index = tree.query(self.goal)
                    goal_node = Node(self.goal[0], self.goal[1], self.node_list[parent_index])
                    goal_node.update_cost()
                
                path = self.generate_path(goal_node)
                # self.displayPath(path)
                path_length = goal_node.cost
                found_iters += 1
                # print(path_length)
                if found_iters > 2000:
                    path_improvement = path_length_prev - path_length
                    if path_improvement<0.005:
                        path_stagnant = True
                    path_length_prev = path_length

                # path_decrement = path_length_prev - path_length
                # path_length_prev = path_length
                # print(path_stagnant_count)
                # if path_decrement < optimal_threshold and flag:
                #     path_stagnant_count += 1
                #     flag = 1
                # else:
                #     flag = 0
            # print(path_length_prev)
                if mode and self.goal_found:
                    self.displayPoints(self.node_list, path)
                elif mode:
                    self.displayPoints(self.node_list)
        return path, self.node_list

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
    planner = RRT_star_planner("Webots_Scripts/Binary_Mask.png",0.5)
    start = [6,1]
    goal = [8,7]
    planner.setGoal(goal)
    planner.setStart(start)
    path = planner.RRT()
   
    #planner.displayPath(path)
