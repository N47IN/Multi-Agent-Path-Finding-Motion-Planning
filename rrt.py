import numpy as np
import matplotlib.pyplot as plt

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

class Map:
    def __init__(self):
        self.x_size = 10
        self.y_size = 10
        self.obstacles = [Object((3,3),1,4), Object((7,4),2,6),Object((4,6),3,3)]