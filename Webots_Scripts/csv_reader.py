import csv
import numpy as np
data = list(csv.reader(open("/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/path.csv")))
x= np.asfarray(data)

print(x)