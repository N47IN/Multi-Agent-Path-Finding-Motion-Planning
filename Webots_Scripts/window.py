import cv2
import numpy as np

W_LEN = 200
class Window():
    def __init__(self,map_image):
        self.map = cv2.imread(map_image)
        self.shape = np.shape(self.map)
        
    def setRobot(self,pos):
        center_x = int(pos[0]/10*self.shape[0])
        center_y = int(self.shape[1] - pos[1]/10*self.shape[1])
        size = 201  # Odd number to ensure the center point is included
        start_x = max(center_x - (size - 1) // 2, 0)
        end_x = min(center_x + (size - 1) // 2 + 1, self.shape[0])
        start_y = max(center_y - (size - 1) // 2, 0)
        end_y = min(center_y + (size - 1) // 2 + 1, self.shape[1])
        self.window = self.map[start_x:end_x, start_y:end_y]
    
    #def setGoal(self,pos)                    
    def showWindow(self):
        cv2.imshow("hi",self.window)
        cv2.waitKey(500)
        
win = Window("/home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/Binary_Mask.png")
win.setRobot([5,5])
win.showWindow()