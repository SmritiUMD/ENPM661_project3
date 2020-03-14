
import math
import numpy as np
from queue import PriorityQueue
import cv2
import time

grid_x = 200 #defining the grid width and height
grid_y = 300
theta_s=0
img = np.zeros((grid_x, grid_y), np.uint8)
r = input("enter value of radius") #taking input for radius

c = input("enter value of clearance") #taking input for clearance

d = int(r+c)
O=0
start_time = time.time() #taking the run time start
start_pos = [5, 195,theta_s]  #defining the start position as per the requirements
goal = [195,5] #defining the goal position as per the requirements
solvable = True

def obstacle(x,y):
    flag=True
    if (y>=(3/5)*x+(25-d)) and (y>=(-3/5)*x+(295-d)):
        flag=False
    elif (y>=(3/5)*x+55+d) or (y>=(-3/5)*x+325+d):
        flag=True
    elif (x-225)**2+(y-50)**2<=(25+d)**2:
        flag=False
    elif (x-150)**2/(40+d)**2+(y-100)**2/(20+d)**2<=1:
        flag=False
    elif (((x>=30.875 and x<=35.875) and (y>=((-1.71)*x+196.84-d))) or ((x>=35.875 and x<=100) and (y>=((0.53)*x+108.45-d)))):
        flag=False
    elif ((x>=30.875 and x<=95) and (y>=((0.5380)*x+118.99+d))) or ((x>=95 and x<=100) and (y>=((-1.71)*x+332.45+d))):
        flag=False
    elif (y>=(-7/5)*x+120 and y>=(7/5)*x-(90+d)) and (y<=(6/5)*x-10+d and y<=(-6/5)*x+170+d) :
        flag=False
    elif (y<=(-7/5)*x+120) and y<=(7/5)*x-20 and y>=15-d:
        flag=False
    elif y>=(7/5)*x-20 and y>=(-13)*x+340+d and y<=(-1)*x+100+d:
        flag=False
    elif (y>=0 and y<=0+d):
        flag=False
    elif (y<=200 and y>=(200-d)):
        flag=False
    elif (x>=0 and x<=d):
        flag=False
    elif (x<=300 and x>=(300-d)):
        flag=False
    else:False
    return flag
    
 #wriitng the condition for the case when start or goal node are defined in an obstacle
if (obstacle(start_pos[0],start_pos[1])==False or (obstacle(goal[0],goal[1])==False)):
        print("position not allowed")
        flag=False
else:
    pass