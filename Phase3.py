import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse

class Obstacle():
    def __init__(self, width = 1000, height = 1020, r = 1, c = 1, threshold=0.5):
        self.threshold = threshold #### Resolution
        self.W = int(width/threshold) 
        self.H = int(height/threshold) 
        self.r = r
        self.c = c
        ### all angles and cost and selfID
        ### Fourth dimention [ cost , x , y , theta ] -- of parent
        self.explored = np.zeros([self.H, self.W, 360//thetaStep, 4])
        ### [ startX , startY , endX , endY ]
        self.plotData_X = []
        self.plotData_Y = []
        self.plotData_U = []
        self.plotData_V = []



        def ObsCheck(self, i, j):
        # Check all the obstacles
        if self.checkBoundary(i,j):
            return False
        elif self.checkInCircle(i,200-j,(700,200),20):
            return False
        
        elif self.checkInQuad(i,1020-j,(25,525), (175, 525), (175, 475), (25,475)):
            return False
        elif self.checkInQuad(i,1020-j,(845, 525), (995, 525), (995, 475), (845, 475)):
            return False
        elif self.checkInQuad(i,1020-j,(225,175), (375,175), (375,125), (225,125)):
            return False
        
        else:
            return True


        def checkInCircle(self, i, j, center, radius):
        ## i = x-direction
        ## j = y-direction
        center_x, center_y = center[0], center[1]
        if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
            return True
        else:
            return False


        def checkInQuad1(self, i, j, vertex1, vertex2, vertex3, vertex4):
            x1, y1 = vertex1[0], vertex1[1]
            x2, y2 = vertex2[0], vertex2[1]
            x3, y3 = vertex3[0], vertex3[1]
            x4, y4 = vertex4[0], vertex4[1]
            if (x>x1 and x<=x2) and (y>=y1 and y<=y2):
                return True
            return False
        
