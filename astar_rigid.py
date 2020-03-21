import sys
sys.path.remove(sys.path[1])
import math
import numpy as np
from queue import PriorityQueue
from heapq import heappush, heappop
import cv2
import time
import matplotlib.pyplot as plt

class Obstacle():
    def __init__(self, width = 300, height = 200, r = 1, c = 1, threshold=0.5, thetaStep = 30):
        self.threshold = threshold #### Resolution
        self.W = int(width/threshold) 
        self.H = int(height/threshold) 
        self.r = r
        self.c = c
        self.thetaStep = thetaStep ###
        ### all angles and cost and selfID
        ### Fourth dimention [ x , y , theta , cost ] -- of parent
        self.explored = np.zeros([self.H, self.W, 360//thetaStep, 4])
        ### [ startX , startY , endX , endY ]
        self.plotData_X = []
        self.plotData_Y = []
        self.plotData_U = []
        self.plotData_V = []

    def plotSpace(self, ax):
        ## Polygons
        polygon_1 = [[20,25,75,100,75,50,20],[120,185,185,150,120,150,120]]
        ang = math.radians(30)
        polygon_2 = [[95, 30],
                     [95 - 75*math.cos(ang), 30 + 75*math.sin(ang)],
                     [95 - 75*math.cos(ang) + 10*math.sin(ang), 30 + 75*math.sin(ang) + 10*math.cos(ang)],
                     [95 + 10*math.sin(ang), 30 + 10*math.cos(ang)],
                     [95, 30]] 
        polygon_2_X, polygon_2_Y = [i for i,j in polygon_2], [j for i,j in polygon_2]          
        polygon_3 = [[225, 200, 225, 250, 225],
                     [10, 25, 40, 25, 10]]
        #### Circle
        centX, centY,radii = 225,150,25 
        circleX = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        circleY = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        ### Ellipse
        centX, centY, a, b = 150, 100, 40, 20
        ellipseX = [centX+a*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        ellipseY = [centY+b*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        
        plt.plot(polygon_1[0],polygon_1[1])
        plt.plot(polygon_2_X, polygon_2_Y)
        plt.plot(polygon_3[0], polygon_3[1])
        plt.plot(circleX, circleY)
        plt.plot(ellipseX, ellipseY)
        # testX = []
        # testY = []
        # for i in range(0,300,10):
        #     for j in range(0,200,10):
        #         if self.ObsCheck(i,j):
        #             plt.scatter(i,200-j,c='r') 
        # plt.show()
        return ax

    def ObsCheck(self, i, j):
        # print(self.r)
        if self.checkBoundary(i,j):
            return False
        elif self.checkInCircle(i,200-j,(225,50),25):
            return False
        elif self.chekckInEllipse(i,200-j,(150, 100), 20, 40):
            return False
        elif self.checkInQuad1(i,200-j,(25, 15), (75, 15), (50, 50), (20, 80), (100, 50)):
            return False
        elif self.checkInQuad2(i,200-j,(75, 15), (100, 50), (75, 80), (50, 50), (25, 15)):
            return False
        elif self.checkInQuad3(i,200-j,(225, 160), (250, 175), (225, 190), (200, 175)):
            return False
        elif self.checkInQuad4(i,200-j,(35, 123), (100, 161), (95, 170), (30, 132)):
            return False
        else:
            return True

    def checkBoundary(self,i,j):
        # print(i,j)
        if i < (300 - self.r - self.c) and i > (self.r + self.c) and j < (200 - self.r - self.c) and j > (self.r + self.c):
            # print("In")
            return False
        return True
    
    def checkInCircle(self, i, j, center, radius):
        ## i = x-direction
        ## j = y-direction
        center_x, center_y = center[0], center[1]
        if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
            return True
        else:
            return False

    def chekckInEllipse(self, i, j, center, semiminor, semimajor):
        center_x, center_y = center[0], center[1]
        if (((i - center_x) / (semimajor + self.r + self.c)) ** 2 + ((j - center_y) / (semiminor + self.r + self.c)) ** 2) <= 1:
            return True
        else:
            return False

    def checkInQuad1(self, i, j, vertex1, vertex2, vertex3, vertex4, vertex5):
        x1, y1 = vertex1[0], vertex1[1]
        x2, y2 = vertex2[0], vertex2[1]
        x3, y3 = vertex3[0], vertex3[1]
        x4, y4 = vertex4[0], vertex4[1]
        x5, y5 = vertex5[0], vertex5[1]
        m1 = (y2 - y1) / (x2 - x1)
        m2 = (y3 - y2) / (x3 - x2)
        m3 = (y4 - y3) / (x4 - x3)
        m4 = (y1 - y4) / (x1 - x4)
        m5 = (y5 - y2) / (x5 - x2)
        x = [x1, x2, x3, x4, x5]
        y = [y1, y2, y3, y4, y5]
        if (j >= m1 * i + y1 - m1 * x1 - ((self.r + self.c) * math.sqrt((m1 ** 2) + 1))) and (
                j <= m2 * i + y2 - m2 * x2 + ((self.r + self.c) * math.sqrt((m2 ** 2) + 1))) and (
                j <= m3 * i + y3 - m3 * x3 + ((self.r + self.c) * math.sqrt((m3 ** 2) + 1))) and (
                j >= m4 * i + y4 - m4 * x4 - ((self.r + self.c) * math.sqrt((m4 ** 2) + 1))):
            return True
        if j <= m5 * i + y5 - m5 * x5 - ((self.r + self.c) * math.sqrt((m5 ** 2) + 1)):
            return False
        return False

    def checkInQuad2(self, i, j, vertex1, vertex2, vertex3, vertex4, vertex5):
        x1, y1 = vertex1[0], vertex1[1]
        x2, y2 = vertex2[0], vertex2[1]
        x3, y3 = vertex3[0], vertex3[1]
        x4, y4 = vertex4[0], vertex4[1]
        x5, y5 = vertex5[0], vertex5[1]
        x = [x1, x2, x3, x4, x5]
        y = [y1, y2, y3, y4, y5]
        m1 = (y2 - y1) / (x2 - x1)
        m2 = (y3 - y2) / (x3 - x2)
        m3 = (y4 - y3) / (x4 - x3)
        m4 = (y1 - y4) / (x1 - x4)
        m5 = (y5 - y1) / (x5 - x1)
        if (j >= m1 * i + y1 - m1 * x1 - ((self.r + self.c) * math.sqrt((m1 ** 2) + 1))) and (
                j <= m2 * i + y2 - m2 * x2 + ((self.r + self.c) * math.sqrt((m2 ** 2) + 1))) and (
                j <= m3 * i + y3 - m3 * x3 + ((self.r + self.c) * math.sqrt((m3 ** 2) + 1))) and (
                j >= m4 * i + y4 - m4 * x4 - ((self.r + self.c) * math.sqrt((m4 ** 2) + 1))):
            return True
        if j <= m5 * i + y1 - m5 * x1 - ((self.r + self.c) * math.sqrt((m5 ** 2) + 1)):
            return False
        return False

    def checkInQuad3(self, i, j, vertex1, vertex2, vertex3, vertex4):
        x1, y1 = vertex1[0], vertex1[1]
        x2, y2 = vertex2[0], vertex2[1]
        x3, y3 = vertex3[0], vertex3[1]
        x4, y4 = vertex4[0], vertex4[1]
        m1 = (y2 - y1)/(x2 - x1)
        m2 = (y3 - y2)/(x3 - x2)
        m3 = (y4 - y3)/(x4 - x3)
        m4 = (y1 - y4)/(x1 - x4)
        if (j >= m1*i + y1 - m1*x1 - ((self.r + self.c)*math.sqrt((m1**2)+1))) and (
                j <= m2*i + y2 - m2*x2 + ((self.r + self.c)*math.sqrt((m2**2)+1))) and (
                j <= m3*i + y3 - m3*x3 + ((self.r + self.c)*math.sqrt((m3**2)+1))) and (
                j >= m4*i + y4 - m4*x4 - ((self.r + self.c)*math.sqrt((m4**2)+1))):
            return True
        return False

    def checkInQuad4(self, i, j, vertex1, vertex2, vertex3, vertex4):
        x1, y1 = vertex1[0], vertex1[1]
        x2, y2 = vertex2[0], vertex2[1]
        x3, y3 = vertex3[0], vertex3[1]
        x4, y4 = vertex4[0], vertex4[1]
        m1 = (y2 - y1) / (x2 - x1)
        m2 = (y3 - y2) / (x3 - x2)
        m3 = (y4 - y3) / (x4 - x3)
        m4 = (y1 - y4) / (x1 - x4)
        if (j >= m1 * i + y1 - m1 * x1 - ((self.r + self.c) * math.sqrt((m1 ** 2) + 1))) and (
                j <= m2 * i + y2 - m2 * x2 + ((self.r + self.c) * math.sqrt((m2 ** 2) + 1))) and (
                j <= m3 * i + y3 - m3 * x3 + ((self.r + self.c) * math.sqrt((m3 ** 2) + 1))) and (
                j >= m4 * i + y4 - m4 * x4 - ((self.r + self.c) * math.sqrt((m4 ** 2) + 1))):
            return True
        return False

    def checkVisited(self, node):
        #### node = [ cost , x , y , angle ]
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        print(node[1], node[2])
        print(checkPosX, checkPosY)
        if self.explored[checkPosY, checkPosX, checkPosA,3] != 0:
            return True ##### Yes...it is visited
        else:
            return False ##### Not visited

    def discret(self,node):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        return [node[0], checkPosY, checkPosX, checkPosA]

    def findVisited(self, node):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        # print(checkPosX, checkPosY, checkPosA)
        return self.explored[checkPosY, checkPosX, checkPosA, :]

    def addVisited(self, node, parentNode):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        self.plotData_X.append(parentNode[1])
        self.plotData_Y.append(parentNode[2])
        self.plotData_U.append(node[1] - parentNode[1]) 
        self.plotData_V.append(node[2] - parentNode[2])
        # self.explored[checkPosX, checkPosY, checkPosA, 3] = newCost
        self.explored[checkPosY, checkPosX, checkPosA, :] = np.array(parentNode)
        return

    def plotAll(self, path):
        plt.ion()
        fig, ax = plt.subplots()
        # fig.canvas.manager.full_screen_toggle() # toggle fullscreen mode
        # fig.show()
        ax = self.plotSpace(ax)
        for i in range(1,len(self.plotData_X)+1,3000):
            # plt.cla()
            plt.xlim(0,300)
            plt.ylim(0,200)
            q = ax.quiver(self.plotData_X[i:(i+3000)], self.plotData_Y[i:(i+3000)], 
                self.plotData_U[i:(i+3000)], self.plotData_V[i:(i+3000)], units='xy' ,
                scale = 1, headwidth = 0.1, headlength=0,
                width=0.2)
            plt.pause(0.0001)

        X,Y,U,V = [],[],[],[] 
        for i in range(len(path)-1):
            X.append(path[i][1])
            Y.append(path[i][2])
            U.append(path[i+1][1] - path[i][1])
            V.append(path[i+1][2] - path[i][2])
            
        for i in range(len(X)):
            # plt.cla()
            plt.xlim(0,300)
            plt.ylim(0,200)
            # plt.xlim(0,self.W)
            # plt.ylim(0,self.H)
            # L = ax.quiver(self.plotData_X, self.plotData_Y, 
            #     self.plotData_U, self.plotData_V, units='xy',
            #     scale=1, headwidth = 0.1, headlength=0,
            #     width=0.2)
            q = ax.quiver(X[i], Y[i], U[i], V[i], units='xy', 
                scale=1, color='r', headwidth = 0.1, 
                headlength=0, width = 0.7)
            plt.pause(0.0001)
            # print(self.plotData_X[i],self.plotData_Y[i],self.plotData_U[i],self.plotData_V[i])
        plt.ioff()
        plt.show()

class pathFinder():
    def __init__(self, initial, goal, thetaStep = 30, stepSize = 1, goalThreshold = 2,
        width = 300, height = 200, threshold = 0.5):
        self.initial = initial
        self.goal = goal
        ##### node = [ x , y , angle , cost ]
        self.nodeData = []
        ##### [ cost , selfID , parentID ]
        ##### selfID and parentID are index in nodeData
        ##### [ cost , x , y , angle ]
        self.Data = []
        self.allData = []
        self.thetaStep = thetaStep
        self.stepSize = stepSize
        self.goalThreshold = goalThreshold
        self.setActions()
        self.obstacle = Obstacle(width, height, r = 1, c = 1, threshold=0.5, thetaStep=self.thetaStep)

    def setActions(self):
        self.actionSet = []
        for angle in np.arange(0, 360, self.thetaStep):
            ang = math.radians(angle)
            x = self.stepSize*math.cos(ang)
            y = self.stepSize*math.sin(ang)
            costToCome = 1
            #### Action  --- [ x , y , angle , cost ] ----
            self.actionSet.append([x, y, angle, costToCome])
        pass

    def initialCheck(self):
    #writing the condition for the case when start or goal node are defined in an obstacle
        # if (self.obstacle(initial[0],initial[1])==False or (self.obstacle(goal[0],goal[1])==False)):
        if not self.obstacle.ObsCheck(self.goal[0], 200-self.goal[1]):
            print("Goal in obstacle field")
            return False
        elif not self.obstacle.ObsCheck(self.initial[0], 200-self.goal[1]):
            print("Initial position in obstavle field")
            return False
        else:
            # heappush(self.Data, [0,0,0])
            cost = math.sqrt((self.initial[0] - self.goal[0])**2 + (self.initial[1] - self.goal[1])**2)
            heappush(self.Data, [cost, self.initial[0], self.initial[1], self.initial[2], 0])
            # self.allData([0,0,0])
            self.nodeData.append([self.initial[0], self.initial[1], self.initial[2], 0])
            return True

    def heuristics(self, current): #defining heuristic function as euclidian distance between current node and goal
        h = math.sqrt((current[1] - self.goal[0])**2 + (current[2] - self.goal[1])**2)
        # h = math.sqrt((current[0] - self.goal[0])**2 + (current[1] - self.goal[1])**2 + (current[2] - self.goal[2])**2)
        return h

    def goalReached(self, current):  # function to check if the explored point is inside threshold area around the goal or not
        x, y = current[1], current[2]
        # ((x - goal[0])**2 + (y - goal[1])**2 <= (self.goalThreshold)**2) and (abs(self.goal[2] - current[2]) <= angleThreshold):
        if (x - goal[0])**2 + (y - goal[1])**2 <= (self.goalThreshold)**2:
            return True
        else:
            return False

    def trackBack(self, presentNode):
        track = []
        currentNode = presentNode[:4]
        # track.append(self.goal)
        track.append(currentNode)
        while currentNode[1:] != self.initial:
            # print(1)
            currentNode = list(self.obstacle.findVisited(currentNode))
            # print(currentNode)
            track.append(currentNode)
        print("-------------------")
        print("Trackback")
        # print(track)
        track.reverse()
        return track

    def findPath(self):
        if self.initialCheck():
            while len(self.Data)>0:
                presentNode = heappop(self.Data)
                previousCost, previousCostToCome = presentNode[0], presentNode[4]
                if self.goalReached(presentNode):
                    self.goalReach = True
                    print(" Goal Reached ")
                    print(presentNode)
                    # self.obstacle.addVisited(presentNode)
                    path = self.trackBack(presentNode)
                    print(path)
                    self.obstacle.plotAll(path)
                    return
                for action in self.actionSet:
                    ##### node = [ x , y , angle , cost]
                    ##### Data = [ cost , selfID , parentID ]
                    newNodeX = presentNode[1] + action[0]
                    newNodeY = presentNode[2] + action[1]
                    newNodeA = action[2]
                    newNode = [0, newNodeX, newNodeY, newNodeA, 0]
                    newCostToCome = previousCostToCome + action[3]
                    newNode[4] = newCostToCome
                    costToGo = self.heuristics(newNode)
                    # newNode[0] = newCost
                    # print("Found a new node " + str(newNode))
                    # print(newNode)
                    if self.obstacle.ObsCheck(newNodeX, newNodeY):
                        if not self.obstacle.checkVisited(newNode):
                            ##### Node is not visited so add to data
                            presentNode[0] = newCostToCome
                            self.obstacle.addVisited(newNode, presentNode[:4])
                            newNode[0] = newCostToCome + costToGo
                            heappush(self.Data, newNode)
                        else: #### Node is visited so check previous cost
                            previousVisited = self.obstacle.findVisited(newNode)
                            previousCost = previousVisited[0]
                            if previousCost > newCostToCome:
                                presentNode[0] = newCostToCome
                                self.obstacle.addVisited(newNode, presentNode[:4])
                                
        print("Could not reach goal..")
        return

initial = [50,30,60]
goal = [150,150,0]
solver = pathFinder(initial, goal, thetaStep=30, stepSize=2)
solver.findPath()
# solver.obstacle.plotSpace()