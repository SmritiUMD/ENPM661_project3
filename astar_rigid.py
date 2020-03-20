import sys
sys.path.remove(sys.path[1])
import math
import numpy as np
from queue import PriorityQueue
from heapq import heappush, heappop
import cv2
import time
import matplotlib.pyplot as plt

# r = input("enter value of radius") #taking input for radius

# c = input("enter value of clearance") #taking input for clearance
# R = int(input("enter value of resolution")) #taking input for resolution


# d = int(r+c)
# solvable=True

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

    def CheckInObstacle(self, x, y):
        d = self.r + self.c
        R = 1
        flag = True
        if (y>=(3/5)*x+(25-d)/R) and (y>=(-3/5)*x+(295-d)/R):
            flag = False
        elif (y>=(3/5)*x+(55+d)/R) or (y>=(-3/5)*x+(325+d)/R):
            flag = True
        elif (x-math.ceil(225/R))**2+(y-math.ceil(50/R))**2<=(math.ceil((25+d)/R))**2:
            flag = False
        elif (x-(math.ceil(150/R)))**2/(math.ceil((40+d)/R))**2+(y-(math.ceil(100/R)))**2/(math.ceil((20+d)/R))**2<=1:
            flag = False
        elif (((x>=30.875/R and x<=35.875/R) and (y>=((-1.71)*x+(196.84-d)/R))) or ((x>=35.875/R and x<=100/R) and (y>=((0.53)*x+(108.45-d)/R)))):
            flag = False
        elif ((x>=30.875/R and x<=95/R) and (y>=((0.5380)*x+(118.99+d)/R))) or ((x>=95/R and x<=100/R) and (y>=((-1.71)*x+(332.45+d)/R))):
            flag = True
        elif (y>=(-7/5)*x+120/R and y>=(7/5)*x-(90+d)/R) and (y<=(6/5)*x-(10+d)/R and y<=(-6/5)*x+(170+d)/R) :
            flag = False
        elif (y<=(-7/5)*x+120/R) and y<=(7/5)*x-20/R and y>=(15-d)/R:
            flag = False
        elif y>=(7/5)*x-20/R and y>=(-13)*x+(340+d)/R and y<=(-1)*x+(100+d)/R:
            flag = False
        elif (y>=0 and y<=0+d/R):
            flag = False
        elif (y<=200/R and y>=(200-d)/R):
            flag = False
        elif (x>=0 and x<=d/R):
            flag = False
        elif (x<=300/R and x>=(300-d)/R):
            flag = False
        else:
            flag = True
        return flag

    def checkVisited(self, node):
        #### node = [ cost , x , y , angle ]
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        if self.explored[checkPosX, checkPosY, checkPosA,3] != 0:
            return True ##### Yes...it is visited
        else:
            return False ##### Not visited

    def findVisited(self, node):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        # print(checkPosX, checkPosY, checkPosA)
        return self.explored[checkPosX, checkPosY, checkPosA, :]

    def addVisited(self, node, parentNode):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3]//self.thetaStep)
        self.plotData_X.append(parentNode[1])
        self.plotData_Y.append(parentNode[2])
        self.plotData_U.append(node[1] - parentNode[1]) 
        self.plotData_V.append(node[2] - parentNode[2])
        # self.explored[checkPosX, checkPosY, checkPosA, 3] = newCost
        self.explored[checkPosX, checkPosY, checkPosA, :] = np.array(parentNode)
        return

    def plotAll(self, path):
        plt.ion()
        fig, ax = plt.subplots()
        for i in range(1,len(self.plotData_X)+1):
            plt.cla()
            plt.xlim(0,30)
            plt.ylim(0,20)
            q = ax.quiver(self.plotData_X[:i], self.plotData_Y[:i], 
                self.plotData_U[:i], self.plotData_V[:i],units='xy' ,scale=1, headwidth = 0.1,headlength=0)
            plt.pause(0.0001)

        X,Y,U,V = [],[],[],[] 
        for i in range(len(path)-1):
            X.append(path[i][1])
            Y.append(path[i][2])
            U.append(path[i+1][1] - path[i][1])
            V.append(path[i+1][2] - path[i][2])
            
        for i in range(1,len(X)+1):
            plt.cla()
            plt.xlim(0,30)
            plt.ylim(0,20)
            L = ax.quiver(self.plotData_X, self.plotData_Y, 
                self.plotData_U, self.plotData_V,units='xy' ,scale=1, headwidth = 0.1,headlength=0)
            q = ax.quiver(X[:i], Y[:i], U[:i], V[:i], units='xy', scale=1, color='r', headwidth = 1, headlength=0)
            plt.pause(0.0001)

        plt.ioff()
        plt.show()



class pathFinder():
    def __init__(self, initial, goal, thetaStep = 30, stepSize = 1, goalThreshold = 1.5,
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
            costToGo = 1
            #### Action  --- [ x , y , angle , cost ] ----
            self.actionSet.append([x, y, angle, costToGo])
        pass

    def initialCheck(self):
    #writing the condition for the case when start or goal node are defined in an obstacle
        # if (self.obstacle(initial[0],initial[1])==False or (self.obstacle(goal[0],goal[1])==False)):
        if False:
            print("position not allowed")
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
            print(currentNode)
            track.append(currentNode)
        print("-------------------")
        print("Trackback")
        print(track)
        return track

    def findPath(self):
        self.initialCheck()
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
                dg = self.heuristics(newNode)
                # newNode[0] = newCost
                # print("Found a new node " + str(newNode))
                # print(newNode)
                if not self.obstacle.checkVisited(newNode):
                    ##### Node is not visited so add to data
                    presentNode[0] = newCostToCome
                    self.obstacle.addVisited(newNode, presentNode[:4])
                    newNode[0] = newCostToCome + dg
                    heappush(self.Data, newNode)
                else: #### Node is visited so check previous cost
                    previousVisited = self.obstacle.findVisited(newNode)
                    # previousNode = self.nodeData[previousVisited]
                    previousCost = previousVisited[3]
                    if previousCost > newCostToCome:
                        # self.nodeData.append(newNode)
                        presentNode[0] = newCostToCome
                        self.obstacle.addVisited(newNode, presentNode[:4])
                        # self.allData[]
        print("Could not reach goal..")
        return


    # def findPath(self):
    #     selfID = 1
    #     self.initialCheck()
    #     while len(self.Data)>0:
    #         presentNode = heappop(self.Data)
    #         # currentNode = self.nodeData[currentNodeID]
    #         if self.goalReached(presentNode):
    #             self.goalReach = True
    #             print(" Goal Reached ")
    #             # self.obstacle.addVisited()
    #             # self.obstacle.plotAll()
    #             self.trackBack(presentNode)
    #             return

    #         for action in self.actionSet:
    #             ##### node = [ x , y , angle , cost]
    #             ##### Data = [ cost , selfID , parentID ]
    #             newNodeX = presentNode[1] + action[0]
    #             newNodeY = presentNode[2] + action[1]
    #             newNodeA = action[2]
    #             newNode = [0, newNodeX, newNodeY, newNodeA]
    #             newCost = presentNode[0] + action[3] + self.heuristics(newNode)
    #             newNode[0] = newCost
    #             print("Found a new node " + str(newNode))
    #             # print(newNode)
    #             if not self.obstacle.checkVisited(newNode):
    #                 ##### Node is not visited so add to data
    #                 self.obstacle.addVisited(newNode, presentNode)
    #                 # self.nodeData.append(newNode)
    #                 heappush(self.Data, newNode)
    #                 # self.allData.append([newCost, selfID, currentNodeID])
    #                 selfID += 1
    #             else: #### Node is visited so check previous cost
    #                 previousVisited = self.obstacle.findVisited(newNode)
    #                 # previousNode = self.nodeData[previousVisited]
    #                 previousCost = previousVisited[3]
    #                 if previousCost > newCost:
    #                     # self.nodeData.append(newNode)
    #                     self.obstacle.addVisited(newNode, presentNode)
    #                     # self.allData[]
    #     print("Could not reach goal..")
    #     return



initial = [5,5,0]
goal = [20,20,0]
solver = pathFinder(initial, goal, thetaStep=30, stepSize=3)
solver.findPath()