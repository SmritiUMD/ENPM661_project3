import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse

class Obstacle():
    def __init__(self, width = 1020, height = 1020, r = 1, c = 1, threshold=0.5):
        self.threshold = threshold #### Resolution
        self.W = int(width/threshold) 
        self.H = int(height/threshold) 
        self.r = r
        self.c = c
        ### all angles and cost and selfID
        ### Fourth dimention [ cost , x , y , theta ] -- of parent
        self.explored = np.zeros([self.H, self.W, 4])
        ### [ startX , startY , endX , endY ]
        self.plotData_X = []
        self.plotData_Y = []
        self.plotData_U = []
        self.plotData_V = []

    def plotSpace(self):
        centX, centY, radii = 0,0,1
        circle_1_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        circle_1_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        centX, centY, radii = 2,3,1
        circle_2_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        circle_2_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        centX, centY, radii = 2,-3,1
        circle_3_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        circle_3_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        centX, centY, radii = -2,-3,1
        circle_4_X = [centX+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        circle_4_Y = [centY+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        square_1_x, square_1_y = [-2.75, -1.25, -1.25, -2.75, -2.75],[ 3.75,  3.75,  2.25,  2.25,  3.75]
        square_2_x, square_2_y = [3.25, 4.75, 4.75, 3.25, 3.25],[ 1.5/2,  1.5/2, -1.5/2, -1.5/2,  1.5/2]
        square_3_x, square_3_y = [-3.25, -4.75, -4.75, -3.25, -3.25],[  1.5/2,   1.5/2,  -1.5/2,  -1.5/2,   1.5/2]
        plt.plot(circle_1_X, circle_1_Y)
        plt.plot(circle_2_X, circle_2_Y)
        plt.plot(circle_3_X, circle_3_Y)
        plt.plot(circle_4_X, circle_4_Y)
        plt.plot(square_1_x, square_1_y)
        plt.plot(square_2_x, square_2_y)
        plt.plot(square_3_x, square_3_y)
        plt.xlim(-10.2/2, 10.2/2)
        plt.ylim(-10.2/2, 10.2/2)
        x = []
        y = []
        for i in range()
        plt.show()

    def ObsCheck(self, i, j):
        ## i = x-direction
        ## j = y-direction
        ## Returns true if not in obstacle
        ## Returns false if in obstacle
        # Check all the obstacles
        if self.checkBoundary(i,j):
            return False
        elif self.checkInCircle(i, j, (0,0), 1):
            return False
        elif self.checkInCircle(i, j, (2,3), 1):
            return False
        elif self.checkInCircle(i, j, (2,-3), 1):
            return False
        elif self.checkInCircle(i, j, (-2,-3), 1):
            return False   
        elif self.checkInSquare(i, j, 3.25, 4.75, 0.75, -0.75):
            return False
        elif self.checkInSquare(i, j, -4.75, -3.25, 0.75, -0.75):
            return False
        elif self.checkInSquare(i, j, -2.75, -1.25, 3.75, 2.25):
            return False
        else:
            return True

    def checkInSquare(self, i, j, left, right, top, bottom):
        l = left - self.r - self.c
        r = right + self.r + self.c
        t = top + self.r + self.c
        b = bottom - self.r - self.c
        if (i<r and i>l and j<t and j>b):
            return True
        else:
            return False

    def checkInCircle(self, i, j, center, radius):
        ## i = x-direction
        ## j = y-direction
        center_x, center_y = center[0], center[1]
        if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
            return True
        else:
            return False

    def checkInQuad(self, i, j, vertex1, vertex2, vertex3, vertex4):
        x1, y1 = vertex1[0], vertex1[1]
        x2, y2 = vertex2[0], vertex2[1]
        x3, y3 = vertex3[0], vertex3[1]
        x4, y4 = vertex4[0], vertex4[1]
        if (i>x1 and i<=x2) and (j>=y1 and j<=y2):
            return True
        return False

    def checkBoundary(self,i,j):
        # print(i,j)
        if i < (1000 - self.r - self.c) and i > (self.r + self.c) and j < (1020 - self.r - self.c) and j > (self.r + self.c):
            # print("In")
            return False
        return True

    def checkVisited(self, node):
        #### node = [ cost , x , y , angle ]
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3])
        print(node[1], node[2])
        print(checkPosX, checkPosY)
        if self.explored[checkPosY, checkPosX, checkPosA,3] != 0:
            return True ##### Yes...it is visited
        else:
            return False ##### Not visited

    def discret(self,node):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3])
        return [node[0], checkPosY, checkPosX, checkPosA]

    def findVisited(self, node):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3])*dt 
        # print(checkPosX, checkPosY, checkPosA)
        return self.explored[checkPosY, checkPosX, checkPosA, :]

    def addVisited(self, node, parentNode):
        checkPosX = int(round(node[1]/self.threshold))
        checkPosY = int(round(node[2]/self.threshold))
        checkPosA = int(node[3])
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
            q = ax.quiver(X[i], Y[i], U[i], V[i], units='xy', 
                scale=1, color='r', headwidth = 0.1, 
                headlength=0, width = 0.7)
            plt.pause(0.0001)
        plt.ioff()
        plt.show()

    # def plotSpace(self):
    #     plt.ion()
    #     fig, ax = plt.subplots()
    #     for x in range(0, self.H, 10):
    #         for y in range(0, self.W, 10):
    #             print("Checking",str(x)," ",str(y))
    #             if self.ObsCheck(x,y):
    #                 print("Not in obstacle")
    #                 ax.scatter(x,y)
    #                 plt.pause(0.0001)
    #     plt.ioff()
    #     plt.show()
    #     pass

class pathFinder():
    def __init__(self, initial, goal, thetaStep = 30, stepSize = 1, goalThreshold = 2,
        width = 1020, height = 1020, threshold = 0.5, r = 1, c = 1, wheelLength=5, Ur=2,Ul=2, wheelRadius=2,
        dt=0.1, dtheta=0):
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
        self.dt=dt
        self.dtheta=dtheta
        self.wheelRadius=wheelRadius
        self.wheelLength=wheelLength
        self.Ur=Ur
        self.Ul=Ul

        self.stepSize = stepSize
        self.goalThreshold = goalThreshold
        # self.setActions()
        self.obstacle = Obstacle(width, height, r = r, c = c, threshold=threshold)


    def setActions(self):
        self.actionSet = []
        t=0
        dt=0.1

        actions=[[0,0],[0,self.Ur],[self.Ul,0],[0,self.Ul],[self.Ur,0],[self.Ul,self.Ur],[self.Ur,self.Ul],[self.Ur,self.Ur],[self.Ul,self.Ul]]
        initial[2]=3.14*initial[2]/180  
        angle=initial[2]   
        while(t<1):
            t=t+dt
            for action in actions:
                x=(self.wheelRadius)*(action[0]+action[1])*math.cos(angle)*dt       
                y=(self.wheelRadius)*(action[0]+action[1])*math.sin(angle)*dt      
                dtheta=(self.wheelRadius/self.wheelLength)*(action[0]-action[1])              
                angle=angle+dtheta
                costToCome = math.sqrt((x-initial[0])**2+(initial[1]-y)**2)
                self.actionSet.append([x, y, angle, costToCome])
                print(self.actionSet)
        pass

    def initialCheck(self):
    #writing the condition for the case when start or goal node are defined in an obstacle
        # if (self.obstacle(initial[0],initial[1])==False or (self.obstacle(goal[0],goal[1])==False)):
        if not self.obstacle.ObsCheck(self.goal[0], 200-self.goal[1]):
            print("Goal in obstacle field")
            return False
        elif not self.obstacle.ObsCheck(self.initial[0], 200-self.goal[1]):
            print("Initial position in obstacle field")
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

Parser = argparse.ArgumentParser()
Parser.add_argument('--Start', default="[50,30,60]", help='Give inital point')
Parser.add_argument('--End', default="[150,150,0]", help='Give final point')
Parser.add_argument('--RobotRadius', default=1, help='Give robot radius')
Parser.add_argument('--Clearance', default=1, help='Give robot clearance')
Parser.add_argument('--ShowAnimation', default=1, help='1 if want to show animation else 0')
Parser.add_argument('--Framerate', default=30, help='Will show next step after this many steps. Made for fast viewing')
Parser.add_argument('--thetaStep', default=30, help='Possibilities of action for angle')
Parser.add_argument('--StepSize', default=2, help='Step size')
Parser.add_argument('--Threshold', default=0.5, help='Threshold value for appriximation')
Parser.add_argument('--GoalThreshold', default=2, help='Circle radius for goal point')
Args = Parser.parse_args()

Args = Parser.parse_args()

start = Args.Start
end = Args.End
r = int(Args.RobotRadius)
c = int(Args.Clearance)
animation = int(Args.ShowAnimation)
framerate = int(Args.Framerate)
StepSize = int(Args.StepSize)
Threshold = float(Args.Threshold)
GoalThreshold = float(Args.GoalThreshold)

initial = [int(i) for i in start[1:-1].split(',')]
goal = [int(i) for i in end[1:-1].split(',')] 


solver = pathFinder(initial, goal, stepSize=StepSize,
    goalThreshold = GoalThreshold, width = 1020, height = 1020, threshold = Threshold,
    r=r, c=c)
# solver.findPath()
solver.obstacle.plotSpace()





