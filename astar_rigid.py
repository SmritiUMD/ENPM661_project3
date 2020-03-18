
import math
import numpy as np
from queue import PriorityQueue
import cv2
import time
initial= 5,5
goal= 10,10


r = input("enter value of radius") #taking input for radius

c = input("enter value of clearance") #taking input for clearance
R = int(input("enter value of resolution")) #taking input for resolution


d = int(r+c)
solvable=True

#start_time = time.time() #taking the run time start
def checkr(R):
    if (250%R)!=0  or(150%R)!=0:
        print("Please enter an achivable resolution")
        exit()
    else:
        pass


def obstacle(x,y):
    flag=True
    if (y>=(3/5)*x+(25-d)/R) and (y>=(-3/5)*x+(295-d)/R):
        flag=False
    elif (y>=(3/5)*x+(55+d)/R) or (y>=(-3/5)*x+(325+d)/R):
        flag=True
    elif (x-math.ceil(225/R))**2+(y-math.ceil(50/R))**2<=(math.ceil((25+d)/R))**2:
        flag=False
    elif (x-(math.ceil(150/R)))**2/(math.ceil((40+d)/R))**2+(y-(math.ceil(100/R)))**2/(math.ceil((20+d)/R))**2<=1:
        flag=False
    elif (((x>=30.875/R and x<=35.875/R) and (y>=((-1.71)*x+(196.84-d)/R))) or ((x>=35.875/R and x<=100/R) and (y>=((0.53)*x+(108.45-d)/R)))):
        flag=False
    elif ((x>=30.875/R and x<=95/R) and (y>=((0.5380)*x+(118.99+d)/R))) or ((x>=95/R and x<=100/R) and (y>=((-1.71)*x+(332.45+d)/R))):
        flag=False
    elif (y>=(-7/5)*x+120/R and y>=(7/5)*x-(90+d)/R) and (y<=(6/5)*x-(10+d)/R and y<=(-6/5)*x+(170+d)/R) :
        flag=False
    elif (y<=(-7/5)*x+120/R) and y<=(7/5)*x-20/R and y>=(15-d)/R:
        flag=False
    elif y>=(7/5)*x-20/R and y>=(-13)*x+(340+d)/R and y<=(-1)*x+(100+d)/R:
        flag=False
    elif (y>=0 and y<=0+d/R):
        flag=False
    elif (y<=200/R and y>=(200-d)/R):
        flag=False
    elif (x>=0 and x<=d/R):
        flag=False
    elif (x<=300/R and x>=(300-d)/R):
        flag=False
    else:False
    return flag
 #writing the condition for the case when start or goal node are defined in an obstacle
if (obstacle(initial[0],initial[1])==False or (obstacle(goal[0],goal[1])==False)):
        print("position not allowed")
        solvable=False
else:
    pass

def heuristics(current, goal): #defining heuristic function as euclidian distance between current node and goal

    h=math.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
    
    return h


def goal_check(i,j):  # function to check if the explored point is inside threshold area around the goal or not
    if (i-goal[0])**2+(j-goal[1])**2<=(1.5)**2:
        k=2
    else:
        k=0
    return k


cost_list=[]
costh_list=[]
cost=float('Inf')
cost_list.append(cost)
cost_h=float('Inf')
costh_list.append(cost_h)

theta_s= 0 #(initial theta =0)
r=1 # step size(can be modified)

plot=np.zeros((600,300), np.uint8)  # creating a matrix to append information of visited nodes
visited = set([]) #\

class Node:
    def __init__(self, pos, cost, parent): #creating objects for position, cost and parent information
        self.pos = pos
        self.x = pos[0]
        self.y = pos[1]
        self.cost = cost
        self.parent = parent

def explore(node): #defining the function for exploring using a star
    i = node.x
    j = node.y
    valid_paths=[]
    for m in range(1,7):
        i=round(initial[0]+r*(math.cos(math.radians(m))))
        j= round(initial[1]+r*(math.sin(math.radians(m))))
        k=theta_s+m*30
        if obstacle(i,j)==True:  #checking for the obstacle space
            # print ('in 2nd if exp')
            cost_go = 1 
            cost_h=heuristics([i,j],goal)
            cost=cost_go+cost_h
            valid_paths.append([(i,j), cost,cost_h,cost_go])
        return valid_paths #returning all the valid paths that pass the conditions

distance = {}
for i in range(0, math.ceil(600/R)):
    for j in range(0, math.ceil(300/R)):
        distance[str([i, j])] = 99999999 #making the value of all the unvisited nodes as infinity
#####

q = PriorityQueue() #defining a priority queue
node_objects = {}
distance[str(initial)] = 0 
visited.add(str(initial))
node = Node(initial, 0, None)
node_objects[str(node.pos)] = node
q.put([node.cost, node.pos])  # adding the cost values in a priority queue
reached = False


####

if solvable:  #logic for astar to check if the nodes traversed is a goal else continue
    while not q.empty():
        node_temp = q.get()
        node = node_objects[str(node_temp[1])]
        if  goal_check(node_temp[1][0], node_temp[1][1] )==2:
            print("Reached")
            node_objects[str(goal)] = Node(goal, node_temp[0], node)
            reached = True
            break

        for next_node, cost, cost_h, cost_go in explore(node):

            if str(next_node) in visited: #defining all the visited nodes and adding the cost values 
                cost_temp = cost + distance[str(node.pos)]
                if cost_temp < distance[str(next_node)]:
                    distance[str(next_node)] = cost_temp
                    node_objects[str(next_node)].parent = node
            else:
                visited.add(str(next_node)) #adding the next node value to the visited node
                #img_show[next_node[1], next_node[0], :] = np.array([0,0,255])
                absolute_cost = cost + distance[str(node.pos)]
                distance[str(next_node)] = absolute_cost
                new_node = Node(next_node, absolute_cost, node_objects[str(node.pos)])
                node_objects[str(next_node)] = new_node
                q.put([absolute_cost, new_node.pos])  #using the queue to add get the least cost 
                print(visited)

    #print("--- %s seconds ---" % (time.time() - start_time)) #printing the total time taken to run the logic
              
    #cv2.imshow('img', img_show)
    #cv2.waitKey(10)
    goal_node = node_objects[str(goal)]
    parent_node = goal_node.parent  #adding the previous node traveled from the goal using backtracking to the parent node
    while parent_node:
        print(parent_node.pos, parent_node.cost)
        #img_show[parent_node.pos[1], parent_node.pos[0],:] = np.array([255,0,0]) #using cv2.imshow to print the final path using backtracking
        parent_node = parent_node.parent
    #cv2.imshow('img', img_show)
    #cv2.waitKey(0)



