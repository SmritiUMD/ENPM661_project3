import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse
from Phase3 import *

### Make this as the main ros file to publish messages


def main():
	Parser = argparse.ArgumentParser()
	Parser.add_argument('--Start', default="[-4.5, 4, 120]", help='Give inital point')
	Parser.add_argument('--End', default="[0, -3, 0]", help='Give final point')
	Parser.add_argument('--RobotRadius', default=0.01, help='Give robot radius')
	Parser.add_argument('--Clearance', default=0.01, help='Give robot clearance')
	Parser.add_argument('--ShowAnimation', default=1, help='1 if want to show animation else 0')
	Parser.add_argument('--Framerate', default=30, help='Will show next step after this many steps. Made for fast viewing')
	Parser.add_argument('--thetaStep', default=30, help='Possibilities of action for angle')
	Parser.add_argument('--StepSize', default=2, help='Step size')
	Parser.add_argument('--Threshold', default=0.01, help='Threshold value for appriximation')
	Parser.add_argument('--GoalThreshold', default=0.1, help='Circle radius for goal point')
	Parser.add_argument('--WheelRadius', default=0.038, help='Radius of the robot wheel in meters')
	Parser.add_argument('--WheelLength', default=0.354, help='Distance between two wheels')
	Parser.add_argument('--LeftRPM', default=10, help='RPM of left wheel')
	Parser.add_argument('--RightRPM', default=10, help='RPM of right wheel')

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

	initial = [float(i) for i in start[1:-1].split(',')]
	goal = [float(i) for i in end[1:-1].split(',')] 

	wheelLength = float(Args.WheelLength) 
	Ur = float(Args.RightRPM)
	Ul = float(Args.LeftRPM) 
	wheelRadius = float(Args.   WheelRadius)

	solver = pathFinder(initial, goal, stepSize=StepSize,
	    goalThreshold = GoalThreshold, width = 10, height = 10, threshold = Threshold,
	    r=r, c=c, wheelLength = wheelLength, Ur = Ur, Ul = Ul, wheelRadius = wheelRadius)
	solver.findPath()
	##### you will get this parameters after solving
	##### use below data to simulate
	path = solver.path
	trackIndex = solver.trackIndex
	foundPath = solver.goalReach

main()