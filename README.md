# ENPM661_project3
Implementation of A* algorithm for a rigid robot

Code in this repository runs A* algorithm to find the path. 

### Dependencies

- python3
- matplotlib
- numpy

You can install these dependencies using python pip package installer.
```
sudo pip3 install matplotlib numpy
```

### Map

Map for this problem statement is shown below.

![Map](https://github.com/SmritiUMD/ENPM661_project3/map.png)

### How to run

There are several arguments you can use to run this code. To see all the possible arguments typr this in command line. Please make sure you are in correct directory.

```
git clone https://github.com/SmritiUMD/ENPM661_project3
cd ENPM661_project3
python3 astar_rigid.py --help
```

These are all the arguments
```
optional arguments:
  -h, --help            show this help message and exit
  --Start START         Give inital point
  --End END             Give final point
  --RobotRadius ROBOTRADIUS
                        Give robot radius
  --Clearance CLEARANCE
                        Give robot clearance
  --ShowAnimation SHOWANIMATION
                        1 if want to show animation else 0
  --Framerate FRAMERATE
                        Will show next step after this many steps. Made for
                        fast viewing
  --thetaStep THETASTEP
                        Possibilities of action for angle
  --StepSize STEPSIZE   Step size
  --Threshold THRESHOLD
                        Threshold value for appriximation
  --GoalThreshold GOALTHRESHOLD
                        Circle radius for goal point

```
To find the path between start point, which is at [50,30,60] and end point, which is at [150,150,0], you can use following command,

```
python3 astar_rigid.py --Start='[50,30,60]' --End='[150,150,0]'
```
To run this program with different angle steps you can use `--thetaStep` argument. To run solver with different step size use the following command.

```
python3 astar_rigid.py --Start='[50,30,60]' --End='[150,150,0]' --StepSize=2 --thetaStep=30
```