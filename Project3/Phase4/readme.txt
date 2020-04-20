##################  README.TXT ################

              PROJECT 3 - PHASE 4

###############################################


Project Description:

====================

The objective of this project is to implement A star algorithm to drive the trutlebot in a vrep map environment considering differential drive constraints ( 8-connected action space).

The implementation of the algorithm will take into account the following input parameters: 

RPM1 (minimum rpm)

RPM2 (maximum rpm)

robot radius

obstacle clearance

star node coordinates and orientation angle

goal node coordinates


goal_radius (can not be changed from user console terminal, change goal_radius in line 387 in main.py)


Libraries used:
===============
Python version used 3.6.5
vrep version 4 ( CoppelliaSim player)
 

Python libraries needed:

numpy

math

opencv

matplotlib


PROJECT GITHUB LINK:

====================

https://github.com/gato78/Class-Projects/tree/master/Project3phase4

To run the simulation, please read the following steps:

## 1. Make sure you have following files in your source code directory, in order to connect to v-rep remote API.
1. vrep.py
2. vrepConst.py
3. The appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
These structural files are downloaded from CoppeliaRobotics: https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm

## 2. In v-rep, load the scene and robot. 
1. Open "new_map.ttt" file with V-REP. 
2. The turtlebot2 model script has been simplified and saved as 'simplified model.ttt' and should be already loaded in the map file.

## 3. The robot should be manually placed at the expected starting position before starting the simulation.
Select the turtlebot and click 'Object/item shift' button to adjust the X, Y coordinates in 'position' tab with a initial orientation as 0 radians.
The origin is present at the center of the workspace.

## 4. The program is supposed to be executed with python. Start the simulation before executing the program.
When running the main.py and enter the start and goal points, the program will first begin searching and generating the animation in a matplot window. 
After the robot reaches the goal point with the optimal path showing in that window, the turtlebot in v-rep will automatically start moving and follow the exact same path as in program to reach the goal.

##5. The 'Simulation results' folder contains two videos:
Scenario 1: start(-4,-3) - goal(0,-3)
Scenario 2: start(-4,-4.5) - goal(4.5,2.5)

IMPORTANT: Before executing python code please make sure you change the start position for the vrep simulation as stated in step 3.
Then enter the same starting position in the python user input console. Otherwise turtlebot will still follow pattern from path planning but will start from wrong position and will not get to desired location.

VIDEO SCENARIO 1 OUTPUT:

========================


Video output shows the following user input parameters:

robot radius = 0.2 ( using conservative radius a little larger than turtlebot but can be reduced if needed)

obstacle clearance = 0.2 ( can also be changed if needed)

RPM1 = 5

RPM2 = 10
start node x = -4

start node y = -4

start angle = 0

goal node x = 0
goal node y = -3



VIDEO SCENARIO 2 OUTPUT:

========================


Video output shows the following user input parameters:

robot radius = 0.2 ( using conservative radius a little larger than turtlebot but can be reduced if needed)

obstacle clearance = 0.2 ( can also be changed if needed)

RPM1 = 5

RPM2 = 10
start node x = -4

start node y = -4.5
start angle = 0
goal node x = 4.5
goal node y = 2.5

