##################  README.TXT ################

              PROJECT 3 - PHASE 3

###############################################


Project Description:

====================

The objective of this project is to implement A star algorithm to drive a robot in a given map environment considering differential drive constraints ( 8-connected action space).

The implementation of the algorithm will take into account the following input parameters: 

RPM1 (minimum rpm)

RPM2 (maximum rpm)

robot radius

obstacle clearance

star node coordinates and orientation angle

goal node coordinates


goal_radius (can not be changed from user console terminal, see below paragraphs to change it)

How to Run Astar_rigid.py:

===================
======
Python version used 3.6.5 

Python libraries needed:

numpy

math

opencv

matplotlib



How to run program:
===================
The project consists of 2 files:

Astar_rigid.py

ObstacleMap.py



Please make sure to have both files in the same folder before running simulation.

The main file is the Astar_rigid.py file. 
This file calls ObstacleMap.py since this file contains the the obstacle space.



Open console terminal and run command below:

python Astar_rigid.py


Program should then start and ask the user to input start node coordinates and angle, goal node coordinates, obstacle clearance, robot radius and RPMs (rpm1 and rpm2).


Program will then execute and once the data results are obtained,it will display them as matplotlib/opencv animation.
Animation will last for about 12 minutes. At the end of the simulation, optimal path will be drawn from start to goal node.
Once simulation is complete, to terminate program please click on simulation graph window and press escape key, this should successfully exit python program and return to console terminal prompt.
Otherwise, press ctrl-c to force terminal to exit python program.  

Note: Clearance used for video output of program is 0.2 for robot radius and 0.2 for obstacle clearance, but these can be changed from user terminal.



IMPORTANT:The simulation is mostly console driven and the only parameter that can not be changed from console is the goal threshold radius. 
This parameter can be changed in line 347 in the Astar_rigid.py file.

Execution Time:
===============

Program will take approximately 10 to 20 seconds to compute optimal path and simulation display will vary based on user input parameters between 12 minutes to 45 minutes or so depending on how small are the rpms provided by the user. 
For example for RPM1 = 12 RPM2 = 24 it should take about 12 minutes to draw visited nodes path and optimal path.



PROJECT GITHUB LINK:

====================


https://github.com/gato78/Class-Projects/tree/master/Project3phase3



VIDEO OUTPUT:

=============


Video output shows the following user input parameters:

robot radius = 0.2 ( using conservative radius a little larger than turtlebot but can be reduced if needed)

obstacle clearance = 0.2 ( can also be changed if needed)

RPM1 = 12

RPM2 = 24

start node x = -4

start node y = -4

start angle = 60

goal node x = 4

goal node y = 4



Note:  goal_radius variable is set at 0.3 and can bed changed in Astar_rigid.py line347



Function descriptions:

======================

function generateNeighborNodes: Function that generates 8 neighbors from current node

function calc_distance: function that calculate distance between 2 coordinate points

function calculate_coord: calculate coordinates of a node

function inputIntegerNumber: utility function to check for integer number

function get_input_coordinates: obtains input start and goal nodes from user along other parameters needed for the simulation

function visited_nodes_duplicate: discrete matrix created to test for nodes duplicates

function exploredNodesCost_discrete: discrete matrix created to store cost of discrete version of nodes

function is_node_duplicate: function to test if nodes are duplicated in visited nodes

function roundToNearestPoint1: function that rounds to either whole number or 0.1 increments

function euclidean_dist: measures distance between current node and goal node

function rigid_robot_obstacle_space: checks if node is in obstacle space

function isNewNodeValid: function that checks if node has valid coordinates (not outside map or inside obstacle space).

function applyingAstarAlgorithm: Algorithm to obtain optimal path between start node and goal node

function backtrackingStartGoalPath: obtains optimal path from visited nodes provided by A start algorithm

function plot_curved_line: creates curved line between 2 nodes

function bufImage: buffers image of simulation

function showSimulation: executes simulation of A star algorithm



