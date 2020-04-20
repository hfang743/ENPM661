# Project 3 - PHASE 3

## PROJECT DESCRIPTION:
<p>The objective of this project is to implement A star algorithm to drive a robot in a given map environment considering differential drive constraints ( 8-connected action space).
<p>The implementation of the algorithm will take into account the following input parameters: 
<li>RPM1 (minimum rpm)</li>
<li>RPM2 (maximum rpm)</li>
<li>robot radius</li>
<li>obstacle clearance</li>
<li>star node coordinates and orientation angle</li>
<li>goal node coordinates</li>
<li>goal_radius (can not be changed from user console terminal, see below paragraphs to change it)</li>

## HOW TO RUN Astar_rigid.py
<p>Python version used:
  <p> Python 3.6.5
<p> Python Libraries needed :
  <p> numpy
  <p> math
  <p> opencv
  <p> matplotlib  
<p> How to run program :
  The project consists of 2 files:
  <li>Astar_rigid.py</li>
  <li>ObstacleMap.py</li>
  <p>Please make sure to have both files in the same folder before running simulation.
  <p>The main file is the Astar_rigid.py file.This file calls ObstacleMap.py since this file contains the the obstacle space.
<p>Open console terminal and run command below:
    <p>python Astar_rigid.py
  <p>Program should then start and ask the user to input start node coordinates and angle, goal node coordinates, obstacle clearance, robot radius and RPMs (rpm1 and rpm2).
<p>Program will then execute and once the data results are obtained,it will display them as matplotlib/opencv animation.
<p>Animation will last for about 12 minutes. At the end of the simulation, optimal path will be drawn from start to goal node.Once simulation is complete, to terminate program please click on simulation graph window and press escape key, this should successfully exit python program and return to console terminal prompt.Otherwise, press ctrl-c to force terminal to exit python program. 
<p>Note: Clearance used for video output of program is 0.2 for robot radius and 0.2 for obstacle clearance, but these can be changed from user terminal.

<p>IMPORTANT : The simulation is mostly console driven and the only parameter that can not be changed from console is the goal threshold radius. This parameter can be changed in line 347 in the Astar_rigid.py file.
  
## Simulation part 1 ( User inputs to program )
<img width="750" height="450" src="https://github.com/gato78/Class-Projects/blob/master/Project3phase3/user_input_console.JPG " width="640 "/>


## Simulation part 2 ( Astar Algorithm visiting nodes in the map space )
<img width="750" height="450" src="https://github.com/gato78/Class-Projects/blob/master/Project3phase3/optimal_path_plot.JPG " width="640 "/>


## Simulation part 3 ( Video showing simulation )
<img width="750" height="450" src="https://github.com/gato78/Class-Projects/blob/master/Project3phase3/prj3-phase3-video.gif " width="640" height="480" width="640 "/>


## Where to change goal_radius parameter (Astar_rigid.py):
<img width="750" height="450" src="https://github.com/gato78/Class-Projects/blob/master/Project3phase3/goal_radius_parameter.JPG " width="640 "/>

## Astar_rigid.py EXECUTION TIME :
<p>Program will take approximately 10 to 20 seconds to compute optimal path and simulation display will vary based on user input parameters between 12 minutes to 45 minutes or so depending on how small are the rpms provided by the user. For example for RPM1 = 12 RPM2 = 24 it should take about 12 minutes to draw visited nodes path and optimal path.


## Function Descriptions:
<li>function generateNeighborNodes: Function that generates 8 neighbors from current node</li>

<li>function calc_distance: function that calculate distance between 2 coordinate points</li>

<li>function calculate_coord: calculate coordinates of a node</li>

<li>function inputIntegerNumber: utility function to check for integer number</li>

<li>function get_input_coordinates: obtains input start and goal nodes from user along other parameters needed for the simulation</li>

<li>function visited_nodes_duplicate: discrete matrix created to test for nodes duplicates</li>

<li>function exploredNodesCost_discrete: discrete matrix created to store cost of discrete version of nodes</li>

<li>function is_node_duplicate: function to test if nodes are duplicated in visited nodes</li>

<li>function roundToNearestPoint1: function that rounds to either whole number or 0.1 increments</li>

<li>function euclidean_dist: measures distance between current node and goal node</li>

<li>function rigid_robot_obstacle_space: checks if node is in obstacle space</li>

<li>function isNewNodeValid: function that checks if node has valid coordinates (not outside map or inside obstacle space).</li>

<li>function applyingAstarAlgorithm: Algorithm to obtain optimal path between start node and goal node</li>

<li>function backtrackingStartGoalPath: obtains optimal path from visited nodes provided by A start algorithm</li>

<li>function plot_curved_line: creates curved line between 2 nodes</li>

<li>function bufImage: buffers image of simulation</li>

<li>function showSimulation: executes simulation of A star algorithm</li>





