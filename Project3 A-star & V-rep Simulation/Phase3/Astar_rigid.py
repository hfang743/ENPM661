import math as m
import heapq as hpq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches
from matplotlib.patches import Ellipse
import cv2
from ObstacleMap import ObsMap

# Generating neighbor nodes
def generateNeighborNodes(node, rpm_1, rpm_2):		   # UL = velocity left, UR = velocity right
	Xi = float(node[0])
	Yi = float(node[1])
	ThetaDeg = float(node[2])
	neighbors = []
	RPM1 = rpm_1
	RPM2 = rpm_2
	actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]] 
	for action in actions:
		X1, Y1, Theta, curve_len = calculate_coord(Xi,Yi,ThetaDeg,action[0],action[1]) # (0,0,45) hypothetical start configuration
		neighbors.append((round(X1,3), round(Y1,3), round(Theta),action[0],action[1], round(curve_len,3)))
	return neighbors

# calculating distance between 2 coordinate points 	
def calc_distance(x1, y1, x2, y2):
	dist = m.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return dist

# Calculating coordinates of new node
def calculate_coord(Xi, Yi, ThetaDeg, UL, UR):
	t = 0
	r = 0.038			# turtlebot tire radius (mm)
	L = 0.354		   # turtlebot distance between wheels	(mm)
	dt = 0.1			 # reasonable dt assigned 
	Xn=Xi
	Yn=Yi
	ThetaRad = 3.14 * ThetaDeg / 180				# Theta angle in radians
	curve_len = 0
	while t<1:
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += (r /2)* (UL + UR) * m.cos(ThetaRad) * dt
		Yn += (r /2 )* (UL + UR) * m.sin(ThetaRad) * dt
		ThetaRad += (r /L) * (UR - UL) * dt
		curve_len += calc_distance(Xs, Ys, Xn, Yn)
	ThetaDeg = 180 * (ThetaRad) / 3.14
	if ThetaDeg >= 360:
		ThetaDeg = (ThetaDeg - 360)
	if ThetaDeg <= -360:
		ThetaDeg = (ThetaDeg + 360)
	return Xn, Yn, ThetaDeg, curve_len

# Custom input function to verify inputs are integer numbers
def inputIntegerNumber(msg):
	while True:
		try:
			usrInput = float(input(msg))		
		except ValueError:
			print("Value must be a number! Please try again!")
			continue
		else:
			return usrInput 
			break 

# Function to get input start and goal node coordinates along with other parameters
def get_input_coordinates():

	inputStartFlag = True
	inputGoalFlag = True

	print("===========================================")
	print("Rigid Robot - A star Algorithm Program ")
	print("===========================================")

	print("Rigid Robot Map Obstacles Clearance (c) and Robot Radius (r)")
	c = inputIntegerNumber("Please enter obstacle clearance (c) : ")
	r = inputIntegerNumber("Please enter robot radius (r) : ")
	rpm1 = inputIntegerNumber("Please enter RPM1 value (from 5 to 15) : ")
	rpm2 = inputIntegerNumber("Please enter RPM2 value (from 15 to 30) : ")
	
	while inputStartFlag:
		print("Please enter Start-Node (x,y) Coordinates")
		start_x = inputIntegerNumber(" Enter x coordinate value : ")
		start_y = inputIntegerNumber(" Enter y coordinate value : ")
		start_theta = inputIntegerNumber("Enter start node theta angle (choices: -60 -30 0 30 60) = ")
		if start_x <map_x_min or start_x > map_x_max or start_y < map_y_min or start_y > map_y_max:
			print("Start Node input coordinates are outside of map boundaries ...")
			print("Please try again!!")
			continue
		if rigid_robot_obstacle_space(start_x, start_y, c, r):
			print("Start Node coordinates are inside the obstacles boundaries...")
			print("Please try again!!")
		else:
			start_node = (start_x,start_y,start_theta)
			inputStartFlag = False
	
	while inputGoalFlag:
		print("Please enter Goal-Node (x,y) Coordinates")
		goal_x = inputIntegerNumber(" Enter x coordinate value : ")
		goal_y = inputIntegerNumber(" Enter y coordinate value : ")
		if goal_x <map_x_min or goal_x > map_x_max or goal_y < map_y_min or goal_y > map_y_max:
			print("Goal Node input coordinates are outside of map boundaries ...")
			print("Please try again!!")
			continue
		if rigid_robot_obstacle_space(goal_x, goal_y, c, r):
			print("Goal Node coordinates are inside the obstacles boundaries...")
			print("Please try again!!")
		else:
			goal_node = (goal_x,goal_y,0)
			inputGoalFlag = False

	print("Running A star Algorithm Simulation...")
	
	return start_node, goal_node, c, r , rpm1, rpm2 

# Visited nodes discrete matrix to check for duplicates
def visited_nodes_duplicate():
	visited_node_duplicate = {}
	for x in np.arange(-50,51,1)/10:
		for y in np.arange(-50,51,1)/10:
			for theta in np.arange(0,120,10)/10:
				visited_node_duplicate[x,y,theta]=0
	return visited_node_duplicate

# Node Cost discrete matrix
def exploredNodesCost_discrete():
	explrdNdCost_discrete = {}
	for x in np.arange(-50,51,1)/10:
		for y in np.arange(-50,51,1)/10:
			for theta in np.arange(0,120,10)/10:
				explrdNdCost_discrete[x,y,theta]=0
	return explrdNdCost_discrete
	
# Check if node is duplicate 
def is_node_duplicate(curr_nd,vis_nd_dupl):
	duplicate = False
	if curr_nd in vis_nd_dupl:
		if vis_nd_dupl[curr_nd] == 1:
			duplicate = True
		else:
			vis_nd_dupl[curr_nd] = 1
	return duplicate

# Rounding nodes to one decimal point
def roundToNearestPoint1(node):
	if node[2] < 0:
		theta_disc	 = node[2] + 360
	else:
		theta_disc = node[2]
	theta_disc = int(theta_disc/30)
	rnd_node = (round(node[0],1),round(node[1],1),theta_disc)
	return rnd_node

# Calculate heuristic distance between 2 nodes
def euclidean_dist(n_node, g_node):
	distX = g_node[0] - n_node[0]
	distY = g_node[1] - n_node[1]
	euclDist = m.sqrt((distX)**2 + (distY)**2)
	return euclDist
	
# Defining Obstacle Space using Half Plane Equations while also adding obstacle clearance and robot radius
def rigid_robot_obstacle_space(x,y,clearance,radius): #clearance = 0.025,radius = 0.127
	obstacle = False
	coord_offset_x= 0
	coord_offset_y = 0
	
	offset_dist = clearance + radius
	
	if ((x - coord_offset_x)**2 + (y - coord_offset_y)**2 - (1 + offset_dist)**2) <= 0:	# Circle in center of map
		obstacle = True

	if ((x - (coord_offset_x + 2))**2 + (y - (coord_offset_y + 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on top corner of map
		obstacle = True

	if ((x - (coord_offset_x - 2))**2 + (y - (coord_offset_y - 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on left lower corner of map
		obstacle = True

	if ((x - (coord_offset_x + 2))**2 + (y - (coord_offset_y - 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on right lower corner of map
		obstacle = True
			
	if ( x + 4.75 + offset_dist >= 0) and (x + 3.25	 - offset_dist<= 0) and (y - 0.75 - offset_dist <= 0) and (y + 0.75 + offset_dist >= 0):	   #  x1,x2,y1 UP, y2 DWN  --> square on left side of map 
		obstacle = True

	if ( x - 3.25 + offset_dist >= 0) and (x - 4.75 - offset_dist <= 0) and (y - 0.75 - offset_dist <= 0) and (y + 0.75 + offset_dist >= 0):	   # square on right side of map 
		obstacle = True
			
	if (x + 2.75 + offset_dist >= 0) and (x + 1.25 - offset_dist <= 0) and (y - 2.25 + offset_dist >= 0) and (y - 3.75 - offset_dist <= 0):	   # square on top left side of map 
		obstacle = True
			
	return obstacle	

# Checking for node to see if it has valid coordinates
def isNewNodeValid(curr_nd, clearance, radius):
		valid = False
		if curr_nd[0] < -5 or curr_nd[1] < -5 :	  
			valid = True
		if curr_nd[0] > 5 or curr_nd[1] > 5 :
			valid = True
		if rigid_robot_obstacle_space(curr_nd[0],curr_nd[1],clearance,radius) == True :
			valid = True
		return valid

# Implementing A star Algorithm to obtain list of visited nodes	
def applyingAstarAlgorithm(start_node, goal_nd, goal_radius, visited_duplicate,clearance,radius, rpm1, rpm2):
	angleList = [60*rad, 30*rad, 0.0, -30*rad, -60*rad]
	visitedListChildPar = []
	visitedCurrListChildPar = []
	node_goal_thd = (0,0,0)
	exploredNodesCost = exploredNodesCost_discrete()  # Contains list of explored nodes cost
	exploredNodesCost[start_node] = 0
	visited_duplicate[start_node] = 1
	Gcost = {} 
	Fcost = {} 
	Gcost[start_node] = 0 
	Fcost[start_node] = euclidean_dist(start_node, goal_nd)
	closedList = set()
	openList = set([start_node])
	exploredNodesPath = {}
	currVisitedNds = {}
	counter = 0
	while len(openList) > 0:
		neighbors_list = []
		current = None
		currF = None
		for node in openList:
			h = euclidean_dist(node,goal_nd)
			pos_index = (node[0],node[1],node[2])
			if current	is None or Fcost[pos_index] + h*0.135 < currF:
				currF  = Fcost[pos_index]
				current = node
		rnd_curNode = roundToNearestPoint1(current)	  
		rnd_curCost =  float(exploredNodesCost[rnd_curNode])

		if counter > 0:
			currVisitedNds[current] = exploredNodesPath[current]
			ParNd = [item for item in visitedListChildPar if item[1] == current]
			ParentNode = ParNd[0][0]
			visitedCurrListChildPar.append((ParentNode,current))
		counter = counter + 1
		if euclidean_dist(current, goal_nd) <= goal_radius:
			node_goal_thd = current
			return currVisitedNds, node_goal_thd, visitedCurrListChildPar 
		openList.remove(current)
		closedList.add(current)
		neighbors_list = generateNeighborNodes(current, rpm1, rpm2)
		current_index = (current[0],current[1],current[2])
		for newNeighbor in neighbors_list:
			#if newNeighbor is not in valid coordinates
			if isNewNodeValid(newNeighbor, clearance, radius) :
				continue	# then skip new neighbor node
			if newNeighbor in closedList:
				continue
			rnd_newNode = roundToNearestPoint1(newNeighbor) ##
			newNeighborGcost = Gcost[current_index] + newNeighbor[5]
			rnd_newCost = rnd_curCost + newNeighbor[5]
			#if newNeighbor not in openList:
			if is_node_duplicate(rnd_newNode,visited_duplicate) == False:
				openList.add(newNeighbor) 
				visitedListChildPar.append((current,newNeighbor))
			elif newNeighborGcost >=exploredNodesCost[rnd_newNode]:
				continue
			exploredNodesCost[rnd_newNode] = rnd_newCost
			exploredNodesPath[newNeighbor] = current
			cost_index =(newNeighbor[0],newNeighbor[1],newNeighbor[2])
			Gcost[cost_index] = newNeighborGcost
			visitedListChildPar.append((current,newNeighbor))
			Hcost = euclidean_dist(newNeighbor, goal_nd)
			Fcost[cost_index] = Gcost[cost_index] + Hcost
			
# Implementing backtracking function to retrieve optimal path from visited nodes   
def backtrackingStartGoalPath(start,goal_thd,explored_path):
	pathlist = []
	goalpath = goal_thd
	pathlist.append(goal_thd)
	while goalpath != start:
		pathlist.append(explored_path[goalpath])
		goalpath = explored_path[goalpath]
	pathlist.reverse()
	return pathlist	
	
# Plotting curve line between 2 coordinate points
def plot_curved_line(nodePar,nodeCh,linecolor):
	t = 0
	r = 0.038			# turtlebot tire radius (mm)
	L = 0.354		   # turtlebot distance between wheels	(mm)
	dt = 0.1			 # reasonable dt assigned 
	Xn = nodePar[0]
	Yn = nodePar[1]
	ThetaRad = 3.14 * nodePar[2] / 180
	UL = nodeCh[3]
	UR = nodeCh[4]

	while t<1:
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += (r /2)* (UL + UR) * m.cos(ThetaRad) * dt
		Yn += (r /2 )* (UL + UR) * m.sin(ThetaRad) * dt
		ThetaRad += (r /L) * (UR - UL) * dt
		plt.plot([Xs, Xn], [Ys, Yn], color=linecolor,linewidth=0.6)
	ThetaDeg = 180 * (ThetaRad) / 3.14
	if ThetaDeg >= 360:
		ThetaDeg = (ThetaDeg - 360)
	if ThetaDeg <= -360:
		ThetaDeg = (ThetaDeg + 360)
	return Xn, Yn, ThetaDeg

# Buffering image 
def bufImage():
	Obs_space.fig.canvas.draw()
	mapImg = np.frombuffer(Obs_space.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(Obs_space.fig.canvas.get_width_height()[::-1] + (3,))
	mapImg = cv2.cvtColor(mapImg,cv2.COLOR_RGB2BGR)
	return mapImg

# Simulating optimal path 
def showSimulation(node_list):
	print("Displaying Simulation")
	for node in node_list:
		plot_curved_line(node[0],node[1],"orange")
		mapImg = bufImage()
		if cv2.waitKey(1) == ord('q'):	
			exit()
		cv2.imshow('A star', mapImg)

	AstarLen = len(AstarPath)-1
	i = 0
	AstarPathNode = None
	while i < AstarLen:
		AstarPathNode = (AstarPath[i],AstarPath[i+1])
		plot_curved_line(AstarPathNode[0],AstarPathNode[1],'black')
		mapImg = bufImage()		
		i = i+1
		cv2.imshow('A star', mapImg)
		if cv2.waitKey(1) == ord('q'):
			exit()
	cv2.imshow('A star', mapImg)
	
# Main routine
if __name__ == '__main__':
	map_x_min = -5
	map_y_min = -5
	map_x_max = 5
	map_y_max = 5
	rad = m.pi/180
	goal_radius = 0.3													# Goal Radius set at 3 by default
	start_node, goal_node, c, r, rpm1, rpm2 = get_input_coordinates()		# Gets user input and formats it for algorithm processing
	vis_nd_duplicate = visited_nodes_duplicate()				# Generate Discrete visited nodes map to check for duplicate nodes
	visited_nodes, goal_threshold,visCurrListChildPar = applyingAstarAlgorithm(start_node, goal_node,goal_radius, vis_nd_duplicate, c, r, rpm1, rpm2)	# Applying Djikstra Algorithm 
	AstarPath = backtrackingStartGoalPath(start_node,goal_threshold,visited_nodes)			# Extract Shortest path from visited nodes list
	Obs_space = ObsMap(c,r)										# Creating an instance of the Obstacle Space Object 
	goal_circ = plt.Circle((goal_node[0],goal_node[1]), radius=goal_radius, color='#F0DB4F')	# Drawing a goal threshold area in the map
	Obs_space.ax.add_patch(goal_circ)							# Adding goal circle drawing to map
	goal_circ = plt.Circle((start_node[0],start_node[1]), radius=0.1, color='#333399')	# Drawing a goal threshold area in the map
	Obs_space.ax.add_patch(goal_circ)							# Adding start circle drawing to map
	showSimulation(visCurrListChildPar)								# Executing A star Simulation
	if cv2.waitKey(0):
		exit()
	cv2.destroyAllWindows()