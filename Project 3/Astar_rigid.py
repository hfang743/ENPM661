import math as m
import heapq as hpq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches
import cv2
from ObstacleMap import ObsMap

def generateNeighborNodes(prev_theta,ang_list,step, curr_pos_x0,curr_pos_y0):
	neighbors = []
	ang_list = ang_list
	for thet in ang_list:
		rotatedX = (m.cos(thet) * m.cos(prev_theta*m.pi/180)*step - m.sin(thet) * m.sin(prev_theta*m.pi/180)*step + curr_pos_x0)
		rotatedY = (m.sin(thet) * m.cos(prev_theta*m.pi/180)*step + m.cos(thet) * m.sin(prev_theta*m.pi/180)*step + curr_pos_y0)
		newTheta = prev_theta + thet/rad
		if newTheta >= 360:
			newTheta = (newTheta - 360)
		if newTheta <= -360:
			newTheta = (newTheta + 360)
		neighbors.append((rotatedX,rotatedY,round((newTheta))))
	return neighbors

# Custom input function to verify inputs are integer numbers
def inputIntegerNumber(msg):
  while True:
    try:
       usrInput = int(input(msg))       
    except ValueError:
       print("Value must be an integer! Please try again!")
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
	stepSizeSz = inputIntegerNumber("Please enter stepSize Size (from 1 to 10) : ")
	
	while inputStartFlag:
		print("Please enter Start-Node (x,y) Coordinates")
		start_x = inputIntegerNumber(" Enter x coordinate value : ")
		start_y = inputIntegerNumber(" Enter y coordinate value : ")
		start_theta = inputIntegerNumber("Enter start node theta angle (choices: -60 -30 0 30 60) = ")
		if start_x <0 or start_x > 300 or start_y < 0 or start_y > 200:
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
		if goal_x <0 or goal_x > 300 or goal_y < 0 or goal_y > 200:
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
	
	return start_node, goal_node, c, r ,stepSizeSz

def visited_nodes_duplicate():
	visited_node_duplicate = {}
	for x in np.arange(0,300.5,0.5):
		for y in np.arange(0,200.5,0.5):
			for theta in range(-60,61,30):
				visited_node_duplicate[(x,y,theta)]=0
	return visited_node_duplicate

def exploredNodesCost_discrete():
	explrdNdCost_discrete = {}
	for x in np.arange(0,300.5,0.5):
		for y in np.arange(0,200.5,0.5):
			for theta in range(-60,61,30):
				explrdNdCost_discrete[(x,y,theta)]=0
	return explrdNdCost_discrete

def is_node_duplicate(curr_nd,vis_nd_dupl):
	duplicate = False
	if curr_nd in vis_nd_dupl:
		if vis_nd_dupl[curr_nd] == 1:
			#print("duplicate node!!",curr_nd)
			duplicate = True
		else:
			#print("new node = ",curr_nd)
			vis_nd_dupl[curr_nd] = 1
	return duplicate

def roundToNearestPoint5(node):
	rnd_node = (round(node[0]*2)/2,round(node[1]*2)/2,node[2])
	return rnd_node

def euclidean_dist(n_node, g_node):
	distX = g_node[0] - n_node[0]
	distY = g_node[1] - n_node[1]
	euclDist = m.sqrt((distX)**2 + (distY)**2)
	return euclDist

#Priority Queue List	
class PointNode:
	def __init__(self, Node_State_i=[]):
		self.Node_State_i = Node_State_i

#Adding a Node to Priority Queue
def addNewNode(point_Node, overall_cost, node_cost, new_node):
	hpq.heappush(point_Node.Node_State_i, [overall_cost, node_cost,new_node])

#Extracts a Node for Priority Queue and returns the node coordinates and cost
def getNode(point_Node):
	node_removed = hpq.heappop(point_Node.Node_State_i)
	node_cost = node_removed[1]
	node = node_removed[2]
	return node, node_cost   # total_cost,

#Get Cost from Current Node to Next Node
def getCost(current_node_cost, move_cost):
	new_node_cost = current_node_cost + move_cost
	return new_node_cost

# Defining Obstacle Space using Half Plane Equations while also adding obstacle clearance and robot radius
def rigid_robot_obstacle_space(x,y,clearance,radius):
    obstacle = False
    offset_dist = clearance + radius
    rhomboid_slope = 3/5
    rhomb_dist = offset_dist*m.sqrt(1 + rhomboid_slope**2)
    rect_slope1 = 37/65
    rect_slope2 = 9/5
    rect1_dist = offset_dist*m.sqrt(1 + rect_slope1**2)
    rect2_dist = offset_dist*m.sqrt(1 + rect_slope2**2)
    poly1_slope = 6/5
    poly2_slope = 1
    poly3_slope = 13
    poly4_slope = 0
    poly5_slope = 7/5
    poly6_slope = 6/5
    midline_slope = 7/5
    poly1_dist = offset_dist*m.sqrt(1 + poly1_slope**2)
    poly2_dist = offset_dist*m.sqrt(1 + poly2_slope**2)	
    poly3_dist = offset_dist*m.sqrt(1 + poly3_slope**2)
    poly4_dist = offset_dist*m.sqrt(1 + poly4_slope**2)
    poly5_dist = offset_dist*m.sqrt(1 + poly5_slope**2)
    poly6_dist = offset_dist*m.sqrt(1 + poly6_slope**2)
    midline_dist = offset_dist*m.sqrt(1 + midline_slope**2)
	
    if ((x - m.ceil(225))**2 + m.ceil(y - (150))**2 - m.ceil(25 + offset_dist)**2) <= 0:   # circle
        obstacle = True
    if ((x - m.ceil(150))/m.ceil(40 + offset_dist))**2 + ((y - m.ceil(100))/m.ceil(20 + offset_dist))**2 - 1 <= 0:	# ellipse
        obstacle = True
    if (5*y + 3*x - 725 + 5*rhomb_dist >= 0) and (5*y - 3*x + 475 - 5*rhomb_dist <= 0) and (5*y + 3*x - 875 - 5*rhomb_dist <= 0) and (5*y - 3*x + 625 + 5*rhomb_dist >= 0):   # rhomboid 
        obstacle = True
    if (5*y + 6*x - 1050 + 5*poly1_dist >= 0) and (5*y - 6*x - 150 + 5*poly6_dist >= 0) and (5*y + 7*x - 1450 - 5*poly5_dist <= 0) and (y - 185 - 1*poly4_dist <= 0) and (x - 50 >= 0):    # right side of polygon   
        obstacle = True
    if (y - x - 100 + 1*poly2_dist >= 0) and (5*y - 65*x + 700 - 5*poly3_dist <= 0) and (y - 185 - 1*poly4_dist <= 0) and (x - 50 <= 0):    # left side of polygon
        obstacle = True
    if (65*y + 37*x - 5465 + 65*rect1_dist >= 0) and (5*y - 9*x - 65 - 5*rect2_dist <= 0) and (65*y + 37*x - 6235 - 65*rect1_dist <= 0) and (5*y - 9*x + 705 + 5*rect2_dist >= 0):  # rectangle rotated 30 degrees
        obstacle = True
		
    return obstacle	

# Implementing A-star Algorithm
def applyingAstarAlgorithm(start_node, goal_nd, goal_radius, visited_duplicate,clearance,radius, stepSize):
	curr_theta = 0
	moveCost = stepSize
	angleList = [60*rad, 30*rad, 0.0, -30*rad, -60*rad]
	visitedListChildPar = []
	node_goal_thd = (0,0,0)
	exploredNodesPath = {}                 			  # Contains list of explored nodes
	exploredNodesCost = exploredNodesCost_discrete()  # Contains list of explored nodes cost
	exploredNodesPath[start_node] = 0
	exploredNodesCost[start_node] = 0
	visited_duplicate[start_node] = 1
	ListOfNodes = PointNode()
	addNewNode(ListOfNodes,0,0,start_node)
	while len(ListOfNodes.Node_State_i) > 0: 
		neighbors_list = []
		currNode, currNodeCost= getNode(ListOfNodes)   #  currNodeTotCost,
		rnd_curNode = roundToNearestPoint5(currNode)
		rnd_curCost = exploredNodesCost[rnd_curNode]
		if euclidean_dist(currNode, goal_node) <= goal_radius:
			node_goal_thd = currNode
			ListOfNodes.Node_State_i.clear()
			break
		curr_x = currNode[0]
		curr_y = currNode[1]
		curr_theta = currNode[2]
		neighbors_list = generateNeighborNodes(curr_theta, angleList, stepSize, curr_x, curr_y)
		for newNode in neighbors_list:
			rnd_newNode = roundToNearestPoint5(newNode)
			newNodeCost = getCost(currNodeCost,moveCost)
			rnd_newCost = rnd_curCost + moveCost
			heuristic_dist = euclidean_dist(newNode, goal_node)
			if newNode[0] < 0 or newNode[1] < 0 :     
				continue
			if newNode[0] > 300 or newNode[1] > 200 :
				continue
			if rigid_robot_obstacle_space(newNode[0],newNode[1],clearance,radius) == True :
				continue
			if is_node_duplicate(rnd_newNode,visited_duplicate) == False:
				exploredNodesCost[rnd_newNode] = rnd_newCost
				exploredNodesPath[newNode] = currNode
				visitedListChildPar.append((currNode,newNode))
				addNewNode(ListOfNodes,heuristic_dist,newNodeCost,newNode)
			elif rnd_newCost < exploredNodesCost[rnd_newNode]:
				exploredNodesCost[rnd_newNode] = rnd_newCost   
				exploredNodesPath[newNode] = currNode
				visitedListChildPar.append((currNode,newNode))
	return exploredNodesPath, node_goal_thd, visitedListChildPar

#Implementing backtracking algorithm between start node and goal node 
def backtrackingStartGoalPath(start,goal_thd,explored_path):
	pathlist = []
	goalpath = goal_thd
	pathlist.append(goal_thd)
	while goalpath != start:
		pathlist.append(explored_path[goalpath])
		goalpath = explored_path[goalpath]
	pathlist.reverse()
	return pathlist

def plot_lines(pt1,pt2,linecolor):
	x1=pt1[0]
	y1=pt1[1]
	theta = pt1[2]
	x2 = pt2[0]
	y2 = pt2[1]
	deltaX = x2 - x1
	deltaY = y2 - y1
	line = plt.Arrow(x1,y1, deltaX, deltaY, color=linecolor)
	return line

def bufImage():
	Obs_space.fig.canvas.draw()
	mapImg = np.frombuffer(Obs_space.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(Obs_space.fig.canvas.get_width_height()[::-1] + (3,))
	mapImg = cv2.cvtColor(mapImg,cv2.COLOR_RGB2BGR)
	return mapImg
	
def showSimulation(path_chPr):
	print("Displaying Simulation")
	for node in path_chPr:
		lineTr = plot_lines(node[0],node[1],'orange')
		Obs_space.ax.add_artist(lineTr)
		mapImg = bufImage()
		if cv2.waitKey(1) == ord('q'):
			exit()
		cv2.imshow('A star', mapImg)
	
	AstarLen = len(AstarPath)-1
	i = 0
	while i < AstarLen:
		lineTr = plot_lines(AstarPath[i],AstarPath[i+1],'black')
		Obs_space.ax.add_artist(lineTr)
		mapImg = bufImage()		
		i = i+1
		cv2.imshow('A star', mapImg)
		if cv2.waitKey(1) == ord('q'):
			exit()
	cv2.imshow('A star', mapImg)
	if cv2.waitKey(5000): # displays map with path for 5 seconds before closing it
		exit()


##############################################################################
#           Main Program Execution											 #
##############################################################################

if __name__ == '__main__':
	rad = m.pi/180
	goal_radius = 3							# Goal Radius set at 3 by default
	start_node, goal_node, c, r, stepSize = get_input_coordinates()		# Gets user input and formats it for algorithm processing
	vis_nd_duplicate = visited_nodes_duplicate()				# Generate Discrete visited nodes map to check for duplicate nodes
	visited_nodes, goal_threshold,childParPath = applyingAstarAlgorithm(start_node, goal_node,goal_radius, vis_nd_duplicate,c,r, stepSize)	# Applying Djikstra Algorithm 
	AstarPath = backtrackingStartGoalPath(start_node,goal_threshold,visited_nodes) 			# Extract Shortest path from visited nodes list
	Obs_space = ObsMap()										# Creating an instance of the Obstacle Space Object 
	goal_circ = plt.Circle((goal_node[0],goal_node[1]), radius=goal_radius, color='#00FFFF')	# Drawing a goal threshold area in the map
	Obs_space.ax.add_patch(goal_circ)							# Adding drawing to map
	showSimulation(childParPath)								# Executing A star Simulation
	

