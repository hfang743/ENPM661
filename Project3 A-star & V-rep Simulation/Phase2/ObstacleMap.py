import numpy as np
import cv2
import numpy as np
# This class contains the obstacle space to be used by the Astar_rigid.py script

from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import math as m

class ObsMap:
	def __init__(self):
		self.fig = plt.figure(figsize=(10,6))
		self.ax = self.fig.subplots()
		self.ax.set_xlim(0, 300)
		self.ax.set_ylim(0, 200)
		self.ax.set_xticks(np.arange(0, 301, 50))
		self.ax.set_xticks(np.arange(0, 301, 10), minor=True)
		self.ax.set_yticks(np.arange(0, 201, 50))
		self.ax.set_yticks(np.arange(0, 201, 10), minor=True)
		
		self.ax.grid(which='minor', alpha=0.2)
		self.ax.grid(which='major', alpha=0.5)
		self.ax.set_aspect('equal')
		
		plt.xlabel('x axis')
		plt.ylabel('y axis')
		plt.title("A STAR -RIGID ROBOT MAP")

		self.draw_obs_map()

		#Draws obstacles in map
		for obs in self.obs_map:
			rect = plt.Rectangle((obs[0],obs[1]),1,1,color='b', fill=True, linewidth=None)
			self.ax.add_patch(rect)		
	
	# Generates a list containing obstacle space cells	
	def draw_obs_map(self):
		map = []
		for x in range(0,299):
			for y in range(0,199):
				if self.robot_obstacle_space(x,y)  == True:
					map.append((x,y))
		self.obs_map = map
	
	# Function to test for obstacle space cells using half plane equations
	def robot_obstacle_space(self,x,y):
		obstacle = False
		if (m.ceil(x-(225))**2 + m.ceil(y-(150))**2 - m.ceil(25)**2)<=0:   #circle
			obstacle=True
		if ((x-m.ceil(150))/m.ceil(40))**2 + ((y - m.ceil(100))/m.ceil(20))**2 - 1 <=0:	#ellipse
			obstacle=True
		if (5*y  + 3*x  -  725  >=  0) and (5*y  - 3*x  +  475  <=   0) and (5*y  + 3*x  -  875   <=  0) and (5*y  - 3*x  +  625   >=  0):   # rhomboid 
			obstacle=True
		if (65*y  + 37*x - 5465 >= 0) and (5*y - 9*x - 65 <= 0) and (65*y + 37*x - 6235 <= 0) and (5*y - 9*x + 705 >= 0):  # rectangle rotated 30 degrees
			obstacle = True
		# 6-Side Polygon has been split into 2 parts: right side and left side 
		if (5*y  + 6*x  -  1050  >=  0) and (5*y  - 6*x  -  150  >=  0) and (5*y  + 7*x  -  1450  <=  0) and (5*y  - 7*x  -  400  <=  0):    # right side of polygon
			obstacle=True
		if (y  - x  -  100   >=   0) and (5*y  -  65*x  +  700  <=  0) and (y  -  185  <=  0) and (5*y  - 7*x  -  400  >=  0):    # left side of polygon
			obstacle=True
		return obstacle
