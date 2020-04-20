import numpy as np
import cv2
import numpy as np
# This class contains the obstacle space to be used by the Astar_rigid.py script

from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import math as m

class ObsMap:
	def __init__(self,robot_radius,obs_clearance):
		self.robot_radius = robot_radius
		self.obs_clearance = obs_clearance
		self.start= -5		# change to 0
		self.end = 5		# change to 10
		self.fig = plt.figure(figsize=(8,6))
		self.ax = self.fig.subplots()
		self.ax.set_xlim(self.start, self.end)
		self.ax.set_ylim(self.start, self.end)
		# self.ax.set_xticks(np.arange(self.start, self.end+0.1, 1))
		# self.ax.set_xticks(np.arange(self.start, self.end+0.1, 0.5), minor=True)
		# self.ax.set_yticks(np.arange(self.start, self.end+0.1, 1))
		# self.ax.set_yticks(np.arange(self.start, self.end+0.1, 0.5), minor=True)
		
		self.ax.grid(which='minor', alpha=0.2)
		self.ax.grid(which='major', alpha=0.5)
		self.ax.set_aspect('equal')
		
		plt.xlabel('x axis')
		plt.ylabel('y axis')
		plt.title("A STAR -RIGID ROBOT MAP")
		self.offset = (self.end + self.start) / 2

		#self.draw_obs_clearance_map()

		self.draw_obs_map()
		
	def draw_obs_map(self):
		goal_circ = Circle((self.offset,self.offset), radius=1, color='#3C873A')			# Circle in center of map
		self.ax.add_patch(goal_circ)							
		goal_circ = Circle((self.offset+2,self.offset+3), radius=1, color='#3C873A')	# Circle on top corner of map
		self.ax.add_patch(goal_circ)								   
		goal_circ = Circle((self.offset-2,self.offset-3), radius=1, color='#3C873A')	# Circle on left lower corner of map
		self.ax.add_patch(goal_circ)							 
		goal_circ = Circle((self.offset+2,self.offset-3), radius=1, color='#3C873A')	# Circle on right lower corner of map
		self.ax.add_patch(goal_circ)							
		rect = Rectangle((self.offset-4.75,self.offset-0.75),1.5,1.5,color='#3C873A') # Square on left side of map
		self.ax.add_patch(rect)
		rect = Rectangle((self.offset+3.25,self.offset-0.75),1.5,1.5,color='#3C873A') # Square on right side of map
		self.ax.add_patch(rect)
		rect = Rectangle((self.offset-2.75,self.offset+2.25),1.5,1.5,color='#3C873A') # Square on top left side of map
		self.ax.add_patch(rect)
		
	def draw_obs_clearance_map(self):
		#robot_radius = 0.25
		#clearance = 0.25
		clear_dist = self.robot_radius + self.obs_clearance
		goal_circ = Circle((self.offset,self.offset), radius=(1 + clear_dist), color='#00FFFF')			# Circle in center of map
		self.ax.add_patch(goal_circ)							
		goal_circ = Circle((self.offset+2,self.offset+3), radius=1 + clear_dist, color='#00FFFF')	# Circle on top corner of map
		self.ax.add_patch(goal_circ)								   
		goal_circ = Circle((self.offset-2,self.offset-3), radius=1 + clear_dist, color='#00FFFF')	# Circle on left lower corner of map
		self.ax.add_patch(goal_circ)							 
		goal_circ = Circle((self.offset+2,self.offset-3), radius=1 + clear_dist, color='#00FFFF')	# Circle on right lower corner of map
		self.ax.add_patch(goal_circ)							
		rect = Rectangle((self.offset-4.75 - clear_dist,self.offset-0.75 - clear_dist),1.5 + 2*clear_dist,1.5 + 2*clear_dist,color='#00FFFF') # Square on left side of map
		self.ax.add_patch(rect)
		rect = Rectangle((self.offset+3.25- clear_dist,self.offset-0.75 - clear_dist),1.5 + 2*clear_dist,1.5 + 2*clear_dist,color='#00FFFF') # Square on right side of map
		self.ax.add_patch(rect)
		rect = Rectangle((self.offset-2.75 - clear_dist,self.offset+2.25 - clear_dist),1.5 + 2*clear_dist,1.5 + 2*clear_dist,color='#00FFFF') # Square on top left side of map
		self.ax.add_patch(rect)
