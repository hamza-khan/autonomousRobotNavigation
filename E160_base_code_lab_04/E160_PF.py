import math
import random
import numpy as np
import copy
from E160_state import*
from scipy.stats import norm


class E160_PF:

	def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
		self.particles = []
		self.environment = environment
		self.numParticles = 400
		
		# maybe should just pass in a robot class?
		self.robotWidth = robotWidth
		self.radius = robotWidth/2
		self.wheel_radius = wheel_radius
		self.encoder_resolution = encoder_resolution
		self.FAR_READING = 1000
		
		# PF parameters
		self.IR_sigma = 0.2 # Range finder s.d
		#self.odom_xy_sigma = 1.25	# odometry delta_s s.d
		#self.odom_heading_sigma = 0.75	# odometry heading s.d
		#self.odom_lwheel_sigma = 
		#self.odom_rwheel_sigma = 
		self.particle_weight_sum = 0

		# define the sensor orientations
		self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
		self.walls = self.environment.walls

		# initialize the current state
		self.state = E160_state()
		self.state.set_state(0,0,0)

		# TODO: change this later
		self.map_maxX = 0.5
		self.map_minX = -0.5
		self.map_maxY = 0.5
		self.map_minY = -0.5
		self.InitializeParticles()
		self.last_encoder_measurements =[0,0]

	def InitializeParticles(self):
		''' Populate self.particles with random Particle 
			Args:
				None
			Return:
				None'''

		for i in range(0, self.numParticles):
			self.SetRandomStartPos(i)
			#self.SetKnownStartPos(i)

			
	def SetRandomStartPos(self, i):
		# add student code here 
		
		x = random.uniform(self.map_minX, self.map_maxX)
		y = random.uniform(self.map_minY, self.map_maxY)
		heading = random.random()*2*math.pi
		weight = 1.0/(self.numParticles)
		
		p = self.Particle( x, y, heading, weight)

		self.particles.append(p)
		# end student code here
		pass

	def SetKnownStartPos(self, i):
		# add student code here 
		
		weight = 1/self.numParticles
		p = self.Particle( 0, 0, 0, weight)
		self.particles.append(p)


		# end student code here
		pass
		    
	def LocalizeEstWithParticleFilter(self, encoder_measurements, sensor_readings):

		''' Localize the robot with particle filters. Call everything
			Args: 
				delta_s (float): change in distance as calculated by odometry
				delta_heading (float): change in heading as calcualted by odometry
				sensor_readings([float, float, float]): sensor readings from range fingers
			Return:
				None'''
		# add student code here


		for i in range(0, self.numParticles):
			self.Propagate(encoder_measurements,i)
			self.particles[i].weight = self.CalculateWeight(sensor_readings, self.walls, self.particles[i])
		
		#save encoder_measurements for next time
		self.last_encoder_measurements[0] = encoder_measurements[0];
		self.last_encoder_measurements[1] = encoder_measurements[1];
		
		self.Resample()
		# end student code here
		return self.GetEstimatedPos()

	# helper function added by Aman

	def Gaussian(self, mu, sigma, x):

		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		den = 1/(sigma*math.sqrt(math.pi*2.0))
		return np.exp(- ((mu - x) ** 2) / ((sigma ** 2)* 2.0)) / den


	def Propagate(self, encoder_measurements, i):
		'''Propagate all the particles from the last state with odometry readings
			Args:
				delta_s (float): distance traveled based on odometry
				delta_heading(float): change in heading based on odometry
			return:
				nothing'''
		# add student code here 

		delta_s = 0
		delta_theta = 0

		# Calculate difference in movement from last time step
		diffEncoder0 = +(encoder_measurements[0]-self.last_encoder_measurements[0]);
		diffEncoder1 = -(encoder_measurements[1]-self.last_encoder_measurements[1]);

		# At the first iteration, zero out
		#if abs(diffEncoder0)> 1000 or abs(diffEncoder1)> 1000:
		#    diffEncoder0 = 0
		#    diffEncoder1 = 0

		if diffEncoder0 == 0.0 and diffEncoder1 == 0.0:
			print "no movement"
			print "encoder_measurements[0]: %f" % (encoder_measurements[0])
			print "last_encoder_measurements[0]: %f" % (self.last_encoder_measurements[0])

		#Localization
		wheelDistanceL = (- 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder0)); # Negative since this is left wheel
		wheelSigmaL = 0.2 * wheelDistanceL
		wheelDistanceL += random.gauss(0.0, wheelSigmaL) 
		wheelDistanceR = (+ 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder1)); # Positive since this is right wheel
		wheelSigmaR = 0.2 * wheelDistanceR
		wheelDistanceR += random.gauss(0.0, wheelSigmaR) 

		# print wheelDistanceL
		# remember for next time
		#self.last_encoder_measurements[0] = encoder_measurements[0];
		#self.last_encoder_measurements[1] = encoder_measurements[1];

		# Calculate delta_S, deltta_theta
		delta_s = 0.5 * (wheelDistanceR + wheelDistanceL);
		delta_theta = 0.5 / self.radius * (wheelDistanceR - wheelDistanceL); # + random.gauss(0.0, self.odom_heading_sigma);


		x = self.particles[i].x
		y = self.particles[i].y
		theta = self.particles[i].heading
		heading = theta + delta_theta 
		heading = self.angleDiff(heading)
		x += delta_s*(math.cos(heading))
		y += delta_s*(math.sin(heading))
		weight = self.particles[i].weight
		p = self.Particle(x,y,heading,weight)
		# self.particles[i].x = x
		# self.particles[i].y = y
		# self.particles[i].heading = heading

		self.particles[i]=p

		# end student code here

	    
	def CalculateWeight(self, sensor_readings, walls, particle):
		'''Calculate the weight of a particular particle
			Args:
				particle (E160_Particle): a given particle
				sensor_readings ( [float, ...] ): readings from the IR sesnors
				walls ([ [four doubles], ...] ): positions of the walls from environment, 
							represented as 4 doubles 
			return:
				new weight of the particle (float) '''

		newWeight = 0
		# add student code here 
		# dist1 = self.FindMinWallDistance(particle, walls, self.sensor_orientation[1]) 

		# if sensor_orientation == 0:
		# 	for i in range(len(walls)):
		# 		if walls[i].slope == "vertical":
		# 			x1 = walls[i].points[0] + walls[i].radius
		# 			y1 = walls[i].points[1] - walls[i].radius
		# 			x2 = walls[i].points[4] - walls[i].radius
		# 			y2 = walls[i].points[5] + walls[i].radius
		# 		else:
		# 			x1 = walls[i].points[0] + walls[i].radius
		# 			y1 = walls[i].points[1] + walls[i].radius
		# 			x2 = walls[i].points[4] - walls[i].radius
		# 			y2 = walls[i].points[5] - walls[i].radius


		dist0 = self.FindMinWallDistance(particle, walls, self.sensor_orientation[0]) # only looking at left sensor
		dist1 = self.FindMinWallDistance(particle, walls, self.sensor_orientation[1]) # only looking at forward sensor
		dist2 = self.FindMinWallDistance(particle, walls, self.sensor_orientation[2]) # only looking at right sensor
		# prob = self.Gaussian(dist0, self.IR_sigma, sensor_readings[0]) * self.Gaussian(dist1, self.IR_sigma, sensor_readings[1]) * self.Gaussian(dist2, self.IR_sigma, sensor_readings[2])
		
		prob = 1.0
		prob *= self.Gaussian(dist0, self.IR_sigma, sensor_readings[0])
		prob *= self.Gaussian(dist1, self.IR_sigma, sensor_readings[1])
		prob *= self.Gaussian(dist2, self.IR_sigma, sensor_readings[2])

		newWeight = prob
		# end student code here
		return newWeight

	def Resample(self):
		'''Resample the particles systematically
			Args:
				None
			Return:
				None'''
		# add student code here 
		p2 = []
		index = int(random.random() * self.numParticles)
		beta = 0.0

		# list of weights
		weights = []
		for i in range(self.numParticles):
			weights.append(self.particles[i].weight)

		mw = max(weights)
		print "max weight: %f" % (mw)
		

		for i in range(self.numParticles):
			beta += random.random() * 2.0 * mw
			while beta > self.particles[index].weight:
				beta -= self.particles[index].weight
				index = (index + 1) % self.numParticles
			
			x = self.particles[index].x
			y = self.particles[index].y
			heading = self.particles[index].heading
			weight = self.particles[index].weight
			p = self.Particle(x,y,heading,weight)

			p2.append(p)
		
		self.particles = p2


		# end student code here

	def GetEstimatedPos(self):
		''' Calculate the mean of the particles and return it 
			Args:
				None
			Return:
				None'''
		# add student code here

		x_list = []
		y_list = []
		heading_list = []
		for i in range(self.numParticles):
			x_list.append(self.particles[i].x)
			y_list.append(self.particles[i].y)
			heading_list.append(self.particles[i].heading)
		x = np.mean(x_list)
		y = np.mean(y_list)
		theta = np.mean(heading_list)
		self.state.set_state(x,y,theta)
		# end student code here
		return self.state


	def FindMinWallDistance(self, particle, walls, sensorT):
		''' Given a particle position, walls, and a sensor, find 
			shortest distance to the wall
			Args:
				particle (E160_Particle): a particle 
				walls ([E160_wall, ...]): represents endpoint of the wall 
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall' (float)'''
		# add student code here 
		distances = []

		for i in range(len(walls)):
			distances.append(self.FindWallDistance(particle, walls[i], sensorT))

		min_wall_dist = min(distances)
		# end student code here
		return min_wall_dist


	def FindWallDistance(self, particle, wall, sensorT):
		''' Given a particle position, a wall, and a sensor, find distance to the wall
			Args:
				particle (E160_Particle): a particle 
				wall ([float x4]): represents endpoint of the wall 
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall (float)'''
		# add student code here 

		# wall

		sensor_heading = particle.heading + sensorT
		sensor_heading = self.angleDiff(sensor_heading)

		# if abs(sensor_heading) == 


		slope_d =  math.tan(sensor_heading)


		y_intercept_d = particle.y - (slope_d * particle.x)

		if slope_d == 0:
			slope_d = 0.00000001


		# check if wall is vertical
		if wall.slope == "vertical":
			x1 = wall.points[0] + wall.radius
			y1 = wall.points[1] - wall.radius
			x2 = wall.points[4] - wall.radius
			y2 = wall.points[5] + wall.radius
			#check if the wall is on the right of the particle
			if particle.x< x1:
				# and sensor is facing right
				if (-math.pi/2) <= sensor_heading <= (math.pi/2):
					x_int = x1
				else:
					# sensor is facing left
					x_int = -10000
			# wall is on the left
			else:
				# and sensor is facing right
				if (-math.pi/2) <= sensor_heading <= (math.pi/2):
					x_int = 10000
				else:
					#  sensor is facing left
					x_int = x1
			
			y_int = (slope_d*x_int) + y_intercept_d
			# min_y = min(y1,y2)
			# max_y = max(y1,y2)
			# if min_y <= y_int <= max_y:
			# 	x_int = x_int
			# 	y_int = y_int
			# else:
			# 	x_int = 10000
			# 	y_int = 10000
		# horizontal wall
		else:
			x1 = wall.points[0] + wall.radius
			y1 = wall.points[1] + wall.radius
			x2 = wall.points[4] - wall.radius
			y2 = wall.points[5] - wall.radius

			# check if the wall is above
			if particle.y<y2:
				#check if sensor is facing up
				if 0 <= sensor_heading <= math.pi:
					y_int = y2
				# sensor is facing down
				else:
					y_int = -10000
			# wall is down
			else:
				#check if sensor is facing up
				if 0 <= sensor_heading <= math.pi:
					y_int = 10000
				# sensor is facing down
				else:
					y_int = y2
			#y_int = y2
			x_int = (y_int - y_intercept_d)/slope_d
		
		min_x = min(x1,x2)
		max_x = max(x1,x2) 
		min_y = min(y1,y2)
		max_y = max(y1,y2) 
		
		if min_x <= x_int <= max_x and min_y <= y_int <= max_y:
			x_int = x_int
			y_int = y_int
		else:
			x_int = 1000
			y_int = 1000

		distance_to_wall = math.sqrt(((x_int-particle.x)**2) + ((y_int - particle.y) **2))



		# wall_dy = (y2-y1)
		# wall_dx = (x2-x1)

		# #print "wall is:"

		# #for i in range(4):
		# 	#print (wall.points[i])
		# #print "wall_dy: %f" % (wall_dy)
		# #print "wall_dx: %f" % (wall_dx)
		# #

		# # if wall_dx == 0:
		# # 	wall_dx = 0.00001

		# # slope_wall = wall_dy/wall_dx

		# # if slope_wall> 9999: # for verticle wall
		# # 	slope_wall = 10000

		# y_intercept_wall = y1 - (slope_wall * x1)

		# # line d

		# # point of intersection

		# slope_diff = slope_wall - slope_d

		# if slope_diff == 0:
		# 	slope_diff = 0.00001

		# x_int = (y_intercept_d - y_intercept_wall)/slope_diff;
		# y_int = (slope_wall * x_int) + y_intercept_wall

		# # check if point of intersection exsist
		# #print "x_int is: %f" % (x_int)
		# #print "y_int is: %f" % (y_int)


		# #print "x_min is: %f" % min(wall.points[0],wall.points[2])

		# #print "x_min is: %f" % min(wall.points[0],wall.points[2])
		# #print "x_max is: %f" % max(wall.points[0],wall.points[2])
		# if min(x1,x2) <= x_int <= max(x1,x2):
		# 	if min(y1,y2) <= y_int <= max(y1,y2):
		# 		x_int = x_int
		# 		y_int = y_int
		# else:
		# 	x_int = 10000
		# 	y_int = 10000

		# # length d

		

		# #print "x_int is: %f" % (x_int)
		# #print "y_int is: %f" % (y_int)
		


		# # end student code here
		#print "distance to the wall is: %f" % distance_to_wall		
		return distance_to_wall


	# def findClosestLine(point, line1, line2):
		'''find the closest line     '''

	

	def angleDiff(self, ang):
		''' Wrap angles between -pi and pi'''
		while ang < -math.pi:
			ang = ang + 2 * math.pi
		while ang > math.pi:
			ang = ang - 2 * math.pi
		return ang

	class Particle:
		def __init__(self, x, y, heading, weight):
			self.x = x
			self.y = y
			self.heading = heading
			self.weight = weight

		def __str__(self):
			return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)



