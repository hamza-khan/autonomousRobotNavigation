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
		self.map_maxX = 1.0
		self.map_minX = -1.0
		self.map_maxY = 1.0
		self.map_minY = -1.0
		self.InitializeParticles()
		self.last_encoder_measurements =[0,0]

	def InitializeParticles(self):
		''' Populate self.particles with random Particle 
			Args:
				None
			Return:
				None'''
		self.particles = []
		for i in range(0, self.numParticles):
			self.SetRandomStartPos(i)
			#self.SetKnownStartPos(i)

			
	def SetRandomStartPos(self, i):
		# add student code here 
        
        p = Particle()
        x = random.uniform(self.map_minX, self.map_maxX)
        y = random.uniform(self.map_minY, self.map_maxY)
        heading = random.random()*2*math.pi
        weight = 1/self.numParticles
        
        
        self.particles[].append(p( x, y, heading, weight))
        # end student code here
        pass

	def SetKnownStartPos(self, i):
		# add student code here 
        p = Particle()
        weight = 1/self.numParticles
        self.particles[].append(p( 0, 0, 0, weight))

        
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
        	self.particles[i].weight = CalculateWeight
        
        self.Resample()
        
        # end student code here
        
        
		return self.GetEstimatedPos()

	# helper functions added by Aman
	def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
   

   	# End of helper function added by Aman


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
        if abs(diffEncoder0)> 1000 or abs(diffEncoder1)> 1000:
            diffEncoder0 = 0
            diffEncoder1 = 0

        #Localization
        wheelDistanceL = - 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder0); # Negative since this is left wheel
        wheelSigmaL = 0.2 * wheelDistanceL
        wheelDistanceL += random.gauss(0.0, wheelSigmaL) 
        wheelDistanceR = (+ 2 * 3.14 * self.wheel_radius / self.encoder_resolution * (diffEncoder1)); # Positive since this is right wheel
        wheelSigmaR = 0.2 * wheelDistanceR
        wheelDistanceR += random.gauss(0.0, wheelSigmaR) 

        # print wheelDistanceL
        # remember for next time
        self.last_encoder_measurements[0] = encoder_measurements[0];
        self.last_encoder_measurements[1] = encoder_measurements[1];

        # Calculate delta_S, deltta_theta
        delta_s = 0.5 * (wheelDistanceR + wheelDistanceL);
        delta_theta = 0.5 / self.radius * (wheelDistanceR - wheelDistanceL); # + random.gauss(0.0, self.odom_heading_sigma);


        x = self.particles[i].x
        y = self.particles[i].y
        theta = self.particles[i].heading
        x += delta_s*(math.cos(theta+delta_theta))
        y += delta_s*(math.sin(theta+delta_theta))
        heading = theta + delta_theta 
        heading %= 2+math.pi

        self.particles[i].x = x
        self.particles[i].y = y
        self.particles[i].heading = heading
        
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
		dist0 = FindMinWallDistance(particle, walls, sensor_readings[0]) # only looking at left sensor
        dist1 = FindMinWallDistance(particle, walls, sensor_readings[1]) # only looking at forward sensor
        dist2 = FindMinWallDistance(particle, walls, sensor_readings[2]) # only looking at right sensor
        prob = 1.0;
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
    	mw = max(self.particles[].weight)
	    for i in range(self.numParticles):
	        beta += random.random() * 2.0 * mw
	        while beta > self.particles[index].weight:
	            beta -= self.particles[index].weight
	            index = (index + 1) % self.numParticles
	        p2.append(self.particles[index])
	    self.particles[] = p2
        
        
        # end student code here
        



	def GetEstimatedPos(self):
		''' Calculate the mean of the particles and return it 
			Args:
				None
			Return:
				None'''
        # add student code here
        
	        
        
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
        


        
        
        # end student code here
        
		return 0
    

	def FindWallDistance(self, particle, wall, sensorT):
		''' Given a particle position, a wall, and a sensor, find distance to the wall
			Args:
				particle (E160_Particle): a particle 
				wall ([float x4]): represents endpoint of the wall 
				sensorT: orientation of the sensor on the robot
			Return:
				distance to the closest wall (float)'''
		# add student code here 
        slope_wall = (wall[3]-wall[1])/(wall[2]-wall[1])
        if slope_wall> 9999: # for verticle wall
        	slope_wall = 10000

    	y_intercept_wall = wall[1] - (slope_of_wall * wall[0])

    	# line d


    	slope_d =  atan2(particle.heading + sensorT)

    	y_intercept_d = particle.y - (slope_d * particle.x)

    	# slope 





        
        
        # end student code here
        		
		return 0

	

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



