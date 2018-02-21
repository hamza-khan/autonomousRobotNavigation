
from E160_state import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)
        #self.v = 0.05
        #self.w = 0.1
        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.034
        self.botDiameter = 0.15
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        #calibration parameters for different tests - forward, backward, rotationCW and rotationCCW
        self.calibrationTest = 'forward'
        self.calibrationRotationNumber = 1
        self.calibrationCycleCount = 0
        self.calibrationDistance = 1.0
        self.calibrationSpeed = 20.0
        #self.firstTime = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        #self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + str(self.calibrationTest) + '_speed' + str(self.calibrationSpeed) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        self.encoder_resolution = 1440
        
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        

        
        
    def update(self, deltaT):
        
        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # localize
        self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)
        
        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
    
    
    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]
            
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            range_measurements = [0,0,0]
        
        return encoder_measurements, range_measurements

        
        
    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
        state_est = self.update_state(state_est, delta_s, delta_theta)
    
        return state_est
    
    
    def update_control(self, range_measurements):
        
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor
        
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":        
            R = 0
            L = 0
        	#Code for Calibration curves as a function of time
        	# if self.firstTime == 0:
		       #  self.state_est.set_state(0,0,0)
		       #  self.firstTime +=1
        	#Rotation
        	#for clockwise - cw = 1 & ccw = -1 and for counter clockwise cw = -1 and ccw = 1
            # if self.calibrationTest == 'rotationCW':        		
            #     rDirection = -1
            #     lDirection = 1 
            # elif self.calibrationTest == 'rotationCCW':        		
            #     rDirection = -1
            #     lDirection = 1 
            # elif self.calibrationTest == 'forward':
            #     rDirection = 1
            #     lDirection = 1
            # elif self.calibrationTest == 'backward':
            #     rDirection = -1
            #     lDirection = -1
            # else :
            #     rDirection = 0
            #     lDirection = 0 
		
    #rotation  		
	  #       if abs(self.state_est.theta) > (2*math.pi*self.calibrationRotationNumber):
	  # #      	print "Number of rotations left {0} and theta {1}".format(self.calibrationCycleCount, self.state_est.theta)
	  #       	self.calibrationCycleCount = self.calibrationRotationNumber
   #      		speed = 0 
   #      	else :
   #      		speed = self.calibrationSpeed
    #Translation
            # if abs(self.state_est.x) < self.calibrationDistance:
            #     speed = self.calibrationSpeed
            # else:
            #     speed = 0

            # R = int(rDirection*speed*256/100)
            # L = int(lDirection*speed*256/100)

    
        	# # code for stopping 30cm before the wall
         #    range_des = 500 # IR sensor value corresponding to 30cm
         #    range_diff = 500 - range_measurements[2]
         #    K_p = 0.5
         #    R = range_diff * K_p
         #    L = range_diff * K_p

        return R, L
            
    def send_control(self, R, L, deltaT):
        
        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
    
    
    def simulate_encoders(self, R, L, deltaT):
        gain = 10
        right_encoder_measurement = -int(R*gain*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*gain*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]
    
        
    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [1,2,3,4,5]]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)                                                         
   


    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************
        # define differential encoder measurements
        diffEncoder0 = -1*(self.encoder_measurements[0]-self.last_encoder_measurements[0])
        diffEncoder1 = -1*(self.encoder_measurements[1]-self.last_encoder_measurements[1])
        if diffEncoder0>1000:
            diffEncoder0 = 0
        if diffEncoder1>1000:
            diffEncoder1 = 0
        # reset last encoder
    #    print "Prev_state {0}, Current_State {1} and difference {2}, {3}".format(self.last_encoder_measurements,self.encoder_measurements,diffEncoder0,diffEncoder1)

 
        self.last_encoder_measurements = self.encoder_measurements
  
        #calculate distance traveled
        wheelDistanceR = (float(diffEncoder1) / float(self.encoder_resolution)) * 2*math.pi*self.wheel_radius 
        wheelDistanceL = (float(diffEncoder0) / float(self.encoder_resolution)) * 2*math.pi*self.wheel_radius
       
        #print "left: " + wheelDistanceL + "  right:" + wheelDistanceR
        #print "Left {0} and Right {1} ".format(wheelDistanceL,wheelDistanceR)

        #calculate delta_s and delta_theta
        delta_s = ( wheelDistanceL + wheelDistanceR )/ 2
        delta_theta = ( wheelDistanceR - wheelDistanceL ) / self.botDiameter 

        # ****************** Additional Student Code: End ************
            
        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta 

    
    
    
    def update_state(self, state, delta_s, delta_theta):
        
        # ****************** Additional Student Code: Start ************
        delta_x = delta_s*math.cos(state.theta + (delta_theta/2) ) 
        delta_y = delta_s*math.sin(state.theta + (delta_theta/2) )
        
        state.x += delta_x  
        state.y += delta_y
        state.theta += delta_theta
        # if state.theta > math.pi:
        #     state.theta = state.theta - 2*math.pi
        # if state.theta < -math.pi:
        #     state.theta = state.theta + 2*math.pi

        #print "X {0} and Y {1} Theta {2}".format(state.x,state.y,state.theta)
        
        # ****************** Additional Student Code: End ************
            
        # keep this to return the updated state
        return state
        
        
        
        
        
        
        