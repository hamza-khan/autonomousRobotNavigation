
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
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_PathTracking_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        self.encoder_resolution = 1440
        
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        
        self.Krho = 1.0 #1.0
        self.Kalpha = 2.0 #2.0
        self.Kbeta = -0.5 #-0.5
        self.KalphaTheta = 2.0 #2.0
        self.KbetaTheta = 1.5 #-0.5
        self.max_velocity = 0.05
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        self.stepCount = 0
        
        
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
    
    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
            
        return a
        
        
    def update_control(self, range_measurements):
        
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":   
            desiredWheelSpeedR, desiredWheelSpeedL = self.straight_line_test()
            
        return desiredWheelSpeedR, desiredWheelSpeedL
  

    def straight_line_path(self, startState, endState):
        minStateInterval = 0.05
        totalDeltaX = endState.x - startState.x
        totalDeltaY = endState.y - startState.y
        if abs(totalDeltaY)>abs(totalDeltaX):
            totalTimeSteps = math.floor(totalDeltaX/minStateInterval) + 1
        else:
            totalTimeSteps = math.floor(totalDeltaY/minStateInterval) + 1



    def straight_line_test(self):

        firstState = E160_state()
        firstState.set_state(0.0,0.0,0.0)
        secondState = E160_state()
        secondState.set_state(0.0,0.0,0.785)
        thirdState = E160_state()
        thirdState.set_state(0.4,0.4,0.785)
        fourthState = E160_state()
        fourthState.set_state(0.6,-0.6,-0.785)
        fifthState = E160_state()
        fifthState.set_state(0.0,0.0,0.785)
        lastState = E160_state()
        lastState.set_state(0.0,0.0,0.0)

        if self.stepCount == 0:
            print "second state"
            self.state_des = secondState
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            deltaX = self.state_des.x - self.state_est.x
            deltaY = self.state_des.y - self.state_est.y        
            deltaStartX = secondState.x - firstState.x
            deltaStartY = secondState.y - firstState.y
            if (deltaX<=deltaStartX/2) & (deltaY<=deltaStartY/2):
                self.stepCount += 1

        elif self.stepCount == 1:
            print "third state"
            self.state_des = thirdState
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            deltaX = self.state_des.x - self.state_est.x
            deltaY = self.state_des.y - self.state_est.y        
            deltaStartX = thirdState.x - secondState.x
            deltaStartY = thirdState.y - secondState.y
            if (deltaX<=deltaStartX/2) & (deltaY<=deltaStartY/2):
                self.stepCount += 1
 
        elif self.stepCount == 2:
            print "fourth state"
            self.state_des = fourthState
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            deltaX = self.state_des.x - self.state_est.x
            deltaY = self.state_des.y - self.state_est.y        
            deltaStartX = fourthState.x - thirdState.x
            deltaStartY = fourthState.y - thirdState.y
            if (abs(deltaX)<=abs(deltaStartX/2)) & (abs(deltaY)<=abs(deltaStartY/2)):
                self.stepCount += 1
 
        
        else:
            print "last state"
            self.state_des = lastState
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()


        return desiredWheelSpeedR,desiredWheelSpeedL







    def point_tracker_control(self):

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:
            ############ Student code goes here ############################################
            #angle wrap desired theta
            self.state_des.theta = self.angle_wrap(self.state_des.theta)

            #get delta values as desired state - current state estimate
            delta_x = self.state_des.x - self.state_est.x
            delta_y = self.state_des.y - self.state_est.y
            delta_theta = self.state_des.theta - self.state_est.theta

            #get the angle between the current state point and the desired state point to 
            #determine which way the robot is facing
            thetaEstimateToDesired = math.atan2(delta_y, delta_x) 
            thetaEstimate = self.state_est.theta
           
            #set alpha
            alpha = self.angle_wrap(-thetaEstimate + math.atan2(delta_y, delta_x))
            
            #if in front of robot
            if abs(alpha) < math.pi/2:
    #            print "im going forward"
                #constants for forward movement
                rho = math.sqrt(math.pow(delta_x, 2.0) + math.pow(delta_y, 2.0))
                alpha = self.angle_wrap(-thetaEstimate + math.atan2(delta_y, delta_x))
                beta = self.angle_wrap(-thetaEstimate - alpha + self.state_des.theta)
                # use constants to get forward and rotational velocity
                desiredV = self.Krho*rho
                desiredW = self.Kalpha*alpha + self.Kbeta*beta
            
            #if behind robot
            if abs(alpha) >= math.pi/2:
     #           print "im going backwards"
                #constants for backwards movement
                rho = math.sqrt(math.pow(delta_x, 2.0) + math.pow(delta_y, 2.0))
                alpha = self.angle_wrap(-thetaEstimate + math.atan2(-delta_y, -delta_x))
                beta = self.angle_wrap(-thetaEstimate - alpha + self.state_des.theta)
               
                # use constants to get forward and rotational velocity
                desiredV = -self.Krho*rho
                desiredW = self.Kalpha*alpha + self.Kbeta*beta   

            #second controller
            xThreshold = 0.05
            yThreshold = 0.05
            thetaThreshold = 0.1
            if (abs(delta_x) < xThreshold) & (abs(delta_y) < yThreshold):
                desiredV = 0
                desiredW = self.KalphaTheta*alpha + self.KbetaTheta*beta  

            #checks if AVA is close enough
            if (abs(delta_x) < xThreshold) & (abs(delta_y) < yThreshold) & (abs(delta_theta) < thetaThreshold):
                self.point_tracked = True

      
            #set desired rotational rate and desired wheel speed 
            L = self.botDiameter / 2
            scaleFactor = 10
            desiredRotRateR = (desiredW + ((desiredV)/L)) /2 
            desiredRotRateL = (desiredW - ((desiredV)/L)) /2 
            desiredWheelSpeedR = scaleFactor* (desiredRotRateR * 2 * L) / self.wheel_radius
            desiredWheelSpeedL = scaleFactor* (-desiredRotRateL * 2 * L) / self.wheel_radius
            #print desiredWheelSpeedL

            #Find m/s bot speed
            #botSpeed = (desiredWheelSpeedL + desiredWheelSpeedR)/2
            
            botSpeedMSRight = ( desiredWheelSpeedR * (self.wheel_radius) / self.encoder_per_sec_to_rad_per_sec ) #/  self.encoder_resolution
            botSpeedMSLeft  = ( desiredWheelSpeedL * (self.wheel_radius) / self.encoder_per_sec_to_rad_per_sec ) #/  self.encoder_resolution
            botSpeedMS = (botSpeedMSLeft + botSpeedMSRight)/2
      #      print "botspeedms {0}".format(botSpeedMS)
 
            #Check max speed
            if (abs(botSpeedMS) > self.max_velocity):
                desiredWheelSpeedR = 2*( self.max_velocity/ (abs(botSpeedMS))) * desiredWheelSpeedR
                desiredWheelSpeedL = 2*(self.max_velocity/ (abs(botSpeedMS))) * desiredWheelSpeedL

                # #Find max velocity in rots 

                # maxBotSpeed = self.max_velocity * (2/self.botDiameter)
                # desiredWheelSpeedL = 2* maxBotSpeed * (desiredWheelSpeedL / (desiredWheelSpeedL+desiredWheelSpeedR)) * (self.encoder_resolution / self.encoder_per_sec_to_rad_per_sec)
                # desiredWheelSpeedR = 2* maxBotSpeed * (desiredWheelSpeedR / (desiredWheelSpeedL+desiredWheelSpeedR)) * (self.encoder_resolution / self.encoder_per_sec_to_rad_per_sec)
                # finalspeed = (desiredWheelSpeedL + desiredWheelSpeedR) /2
                # print finalspeed
            


        #the desired point has been tracked, so don't move
        
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0
                
        return desiredWheelSpeedR,desiredWheelSpeedL

    # def path_tracker(self):

    #     point1 = [0, 0, 0]
    #     point2 = [0.5, 0, 0]
    #     point3 = [1, 0, 0]
        
    #     for ()
    #     self.state_des.set_state(x, y, theta)

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
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        #print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]
    
        
    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('X', 'Y', 'Theta', 'RW', 'LW'))
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [self.state_est.x, self.state_est.y, self.state_est.theta, self.R, self.L]]
        
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
        if state.theta > math.pi:
            state.theta = state.theta - 2*math.pi
        if state.theta < -math.pi:
            state.theta = state.theta + 2*math.pi

        #print "X {0} and Y {1} Theta {2}".format(state.x,state.y,state.theta)
        
        # ****************** Additional Student Code: End ************
            
        # keep this to return the updated state
        return state
        
        
        
        
        
        
        