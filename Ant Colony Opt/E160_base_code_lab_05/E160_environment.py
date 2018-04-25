from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
from xbee import XBee


class E160_environment:

    
    def __init__(self):
        self.width = 2.0
        self.height = 1.2
        
        # set up walls, putting top left point first
        self.walls = []
        self.walls.append(E160_wall([-0.5, 0.5, -0.5, -0.5],"vertical"))
        self.walls.append(E160_wall([0.5, 0.5, 0.5, -0.5],"vertical"))
        self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5],"horizontal"))
        self.walls.append(E160_wall([0.0, -0.5, 0.0, -1.0],"vertical"))
        self.walls.append(E160_wall([0.0, -1.0, 1.0, -1.0],"horizontal"))
        self.walls.append(E160_wall([0.5, -0.5, 1.0, -0.5],"horizontal"))
        self.walls.append(E160_wall([1.0, -0.5, 1.0, -1.0],"vertical"))
        self.walls.append(E160_wall([1.0, -1.0, 1.0, -1.5],"vertical"))
            
        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "AUTONOMOUS CONTROL MODE" #"MANUAL CONTROL MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial('/dev/tty.usbserial-DN01IWND', 9600)
            print" Setting up serial port"
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range (0,self.num_robots):
            
            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i)
            self.robots.append(r)
    
    def update_robots(self, deltaT):
        
        # loop over all robots and update their state
        for r in self.robots:
            
            # set the control actuation
            r.update(deltaT)
        
        
    def log_data(self):
        
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()
            
    def quit(self):
        self.xbee.halt()
        self.serial.close()


        #  add function getCell to grid
    class grid:
    	"""docstring for ClassName"""
    	def __init__(self, environment, cell_edge_length, robot_radius):
    		self.environment = environment
    		self.cell_edge_length = cell_edge_length
    		self.robot_radius = robot_radius
    		self.discritizedMap = []

		# TODO: fix __str__
		# def __str__(self):
        #     return str(self.cell_edge_length) + " " + str(self.robot_radius)

        def numberOfCells(self):
        	numCells = self.numberOfCols()*self.numberOfRows()
        	return numCells
            
        def numberOfCols(self):
            mapWidth = self.environment.width
            numCols = mapWidth/self.cell_edge_length
            return numCols

        def numberOfRows(self):
            mapHeight = self.environment.height
            numRows = mapHeight/self.cell_edge_length
            return numRows

    	def discritizeMap(self):
    		##access by calling discritizeMap[row][col]
            for row in numRows:
                column = []
                for col in numCols:
                    column.append(self.cell(row, col, self, 0.1, False, False)) 
                self.discritizedMap.append(column)
            pass
		
		def getCell(self, row, col):
			if row >= self.numberOfRows() or col>= self.numberOfCols():
				c = environment.cell(row, col, self, 0, True)
			elif row < 0 or col < 0:
				c = environment.cell(row, col, self, 0, True)
			else:
				c = self.discritizedMap[row][col]
		    return c


        def updateOccupancy(self):
            walls = self.environment.walls

            for wall in walls:
                [xa, xb, ya, yb] = self.bufferWallCoordinates(wall)
                for cell in self:
                    [x,y] = cell.returnXY()
                    if x > xa and x < xb:
                        if y > yb and y < ya: 
                            cell.occupied = True
            pass


        def bufferWallCoordinates(self, wall):
            robotRadius = self.robot_radius
            if wall.slope == "vertical":
                x1 = wall.points[0] + wall.radius
                y1 = wall.points[1] - wall.radius
                x2 = wall.points[4] - wall.radius
                y2 = wall.points[5] + wall.radius
                xa = x1 - (2.5*robotRadius)
                xb = x1 + (2.5*robotRadius)
                ya = y1 + (2.5*robotRadius)
                yb = y2 - (2.5*robotRadius)
        # horizontal wall
            else:
                x1 = wall.points[0] + wall.radius
                y1 = wall.points[1] + wall.radius
                x2 = wall.points[4] - wall.radius
                y2 = wall.points[5] - wall.radius
                xa = x1 - (2.5*robotRadius)
                xb = x2 + (2.5*robotRadius)
                ya = y1 + (2.5*robotRadius)
                yb = y1 - (2.5*robotRadius)
            return [xa, xb, ya, yb]

	    def returnRowCol(self, x, y):
	    	for row in numRows:
	    		for col in numCols:
	    			c = self.getCell(row,col)
	    			[[x,y][xa,xb,ya,yb]] = c.returnCellDim()
	    			if y > ya and y < yb:
	    				if x > xa and x < xb:
	    					return [row col]
	    	pass



    class cell:
        """docstring for ClassName"""
        def __init__(self, row, col, grid, pheromone = 0.1, occupied = False, isGoal = False):
            self.row = row
            self.col = col
            self.pheromone = pheromone
            self.occupied = occupied
            self.isGoal = isGoal
            self.cell_edge_length = grid.cell_edge_length
            self.width = grid.environment.width
            self.height = grid.environment.height

        def returnXY(self):
            x = ((self.col + 1)*self.cell_edge_length/2) - self.width/2
            y = ((self.row + 1)*self.cell_edge_length/2) - self.height/2

            return [x,y]

        #returns [[x,y][xa,xb,ya,yb]] or [[center][corners]]
        def returnCellDim(self):
            x = ((self.col + 1)*self.cell_edge_length/2) - self.width/2
            y = ((self.row + 1)*self.cell_edge_length/2) - self.height/2

            xa = x - (self.width/2)
            xb = x + (self.width/2)
            ya = y - (self.cell_edge_length/2)
            yb = y + (self.cell_edge_length/2)

            return [[x,y][xa,xb,ya,yb]]

        # #TODO: wrong
        # #inputs any x and y and gets cell using returnCellDim as bounds
        # def returnRowCol(self, x, y):
        #     row = (((2*y)+self.height)/self.cell_edge_length) - 1
        #     col = (((2*x)+self.width)/self.cell_edge_length) - 1

        #     row = math.ceiling(row)
        #     col = math.ceiling(col)

        #     return [row, col]