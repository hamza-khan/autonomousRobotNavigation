from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
import math
from xbee import XBee


class E160_environment:

    
    def __init__(self):
        self.width = 2.0
        self.height = 1.2
        self.cell_edge_length = 0.1
        self.robot_radius = 0.147/2
        # set up walls, putting top left point first
        self.walls = []
        self.walls.append(E160_wall([-0.5, 0.5, -0.5, -0.5],"vertical"))
        #self.walls.append(E160_wall([0.5, 0.5, 0.5, -0.5],"vertical"))
        self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5],"horizontal"))
        # self.walls.append(E160_wall([0.0, -0.5, 0.0, -1.0],"vertical"))
        # self.walls.append(E160_wall([0.0, -1.0, 1.0, -1.0],"horizontal"))
        # self.walls.append(E160_wall([0.5, -0.5, 1.0, -0.5],"horizontal"))
        # self.walls.append(E160_wall([1.0, -0.5, 1.0, -1.0],"vertical"))
        # self.walls.append(E160_wall([1.0, -1.0, 1.0, -1.5],"vertical"))
        
        #set up the AntGrid 
        self.grid = self.AntGrid(self.width, self.height, self.walls, self.cell_edge_length, self.robot_radius)        
        self.grid.initializeGrid()

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
                print "Couldn't find the serial port" 
        
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
    class AntGrid:
        """docstring for ClassName"""
        def __init__(self, width, height, walls, cell_edge_length, robot_radius):
            #self.environment = environment #removed  passing in environment
            self.cell_edge_length = cell_edge_length
            self.robot_radius = robot_radius
            self.width = width
            self.height = height
            self.walls = walls
            self.discretizedMap = []

        # TODO: fix __str__
        # def __str__(self):
        #     return str(self.cell_edge_length) + " " + str(self.robot_radius)

        def numberOfCells(self):
            numCells = self.numberOfCols()*self.numberOfRows()
            return numCells
            
        def numberOfCols(self):
            mapWidth = self.width
            numCols = math.ceil(mapWidth/self.cell_edge_length)
            return int(numCols)

        def numberOfRows(self):
            mapHeight = self.height
            numRows = math.ceil(mapHeight/self.cell_edge_length)
            return int(numRows)
        
        def initializeGrid(self):
            self.discretizeMap()
            self.addWalls()


        def discretizeMap(self):
            ##access by calling discretizeMap[row][col]
            numRows = self.numberOfRows()
            print "numrows: ", numRows
            numCols = self.numberOfCols()
            print "numcols: ", numCols
            for row in range(numRows):
                rowList = []
                for col in range(numCols):
                    rowList.append(self.AntCell(row, col, self, 0.1, False, 10000)) 
                self.discretizedMap.append(rowList)
            pass


        def getCell(self, row, col):
            if row >= self.numberOfRows() or col>= self.numberOfCols():
                c = self.AntCell(row, col, self, 0, True, 10000)
            elif row < 0 or col < 0:
                c = self.AntCell(row, col, self, 0, True, 10000)
            else:
               # print "row,col", row, col
                c = self.discretizedMap[row][col]
            return c

        def modCellInGrid(self, cell, row, col):
            self.discretizedMap[row][col] = cell



        def addWalls(self):
            walls = self.walls

            for wall in walls:
                [xa, xb, ya, yb] = self.bufferWallCoordinates(wall)
                for row in range(self.numberOfRows()):
                    for col in range(self.numberOfCols()):                    
                        cell = self.getCell(row,col)
                        x, y = cell.returnXY()
                        if x >= xa and x <= xb:
                            if y <= yb and y >= ya: 
                                cell.occupied = True
                                print x,y
            pass


        def bufferWallCoordinates(self, wall):
            robotRadius = self.robot_radius
            bufferScaling = 0 #added a buffer zone scaling factor
            if wall.slope == "vertical":
                print "wall.points: ", wall.points
                xa = wall.points[0] #+ wall.radius
                ya = wall.points[5] #- wall.radius
                xb = wall.points[2] #- wall.radius
                yb = wall.points[1] #+ wall.radius
                xa = xa - (bufferScaling*robotRadius)
                xb = xb + (bufferScaling*robotRadius)
                ya = ya - (bufferScaling*robotRadius)
                yb = yb + (bufferScaling*robotRadius)
        # horizontal wall
            else:
                xa = wall.points[0] #+ wall.radius
                ya = wall.points[1] #+ wall.radius
                xb = wall.points[4] #- wall.radius
                yb = wall.points[5] #- wall.radius
                xa = xa - (bufferScaling*robotRadius)
                xb = xb + (bufferScaling*robotRadius)
                ya = ya - (bufferScaling*robotRadius)
                yb = yb + (bufferScaling*robotRadius)
            return [xa, xb, ya, yb]

        # added a new function for returning row col
        #9:10am 4/26 changed row col function because flooring wasn't working
        def returnRowCol(self, x, y):
            for row in range(self.numberOfRows()):
                for col in range(self.numberOfCols()):
                    c = self.getCell(row,col)
                   
                    #x, y = c.returnXY()
                    [xa,ya,xb,yb] = c.returnCellDim(row, col)
                    if y > ya and y < yb:
                        if x > xa and x < xb:
                            #print "xa, xb, ya, yb", xa, xb, ya, yb
                            #print "cell x y", c.returnXY()
                            #print "cell row col", row,col
                            return [row, col]
                            
        	pass

        # def returnRowCol(self, x, y):
        #     col = math.floor((x + (self.width/2))/self.cell_edge_length )  
        #     row = math.floor((x + (self.height/2))/self.cell_edge_length )

        #     return row, col

        class AntCell:
            """docstring for ClassName"""
            def __init__(self, row, col, grid, pheromone = 0.1, occupied = False, DtoGoal = 10000):
                self.row = row
                self.col = col
                self.pheromone = pheromone
                self.occupied = occupied
                self.DtoGoal = DtoGoal
                self.cell_edge_length = grid.cell_edge_length
                self.width = grid.width
                self.height = grid.height

            def returnXY(self): 
                x = ((self.col)*self.cell_edge_length) - self.width/2 + self.cell_edge_length/2
                y = ((self.row)*self.cell_edge_length) - self.height/2 + self.cell_edge_length/2
                #change: got rid of +1 from row and col and the edge_length/2
                return x,y

            #returns [xa,ya,xb,yb] or [[center][corners]]
            def returnCellDim(self, row, col): 
                x = ((col)*self.cell_edge_length) - self.width/2  + self.cell_edge_length/2
                y = ((row)*self.cell_edge_length) - self.height/2 + self.cell_edge_length/2
                #change: got rid of +1 from row and col and the edge_length/2
                xa = x - (self.cell_edge_length/2)
                xb = x + (self.cell_edge_length/2)
                ya = y - (self.cell_edge_length/2)
                yb = y + (self.cell_edge_length/2)

                #10am 4/26 changed to list
                return [xa,ya,xb,yb]

            # #TODO: wrong
            # #inputs any x and y and gets cell using returnCellDim as bounds
            # def returnRowCol(self, x, y):
            #     row = (((2*y)+self.height)/self.cell_edge_length) - 1
            #     col = (((2*x)+self.width)/self.cell_edge_length) - 1

            #     row = math.ceiling(row)
            #     col = math.ceiling(col)

            #     return [row, col]