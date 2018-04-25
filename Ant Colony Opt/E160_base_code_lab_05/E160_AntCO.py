import math
import random
import numpy as np
from E160_state import *

class E160_AntCO:

    def __init__(self, environment, start_robot_state): #removed:robot_radius as an input

       self.ants = []
       self.best_path = []
       self.allPaths = []
       self.environment = environment
       # self.state = E160_state()
       
       # 4/25 3:30 pm added the necessary cnstants from enviornment to instate self.grid
       self.cell_edge_length = 0.1
       self.width = environment.width
       self.height = environment.height
       self.walls = environment.walls
       self.robot_radius = environment.robot_radius
       self.grid = environment.AntGrid(self.width, self.height, self.walls, self.cell_edge_length, self.robot_radius)
       # Variables

       self.num_ants = 10
       self.max_iteration = 10

       # Constants
       # self.num_y_cell = 10
       # self.num_x_cell = 10
       # self.x_grid_cell_size = self.max_x/self.grid_size
       # self.y_grid_cell_size = self.max_x/self.grid_size
       # self.MAX_NODE_NUMBER = 100000
       # self.expansion_range = 0.4


    # def initializeGrid(self, environment, cell_edge_length, robot_radius):
    #     self.grid.discretizeMap()
    #     self.grid.updateOccupany()

    # TODO: Update
    # def update_plan(self, start_robot_state, goal_node):
    #     self.cell_grid = {}
    #     self.node_list = []
    #     self.num_nodes = 0

    #     # Add Code: set the variable self.start_node and add it to the PRM
    #     neighbors = []
    #     self.start_node = self.Node(start_robot_state.x, start_robot_state.y,0.1,0,neighbors, 0)
    #     self.start_node = self.Node(0, 0, 0.1, 0, neighbors, 0)
    #     self.addNode(self.start_node)
               
        
    #     return self.AntColonyPathPlanner(goal_node)

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
        return a

    def InitializeAnts(self):
        ''' Populate self.ants with random Particle 
            Args:
                None
            Return:
                None'''
        self.ants = []
        for i in range(0, self.num_ants):
            #self.SetRandomStartPos(i)
            self.SetKnownStartPos(i)

            
    #def SetRandomStartPos(self, i):
        # add student code here 
		
		# x = random.uniform(self.map_minX, self.map_maxX)
		# y = random.uniform(self.map_minY, self.map_maxY)
		# heading = random.random()*2*math.pi
		# weight = 1.0/(self.numParticles)
		
		# p = self.Particle( x, y, heading, weight)

		# self.particles.append(p)
		# # end student code here
		# pass

    def SetKnownStartPos(self, i):
        # add student code here 
        probability = [0, 0, 0, 0]
        path = []
        current_state = E160_state()
        current_state.set_state(0,0,0)
        desired_state = E160_state()
        desired_state.set_state(0,0,0)
        ant = self.Ant(current_state, desired_state, probability, path)
        self.ants.append(ant)
        # end student code here
        pass

    # TODO: Update, as discussed in class today we do have hurstic information whiich we want to introduce
    # the hurestic information can be the distance betweeen goal cell and cells under consideration
    # where cells under consideration are the neigbouring cells
    # therefore of the neigbours, the cell closest to goal will have higher probability
    def MoveProbability(self, ant, goal_state):
        # gives the probablities of the Ant moving N, NE, E, SE, S, SW, W, or NW
        alpha = 0.6
        beta = 0.4
        for row in range(3):
            column = []
            for col in range(3):
                column.append(0) 
            ant.probability.append(column)    
        

        current_state = ant.current_state
        total_neighbor_pheromones = 0
        total_neighbor_dist = 0
        
        current_x = ant.current_state.x
        current_y = ant.current_state.y
        current_heading = ant.current_state.heading

        goal_x = goal_state.x
        goal_y = goal_state.y
        goal_heading =  goal_state.heading

        # 4/25 3:34 pm rcorrecrted the calling of the functio was self.enviornment.grid.... changed to self.grid....
        [current_row, current_col] = self.grid.returnRowCol(current_row, current_col)
        [goal_row, goal_col] = self.grid.returnRowCol(goal_x, goal_y)

        current_cell =  self.grid.getCell(current_row, current_col)
        goal_cell = self.grid.getCell(current_row, current_col)

        RowColList = [-1, 0, 1]

        for row in RowColList:
            for col in RowColList:
                # 4/25 3:38 pm fixed a bug in calling of getCell (changed from minus to plus)
                c = self.grid.getCell(current_row+row, current_col+col)
                [x, y] = c.returnXY()
                distance_to_goal = math.sqrt(math.pow(goal_x-x,2)+math.pow(goal_y-y,2))
                c.DtoGoal = distance_to_goal
                total_neighbor_dist += distance_to_goal
                total_neighbor_pheromones += c.pheromone
                self.grid.modCellInGrid(c, row, col)

        for row in ant.probability:
            for col in row:
                c = self.environment.grid.getCell(row, col)
                ant.probability[row][col] = math.pow(c.pheromone, alpha) * math.pow(c.DtoGoal, beta) / (total_neighbor_dist*total_neighbor_pheromones)



        # Hurestic information








    #sub class grid
    #write get x, y cell/ grid cell
    #get cell center 
    #header file for dim and #cells
    #walls and obstacles
    #grid cell, has obstacle notation
    #space and cell width gives everything 

    #grid in environment? 

    #holonomic constraints
    #every node have two nodes in trajectory
    #set orientations for nodes
    #trajectory needs orientations
    #do some atan

    # TODO: Update
    def AntColonyPathPlanner(self, goal_state):
        # Start ants at nest 
        self.InitializeAnts()

        # establish criteria for stopping
        path_found = False
        iteration = 0

        #initialize the path
        current_path = []
        while(iteration <= self.max_iteration):
            #iterate ants until a path is found
            for ant in self.ants:
                lastMove = 0
                path_found = False
                while (path_found == False):
                    #Find current cell
                    current_state = ant.current_state
                    # 4/25 3:42pm corrected call of returnRowCAll
                    [row, col] = self.grid.returnRowCol(x,y)
                    current_cell = self.grid.getCell(row,col)

                    #update ant motion probability
                    self.MoveProbability(ant)

                    #determine best direction
                    ant.probability[lastMove] = 0
                    nextMoveDirection = ant.probability.index(max(ant.probability))
                    
                    #set up next state and save lastMove
                    [new_x,new_y] = self.setNewState(current_state, nextMoveDirection)
                    lastMove = self.setLastMove(current_state, nextMoveDirection)

                    #add current cell to path 
                    ant.path.append(current_cell)

                    #check for complete path
                    if (current_state.x == goal_state.x) & (current_state.y == goal_state.y):
                        #Path found! 
                        #Check if it's a good path
                        if len(self.best_path) == 0:
                            #first path found
                            self.best_path = ant.path
                        if len(ant.path) < len(self.best_path):
                            #better path found
                            self.best_path = ant.path
                        self.allPaths.append(ant.path)
                        #set current path
                        current_path = ant.path
                        path_found = True
                        print "path found"
                            
                    #Move to next state if no path
                    ant.current_state.x = new_x
                    ant.current_state.y = new_y
                    #TODO:fix theta

                #a path is found. evaporate pheromones 
                self.updatePheromones(current_path)

            #iterate paths
            iteration += 1

        
            print "sucess!!"
            print self.best_path
        # return the best path

        return self.best_path

    def updatePheromones(self, current_path):
        roe = 5
        numCols = self.grid.numberOfCols
        numRows = self.grid.numberOfRows
        for row in numRows:
            for col in numCols:
                cell = self.grid.getCell(row,col)
                cell.pheromone = cell.pheromone*(1-roe)
                self.grid.modCellInGrid(cell, row, col)

        
        ck = len(current_path)
        for cell in current_path:
            cell.pheromone += (1/ck)
            [x,y] = cell.returnXY()
            [row, col] = self.grid.returnRowCol(x, y)
            self.grid.modCellInGrid(cell, row, col)

    
    #return new state variables
    def setNewState(self, current_state, nextMoveDirection):
        x = current_state.x            
        y = current_state.y
        theta = current_state.theta

        #initialize new row/col
        new_row = 0
        new_col = 0

        [row, col] = self.environment.cell.returnRowCol(x,y)
        if nextMoveDirection == 0:
            #go north
            new_col = col + 1
            new_row = row
        if nextMoveDirection == 1:
            #go east 
            new_col = col 
            new_row = row - 1       
        if nextMoveDirection == 2:
            #go west
            new_col = col 
            new_row = row + 1 
        if nextMoveDirection == 3:
            #go south
            new_col = col - 1
            new_row = row

        #get new x, y
        new_cell = self.grid.getCell(new_row,new_col)
        new_x = new_cell.x
        new_y = new_cell.y

        return [new_x, new_y]
    
    #returns the next move
    def setLastMove(self, current_state, nextMoveDirection):
        if nextMoveDirection == 0:
            #go north, y+1
            lastMove = 3
            y = current_state.y + 1

        if nextMoveDirection == 1:
            #go east 
            lastMove = 2
                    
        if nextMoveDirection == 2:
            lastMove = 1
        
        if nextMoveDirection == 3:
            lastMove = 0

        return lastMove


    def build_trajectory(self, goal_node):
        '''Given a goal_node, build a trajectory from start to goal
            Args:
                goal_node (Node)
            returnL
                a list of node index'''
        node = goal_node
        trajectory = []
        trajectory.append(goal_node.index)
        while(node.index != 0):
            # print node.index
            node = node.parent
            trajectory.append(node.index)
        return trajectory[::-1]

    def check_collision(self, node1, node2, tolerance):
        '''Check if there is a obstacle between the two node
            Args:
                node1 (Node)
                node2 (Node)
            Return:
                bool: True if collision, false if not '''

        # Loop through all the walls in the environment
        for wall in self.environment.walls:
            #for each wall, check for collision for each of the four line
            p1 = wall.points[:2]
            p2 = wall.points[2:4]
            p3 = wall.points[4:6]
            p4 = wall.points[6:8]
            line_p1 = self.Node(p1[0] - tolerance, p1[1] + tolerance)
            line_p2 = self.Node(p2[0] + tolerance, p2[1] + tolerance)
            line_p3 = self.Node(p3[0] + tolerance, p3[1] - tolerance)
            line_p4 = self.Node(p4[0] - tolerance, p4[1] - tolerance)

            # Given four points, check for 4 collisions
            b1 = self.check_line_collision(node1, node2, line_p1, line_p2)
            b2 = self.check_line_collision(node1, node2, line_p2, line_p3)
            b3 = self.check_line_collision(node1, node2, line_p3, line_p4)
            b4 = self.check_line_collision(node1, node2, line_p4, line_p1)
            
            # if there is a collision, 
            if (b1 or b2 or b3 or b4):
                return True
            # else
                # do nothing and keep going

        return False

    def check_line_collision(self, node1, node2, line1, line2):
        '''Check for collision between two arbitrary line, taken from
        https://stackoverflow.com/questions/563198
        /how-do-you-detect-where-two-line-segments-intersect#565282
            Args:
                node1 (Node): start node
                node2 (Node): end node
                line1 (Node): wall point 1
                line2 (Node): wall point 2
            Return:
                bool: True if collision'''
        s1_x = node2.x - node1.x
        s1_y = node2.y - node1.y
        s2_x = line2.x - line1.x
        s2_y = line2.y - line1.y

        d1 = (-s2_x * s1_y + s1_x * s2_y)
        if (d1 == 0.0):
            d1 = 0.001
        s = (-s1_y * (node1.x - line1.x) + s1_x * (node1.y - line1.y)) / d1
        t = ( s2_x * (node1.y - line1.y) - s2_y * (node1.x - line1.x)) / d1
        
        # intersect
        i_x = 0
        i_y = 0
        if (s >= 0 and s <= 1 and t >= 0 and t <= 1):
            # Collision detected
            i_x = node1.x + (t * s1_x)
            i_y = node1.y + (t * s1_y)
            return True
        return False

    # Ant class will take argument current state desired state
    class Ant:
        def __init__(self, current_state, desired_state, probability, path):
            self.current_state = current_state
            self.desired_state = desired_state
            self.probability = probability
            self.path = []


        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.probability)

        def pathLength(self):
            return len(self.path)