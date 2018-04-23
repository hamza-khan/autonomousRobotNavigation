import math
import random
import numpy as np

class E160_ACO:

    def __init__(self, environment, start_robot_state, robot_radius):
       # Cell grid and node list
       self.cell_grid = {}
       self.node_list = []
       self.traj_node_list = []
       self.ants = []
       self.best_path = []
       self.allPaths = []
       self.environment = environment

       # Variables
       self.num_nodes = 0
       self.num_ants = 10
       self.max_x = 2.0
       self.max_y = 2.0
       self.min_x = -2.0
       self.min_y = -2.0
       self.grid_size = 10
       self.max_iteration = 10

       # Constants
       self.num_y_cell = 10
       self.num_x_cell = 10
       self.x_grid_cell_size = self.max_x/self.grid_size
       self.y_grid_cell_size = self.max_x/self.grid_size
       self.MAX_NODE_NUMBER = 100000
       self.expansion_range = 0.4
       self.robot_radius = robot_radius

       start_node = self.Node(start_robot_state.x, start_robot_state.y,0.1,0,[], 0)
       self.start_node = start_node
       self.addNode(self.start_node)

    def update_plan(self, start_robot_state, goal_node):
        self.cell_grid = {}
        self.node_list = []
        self.num_nodes = 0

        # Add Code: set the variable self.start_node and add it to the PRM
        neighbors = []
        self.start_node = self.Node(start_robot_state.x, start_robot_state.y,0.1,0,neighbors, 0)
        self.start_node = self.Node(0, 0, 0.1, 0, neighbors, 0)
        self.addNode(self.start_node)
               
        
        return self.AntColonyPathPlanner(goal_node)

    def addNode(self, n):
        '''Add node n in self.cell_grid'''
        # print self.cell_grid.keys()
        col, row = self.getCellNumbder(n)
        if (col, row) in self.cell_grid:
            self.cell_grid[col, row].append(n)
        else:
            self.cell_grid[col, row] = [n]
        n.index = self.num_nodes
        self.node_list.append(n)
        self.num_nodes += 1 

    def getCellNumbder(self, n):
        '''Calculate x and y indices for a given node '''
        col = math.floor((n.x - self.min_x)/self.x_grid_cell_size )
        row = math.floor((n.y - self.min_y)/self.y_grid_cell_size )
        return col, row

    # def select_expansion_node(self):
    #     '''Randomly select a node to expand on in the grid
    #         Return
    #             Node '''
    #     #select based on this probability
    #     cell_length = len(self.cell_grid.keys())
    #     random_cell_num = int(random.random() * cell_length)
    #     random_key = self.cell_grid.keys()[random_cell_num]
    #     random_node_num = int(random.random() * len(self.cell_grid[random_key]))
    #     return self.cell_grid[random_key][random_node_num]

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
        ant = self.Ant( self.start_node, 0, probability, path)
        self.ants.append(ant)
        # end student code here
        pass

    def MoveProbability(self, ant):
        # gives the probablities of the Ant moving N, E, W, or S

        ant.probability = [0, 0, 0, 0]
        current_node = ant.current_node
        total_neighbor_pheromones = 0

        # total number of pheromones in all neighbors
        for index in range(len(current_node.neighbors)):
            if current_node.neighbors[index] != None:
        	    total_neighbor_pheromones += current_node.neighbors[index].pheromone
        #print current_node.neighbors[0].pheromone
        
        #if total_neighbor_pheromones == 0:
        #    Ant.probability = [0, 0, 0, 0]
            #print "setting to zero"  
        
        for index in range(len(current_node.neighbors)):
            if current_node.neighbors[index] != None:
                if current_node.neighbors[index].y == current_node.y + 1:
                    ant.probability[0] = current_node.neighbors[index].pheromone/total_neighbor_pheromones
                if current_node.neighbors[index].x == current_node.x + 1:
                    ant.probability[1] = current_node.neighbors[index].pheromone/total_neighbor_pheromones
                if current_node.neighbors[index].x == current_node.x - 1:
                    ant.probability[2] = current_node.neighbors[index].pheromone/total_neighbor_pheromones
                if current_node.neighbors[index].y == current_node.x - 1:
                    ant.probability[3] = current_node.neighbors[index].pheromone/total_neighbor_pheromones
        #print Ant.probability

        #adds pheromone probability in [N, E, W, S]
        # for index in range(len(current_node.neighbors)):
        #     if current_node.neighbors[index].y == current_node.y + 1:
        #         Ant.probability[0] = current_node.neighbors[index].pheromone/total_neighbor_phermones
        #     if current_node.neighbors[index].x == current_node.x + 1:
        #         Ant.probability[1] = current_node.neighbors[index].pheromone/total_neighbor_phermones
        #     if current_node.neighbors[index].x == current_node.x - 1:
        #         Ant.probability[2] = current_node.neighbors[index].pheromone/total_neighbor_phermones
        #     if current_node.neighbors[index].y == current_node.x - 1:
        #         Ant.probability[3] = current_node.neighbors[index].pheromone/total_neighbor_phermones
        

    def InitializeGrid(self):
        #TODO: add walls as input
        #Code to get grid and set nodes
        #col = math.floor((n.x - self.min_x)/self.x_grid_cell_size )
        #row = math.floor((n.y - self.min_y)/self.y_grid_cell_size )
        for x in range(self.grid_size):
            for y in range(self.grid_size):     
                newNode = self.Node(x, y, 0.1, 0, [])
                self.addNode(newNode)
                
                #print x,y
        
        for node in self.node_list:
           # col, row = self.getCellNumbder(Node)
            x = node.x
            y = node.y

            colx = math.floor((x+1 - self.min_x)/self.x_grid_cell_size )
            rowy = math.floor((y - self.min_y)/self.y_grid_cell_size )
            if (colx, rowy) in self.cell_grid:
                node.neighbors[1] = (self.cell_grid[colx, rowy][0])
                print "added 1"
            
            colx = math.floor((x-1 - self.min_x)/self.x_grid_cell_size )
            rowy = math.floor((y - self.min_y)/self.y_grid_cell_size )
            if (colx, rowy) in self.cell_grid:
                node.neighbors[2] = (self.cell_grid[colx, rowy][0])
                print "added 2"
            
            colx = math.floor((x - self.min_x)/self.x_grid_cell_size )
            rowy = math.floor((y+1 - self.min_y)/self.y_grid_cell_size )
            if (colx, rowy) in self.cell_grid:
                node.neighbors[0] = (self.cell_grid[colx, rowy][0])
                print "added 3"

            colx = math.floor((x - self.min_x)/self.x_grid_cell_size )
            rowy = math.floor((y-1 - self.min_y)/self.y_grid_cell_size )
            if (colx, rowy) in self.cell_grid:
                node.neighbors[3] = (self.cell_grid[colx, rowy][0])
                print "added 4"

            #  print self.cell_grid[x,y]
            #print x,y
            print node.neighbors

    def AntColonyPathPlanner(self, goal_node):
        
        #Discritize map 
        self.InitializeGrid()

        #Start ants at nest 
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
                while (path_found == False):
                    
                    current_node = ant.current_node

                    #update ant motion probability
                    self.MoveProbability(ant)

                    #determine best direction
                    #ant.probability[lastMove] = 0
                    #print ant.probability
                    ant.probability[lastMove] = 0
                    for x in range(4):
                        if current_node.neighbors[x] == None:
                            ant.probability[x] = 0

                    nextMoveDirection = ant.probability.index(max(ant.probability))
                    print ant.probability
                    print nextMoveDirection
                    #print current_node
                    print current_node.neighbors

                    #for node in ant.path:
                    #    node.pheromone = 0
                    
                    if nextMoveDirection == 0:
                        lastMove = 3
                    if nextMoveDirection == 1:
                        lastMove = 2
                    if nextMoveDirection == 2:
                        lastMove = 1
                    if nextMoveDirection == 3:
                        lastMove = 0
                    
                    #print ant.probability

                    #add current node to path 
                    ant.path.append(current_node)

                    #check for complete path
                    if (current_node.x == goal_node.x) & (current_node.y == goal_node.y):
                        #Path found! 
                        #Check if it's a good path
                        if len(self.best_path) == 0:
                            #first path found
                            self.best_path = ant.path
                        if len(ant.path) < len(self.best_path):
                            #better path found
                            self.best_path = ant.path
                        #set current path
                        current_path = ant.path
                        path_found = True
                        print "I am going through Ants"
                            
                    #Move to next node if no path
                    ant.current_node = ant.current_node.neighbors[nextMoveDirection]
                    
                    if ant.current_node == None:
                        print "current node is None"
                    #print "I am searching for path"
                    #print ant.current_node.x, ant.current_node.y
           
            #found complete path
            iteration += 1
            print "I am finding better path"

            #a path is found. evaporate pheromones 
            roe = 0.5
            for node in node_list:
                node.pheromone = node.pheromone*(1-roe)

            #Add pheromones to path 
            numAntsOnPath = 0
            for ant in self.ants:
                if ant.path == current_path:
                    numAntsOnPath += 1

            ck = len(current_path)
            for node in current_path:
                node.pheromone += numAntsOnPath*(1/ck)

        #iterate paths
        iteration += 1


            # if nextMoveDirection == 1:
            #     #go east
            #     ant.current_node = #node.x + 1

            # if nextMoveDirection == 2:
            #     #go west
            #     ant.current_node = #node.x - 1

            # if nextMoveDirection == 3:
            #     #go south
            #     ant.current_node = #node.y - 1
            #current_node = current_node.children[nextMoveDirection]
        
        # print self.best_path
        # return the best path
        return self.best_path


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

    class Node:
        def __init__(self, x = 0, y = 0, pheromone = 0, time = 0, neighbors = [None, None, None, None], index = 0):
            self.x = x
            self.y = y
            self.pheromone = 0.1 
            self.time = 0
            self.neighbors = [None, None, None, None]
            self.index = index

        def __str__(self):
            return str(self.x) + " " + str(self.y)
        
        def __repr__(self):
            return '[' + str(self.x) + "," + str(self.y) + "," + str(self.index) +  ']'

    class Ant:
        def __init__(self, current_node, heading, probability, path):
            self.current_node = current_node
            self.heading = heading
            self.probability = probability
            self.path = []

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.probability)
