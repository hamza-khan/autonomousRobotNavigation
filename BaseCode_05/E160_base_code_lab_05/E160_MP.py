import math
import random
import numpy as np

class E160_MP:

    def __init__(self, environment, start_robot_state, robot_radius):
       # Cell grid and node list
       self.cell_grid = {}
       self.node_list = []
       self.traj_node_list = []

       self.environment = environment

       # Variables
       self.num_nodes = 0

       self.max_x = 2.0
       self.max_y = 2.0
       self.min_x = -2.0
       self.min_y = -2.0

       # Constants
       self.num_y_cell = 10
       self.num_x_cell = 10
       self.x_grid_cell_size = self.max_x/self.num_x_cell
       self.y_grid_cell_size = self.max_x/self.num_y_cell
       self.MAX_NODE_NUMBER = 100000
       self.expansion_range = 0.4
       self.robot_radius = robot_radius

       start_node = self.Node(start_robot_state.x, start_robot_state.y)
       self.start_node = start_node
       self.addNode(self.start_node)

    def update_plan(self, start_robot_state, goal_node):
        self.cell_grid = {}
        self.node_list = []
        self.num_nodes = 0

        # Add Code: set the variable self.start_node and add it to the PRM
        children = []
        self.start_node = self.Node(start_robot_state.x, start_robot_state.y,None,children, 0)
        self.addNode(self.start_node)
               
        
        return self.MotionPlanner(goal_node)

    def addNode(self, n):
        '''Add node n in self.cell_grid'''
        
        col, row = self.getCellNumbder(n)
        if (col, row) in self.cell_grid:
            self.cell_grid[col, row].append(n)
        else:
            self.cell_grid[col, row] = [n]
        n.index = self.num_nodes
        self.node_list.append(n)
        self.num_nodes += 1 

        #print self.cell_grid.keys()[0][0]

    def getCellNumbder(self, n):
        '''Calculate x and y indices for a given node '''
        col = math.floor((n.x - self.min_x)/self.x_grid_cell_size )
        row = math.floor((n.y - self.min_y)/self.y_grid_cell_size )
        return col, row

    def select_expansion_node(self):
        '''Randomly select a node to expand on in the grid
            Return
                Node '''
        cell_length = len(self.cell_grid.keys())
        random_cell_num = int(random.random() * cell_length)
        random_key = self.cell_grid.keys()[random_cell_num]
        random_node_num = int(random.random() * len(self.cell_grid[random_key]))
        return self.cell_grid[random_key][random_node_num]

    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
        return a

    def MotionPlanner(self, goal_node):
        '''Come up with a trajectory plan using RRT from the start to
            the goal node
            Args:
                goal_node (Node): node that robot should go to
            Return:
                [a list of node_indices] '''
        # establish criteria for stopping
        path_found = False
        iteration = 0

        # trivial case: if start node connect to goal_node
        if (self.check_collision(self.start_node, goal_node, self.robot_radius) == False):
            goal_node.parent = self.start_node
            self.start_node.children.append(goal_node)
            self.addNode(goal_node)
            path_found = True

        # while loop to continue add node until one of the criteria
        # is met
        
        while(iteration < self.MAX_NODE_NUMBER and path_found == False):
            # Add Code: randomly select an expansion node
            expansion_node = self.select_expansion_node()

            # Add Code: From the expansion node, create a new node
            randLength = 3*self.robot_radius*random.random()
            randOrientation = self.angle_wrap(2*math.pi*random.random())
    
            #Find orientation and intialize children of new node
            xNewNode = randLength*math.cos(randOrientation) + expansion_node.x
            yNewNode = randLength*math.sin(randOrientation) + expansion_node.y
            #childrenNewNode = []

            #Create new node
            new_node = self.Node(xNewNode,yNewNode,expansion_node,[],self.num_nodes)
            
            # Add Code: check collision for the expansion
            #if ...
            if not self.check_collision(new_node.parent, new_node, 2.5*self.robot_radius):
                self.addNode(new_node)
                expansion_node.children.append(new_node)
                

            # slope = math.tan(randOrientation)
            # y_intercept = yNewNode + slope*xNewNode

            # lineEq = slope*xNewNode + y_intercept

            # collision = False
            # for i in self.environment.walls:
            #     if i.slope == "vertical":
            #         x1 = i.points[0] + wall.radius
            #         y1 = i.points[1] - wall.radius
            #         x2 = i.points[4] - wall.radius
            #         y2 = i.points[5] + wall.radius
                    
            #         #Check for intersection
            #         x_int = x1
            #         y_int = slope*x_int + y_intercept 

            #         min_x = min(x1,x2)
            #         max_x = max(x1,x2) 
            #         min_y = min(y1,y2)
            #         max_y = max(y1,y2) 

            #         if min_y <= y_int <= max_y:
            #             collision = True
            #             break

            #     if i.slope == "horizontal":
            #         x1 = wall.points[0] + wall.radius
            #         y1 = wall.points[1] + wall.radius
            #         x2 = wall.points[4] - wall.radius
            #         y2 = wall.points[5] - wall.radius

            #         y_int = y1
            #         x_int = (y_int - y_intercept)/slope

            #         min_x = min(x1,x2)
            #         max_x = max(x1,x2) 
            #         min_y = min(y1,y2)
            #         max_y = max(y1,y2) 

            #         if min_x <= x_int <= max_x:
            #             collision = True
            #             break
            
            #if no collision add node
            #if collision = False:
                
            
            # Add Code: check if stopping criteria is met or not

                if not self.check_collision(new_node, goal_node, 2.5*self.robot_radius):
                    goal_node.parent = new_node
                    new_node.children.append(goal_node)                
                    self.addNode(goal_node)
                    expansion_node.children.append(goal_node)
                    path_found = True       
            
            # keep track of the number of attempted expansions
            iteration += 1
            
        # build the trajectory
        self.traj_node_list = self.build_trajectory(goal_node)
        
        print self.traj_node_list
        # return the trajectory
        return self.traj_node_list

    
    
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
        def __init__(self, x = 0, y = 0, parent = None, children = [], index = 0):
            self.x = x
            self.y = y
            self.parent = parent
            self.children = []
            self.index = index

        def __str__(self):
            return str(self.x) + " " + str(self.y)
        
        def __repr__(self):
            return '[' + str(self.x) + "," + str(self.y) + "," + str(self.index) +  ']'
