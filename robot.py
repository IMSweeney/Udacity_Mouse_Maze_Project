"""
This module contains the Robot class for the Udacity Micromouse Maze Capstone
Project.

@Author: Ian Sweeney
@Date: 07/15/2020
"""

from queue import Queue
from graphics import *
import numpy as np
import pandas as pd


class Node():
    """
    This class represtents one of the squares of the maze for use in a graph.

    Attributes:
        location (int, int): A tuple defining the x,y location of the node
        edges (set): A set of nodes representing the edges of the graph.
    """

    def __init__(self, x, y):
        """ Initializes a node with an x and y position and no edges. """
        self.location = (x, y)
        self.edges = set()  # each edge is a tuple (other node, distance)

    def add_edge(self, other):
        """ Takes in a node, other, and adds it to this nodes edges. """
        self.edges.add(other)
        other.edges.add(self)

    def wall_between(self, other):
        """ Returns False if there is an edge between this node and other """
        return other not in self.edges

    def distance(self, other):
        """ Returns Manhattan distance between this node and other. """
        return (np.abs(self.location[0] - other.location[0]) +
                np.abs(self.location[1] - other.location[1]))

    # From: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    def get_path_to(self, other):
        """
        Gets a path (or list of nodes) from this node to other.

        Uses a depth first search to find the shortest path from this node
        (self) to another node (other). Adapted from code by Red Blob Games:
        https://www.redblobgames.com/pathfinding/a-star/introduction.html

        Parameters:
        other (Node): The node on the graph to plan a path to.

        Returns:
        path (list): A list of nodes defining the steps to take from self to
        other.
        """
        if self == other:
            print('This is an empty path')
            return []

        frontier = Queue()
        frontier.put(self)  # Add the current node to the fronier
        came_from = {}
        came_from[self] = None

        while not frontier.empty():
            current = frontier.get()
            for node in current.edges:
                if node not in came_from:
                    frontier.put(node)
                    came_from[node] = current

        current = other
        path = []
        while current != self:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def __repr__(self):
        """ Returns the location of this node as a string. """
        return str(self.location)

    def __eq__(self, other):
        """ Returns true if this node and other occupy the same location. """
        return self.location == other.location

    def __hash__(self):
        """ Returns a hash of the location to use as the object hash. """
        return hash(self.location)

    def __lt__(self, other):
        """ Returns a random choice between self and other. """
        return np.random.choice([self, other])


class Robot(object):
    """
    This is a class for the Robot (or Micromouse). It contains path planning
    techniques, and map generation/ knowledge gathering. The next_move method
    is the main runner, called by the tester class.

    Attributes:
        Basic info:
        num_moves (int): The number of moves the robot has performed since the
            last reset
        score [float, float]: The number of moves for each run
        location [int, int]: The current x, y location of the robot
        heading (str): The current heading of the robot

        maze_dim (int): The number of squares tall or wide the maze is.
        headings (list): The list of valid headings
        heading_to_sensor_direction_map (dict): Defines the absolute direction
            for a given heading and sensor side.
        max_move (int): The most squares the robot can move in one timestep.

        Graph structure:
        maze_map (2d list): An array of Nodes for each location in the maze
        cur_node (Node): A node in maze_map at the current location
        goal_node (Node): THe Node at the top right of the goal square.

        Drawing parameters:
        grid_width (int): width of a square in pixels.
        origin (int, int): The origin of the maze in the drawing.

        User options:
        random_weights (bool): Randomize the heuristic weights
        writing_data (bool): Write data (score/params) to csv
        wait_for_user (bool): Wait for user input before 2nd run
        do_draw (bool): Enable drawing of the maze and path

        Path Planning:
        frontier (set): A set of nodes that have been seen but not visited.
        visited (set): A set of nodes that have been visited.
        path (list): A list of nodes representing the current chosen path

        search_type (str): Defines which phase of search the robot is in.
        explore_percent (float): Percent of the maze to explore before
            switching to round 2
        weight1_goal_dist (int): Weight to apply to the distance to the goal
            node during the first phase.
        weight1_self_dist (int): Weight to apply to the distance to from the
            current node during the first phase.
        weight1_area_explored (int): Weight to apply to the amount of
            surrounding area explored during the first phase.
        weight2_goal_dist (int): Weight to apply to the distance to the goal
            node during the second phase.
        weight2_self_dist (int): Weight to apply to the distance to from the
            current node during the second phase.
        weight2_area_explored (int): Weight to apply to the amount of
            surrounding area explored during the second phase.
    """

    def __init__(self, maze_dim):
        """
        Set up attributes that the robot will use to learn and navigate the
        maze. Some initial attributes are provided based on common
        information, including the size of the maze the robot is placed in.

        Paramters:
        maze_dim (int): The number of squares tall or wide the maze is.
        """
        self.num_moves = 0
        self.score = [0, 0]

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim

        self.headings = ['up', 'right', 'down', 'left']
        self.heading_to_sensor_direction_map = {
            'up': [(-1, 0), (0, 1), (1, 0)],
            'right': [(0, 1), (1, 0), (0, -1)],
            'down': [(1, 0), (0, -1), (-1, 0)],
            'left': [(0, -1), (-1, 0), (0, 1)]
        }
        self.max_move = 3
        self.maze_map = self.initialize_map()
        self.cur_node = self.get_node(self.location)
        # just one of the goal nodes but it's fine...
        self.goal_node = self.get_node((int(self.maze_dim / 2),
                                        int(self.maze_dim / 2)))

        # User options
        self.random_weights = False  # Randomize the heuristic weights
        self.writing_data = True  # Write data (score/params) to csv
        self.wait_for_user = True  # Wait for user input before 2nd run
        self.do_draw = True  # Enable drawing of the maze and path

        self.init_search_simple()

        # Initialize the drawing functions
        if self.do_draw:
            self.grid_width = 40
            self.origin = (self.grid_width,
                           (self.maze_dim - 1 + 2) * self.grid_width)
            self.initialize_window()
            self.init_draw_frontier()

    def write_parameters_score_to_file(self):
        """
        If enabled, this function writes the current parameters and the run
        score to the csv specified just before the last move is submitted to
        the tester.
        """
        data_file = 'parameters.csv'
        print('writing to {}'.format(data_file))
        data = {
            'dim': self.maze_dim,
            'score': np.round(self.score[0] / 30 + self.score[1], 2),
            'explore_percent': self.explore_percent,
            'w1_goal': self.weight1_goal_dist,
            'w1_self': self.weight1_self_dist,
            'w1_area': self.weight1_area_explored,
            'w2_goal': self.weight2_goal_dist,
            'w2_self': self.weight2_self_dist,
            'w2_area': self.weight2_area_explored
        }
        data = pd.DataFrame([data])
        data.to_csv(data_file, mode='a', header=False)

    # ------------
    # Graph functions
    def initialize_map(self):
        """ returns a list of lists of nodes in the maze. """
        maze_map = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                maze_map.append(Node(i, j))
        return maze_map

    def get_node(self, location):
        """ Takes in a location and returns the Node at that location. """
        return self.maze_map[location[0] * self.maze_dim + location[1]]

    def update_knowledge(self, sensors):
        """
        Takes in sensor information and adds edges to the graph based on this
        information. Each edge added represents a valid single-turn move from
        one node to another.

        Parameters:
        sensors (list): A list of the distance to the wall on the front, left,
        and right of the robot.
        """
        self.cur_node = self.get_node(self.location)
        sensor_directions = self.heading_to_sensor_direction_map[self.heading]

        # First add edges for passageways
        for i in range(len(sensors)):

            dist_to_wall = sensors[i]
            # d1 is the distance from the current node to this node
            for d1 in range(dist_to_wall + 1):
                n1_x = self.location[0] + sensor_directions[i][0] * d1
                n1_y = self.location[1] + sensor_directions[i][1] * d1
                n1 = self.get_node((n1_x, n1_y))

                # Cap edge creation at the max move distance or wall
                furthest_move = min(d1 + 1 + self.max_move, dist_to_wall + 1)
                # d2 is the distance from the current node to the next node
                # for an edge
                for d2 in range(d1 + 1, furthest_move):
                    n2_x = self.location[0] + sensor_directions[i][0] * d2
                    n2_y = self.location[1] + sensor_directions[i][1] * d2
                    n2 = self.get_node((n2_x, n2_y))
                    n1.add_edge(n2)
        pass

    # ------------
    # Planning algorithms
    #   Resources:
    #       https://www.redblobgames.com/pathfinding/a-star/introduction.html
    #
    def init_search_simple(self):
        """
        Set up attributes for the path planning algorithm used. Initializes
        the frontier and visited sets and sets the starting search type as
        well as the heuristic weights for scoring frontier nodes.
        """
        self.frontier = set()
        self.visited = set()
        self.visited.add(self.cur_node)
        self.path = []

        self.search_type = 'find_goal'
        # Best found weights:
        #   [0.5, 1, 3, 0, 2, 3, 0]
        # Percent of the maze to explore before switching to round 2
        self.explore_percent = 0.5
        # Weights for the find goal phase of the first round
        self.weight1_goal_dist = 1
        self.weight1_self_dist = 3
        self.weight1_area_explored = 0
        # Weights for the explore pahse of the first round
        self.weight2_goal_dist = 2
        self.weight2_self_dist = 3
        self.weight2_area_explored = 0

        if (self.random_weights):
            self.explore_percent = np.round(np.random.random(), 1)
            # Weights for the find goal phase of the first round
            max_weight = 4
            self.weight1_goal_dist = 1
            self.weight1_self_dist = 3
            self.weight1_area_explored = 0
            # Weights for the explore pahse of the first round
            self.weight2_goal_dist = np.random.randint(0, max_weight)
            self.weight2_self_dist = np.random.randint(0, max_weight)
            self.weight2_area_explored = 0

    def search_simple(self):
        """
        Implements a modified A* algorithm to navigate towards the goal and
        find the optimal path.
        """

        # Figure out what mode we are in
        if self.search_type == 'find_goal':
            if self.cur_node == self.goal_node:
                self.search_type = 'explore'

        elif self.search_type == 'explore':
            if len(self.visited) >= self.explore_percent * len(self.maze_map):
                self.search_type = 'finish'
                self.path = []
                return 'Reset', 'Reset'

        # All this does is remove the current node from the frontier
        self.visited.add(self.cur_node)
        self.color_node(self.cur_node, 'visited')
        self.frontier -= self.visited

        # Now add new nodes to the frontier
        for edge in self.cur_node.edges:
            if edge not in self.visited:
                self.frontier.add(edge)
                self.color_node(edge, 'frontier')

        # If we don't have a path, choose one
        if len(self.path) == 0:
            if self.search_type == 'finish':
                current_goal = self.goal_node

            else:
                if len(self.frontier) == 0:
                    print('All paths explored')
                    return 0, 0
                # Choose a new node to explore and move towards it
                frontier = [(n, self.score_search_simple(n))
                            for n in self.frontier]
                frontier.sort(key=lambda n: n[1])
                current_goal = frontier[0][0]

            self.path = self.cur_node.get_path_to(current_goal)

            # Draw the path
            for node in self.path[:-1]:
                self.color_node(node, 'path')

            if self.search_type == 'finish':
                print('Shortest path found to the goal is: {}'
                      .format(len(self.path)))
                # Wait for user before performing run two
                if self.wait_for_user:
                    input('Press enter to continue:')
                # Write scores and params to a file on the last move
                if self.writing_data:
                    self.score[1] = len(self.path)
                    self.write_parameters_score_to_file()

        # Now move along that path
        move = self.path[0]
        self.path = self.path[1:]
        return self.move_along_edge(move)

    def score_search_simple(self, node):
        """ 
        Takes in a node on the frontier and scores is so that the next node to move to can be chosen.

        Parameters:
        node (Node): The frontier node to score.

        Returns:
        score (float): The score given to the node.
        """
        score = 0
        if self.search_type == 'find_goal':
            score = (
                self.weight1_goal_dist * node.distance(self.goal_node)
                + self.weight1_self_dist * node.distance(self.cur_node)
                + self.weight1_area_explored * self.area_explored(node)
            )
        elif self.search_type == 'explore':
            score = (
                self.weight2_goal_dist * node.distance(self.goal_node)
                + self.weight2_self_dist * node.distance(self.cur_node)
                + self.weight2_area_explored * self.area_explored(node)
            )

        return score

    def area_explored(self, node):
        """
        A measure of how explored the area of the maze around node is.

        Parameters:
        node (Node): The node around which to measure.

        Returns:
        score (float): A score based on the number of explored nodes and their 
            distance from node. A larger magnitude negative means that the
            area is more exlplored.
        """
        # Max score ~maze_dim for corner node with nothing visited
        score = 0
        for n in self.maze_map:
            if n == node:
                continue
            if n not in self.visited:
                dist = node.distance(n)
                score -= 1 / dist

        return score

    def move_along_edge(self, edge):
        """
        Determines a (rotation, movement) command based on the the move required to go from the current location to the destination defined by the Node edge.

        Parameters:
        edge (Node): The node to move to.

        Returns:
        rotation (int): The amount of rotation required for the move.
        movement (int): The amount of movement required for the move.
        """
        dest = edge.location
        rotation = 0
        movement = 0

        x_dist = dest[0] - self.location[0]
        y_dist = dest[1] - self.location[1]
        if x_dist != 0 and y_dist != 0:
            print('invalid edge: {} to {}'.format(self.cur_node, dest))

        elif y_dist > 0:
            movement = np.abs(y_dist)
            if self.heading == 'up':
                rotation = 0
            elif self.heading == 'right':
                rotation = -90
            elif self.heading == 'down':
                rotation = 0
                movement *= -1
            elif self.heading == 'left':
                rotation = 90

        elif x_dist > 0:
            movement = np.abs(x_dist)
            if self.heading == 'up':
                rotation = 90
            elif self.heading == 'right':
                rotation = 0
            elif self.heading == 'down':
                rotation = -90
            elif self.heading == 'left':
                rotation = 0
                movement *= -1

        elif y_dist < 0:
            movement = np.abs(y_dist)
            if self.heading == 'up':
                rotation = 0
                movement *= -1
            elif self.heading == 'right':
                rotation = 90
            elif self.heading == 'down':
                rotation = 0
            elif self.heading == 'left':
                rotation = -90

        elif x_dist < 0:
            movement = np.abs(x_dist)
            if self.heading == 'up':
                rotation = -90
            elif self.heading == 'right':
                rotation = 0
                movement *= -1
            elif self.heading == 'down':
                rotation = 90
            elif self.heading == 'left':
                rotation = 0

        else:
            print('where am I going? '.format(edge))

        return rotation, movement
    # ------------

    # ------------
    # Drawing functions
    def color_node(self, node, type):
        """
        Applies color to a given node if drawing is enabled.
        """
        if not self.do_draw:
            return
        cell = self.drawn_cells[node]
        if type == 'frontier':
            cell.setFill('yellow')
        if type == 'visited':
            cell.setFill('light grey')
        if type == 'path':
            cell.setFill('plum1')

    def init_draw_frontier(self):
        """
        Initializes the frontier for drawing by creating a list of drawn cells.
        """
        if not hasattr(self, 'drawn_cells'):
            border = 2
            self.drawn_cells = {}
            for node in self.maze_map:
                x = (self.origin[0] + (node.location[0] * self.grid_width) +
                     border)
                y = (self.origin[1] - (node.location[1] * self.grid_width) -
                     border)
                x2 = x + self.grid_width - border
                y2 = y - self.grid_width + border
                rect = Rectangle(Point(x, y), Point(x2, y2))
                rect.draw(self.win)
                rect.setOutline('white')
                rect.setFill('white')
                self.drawn_cells[node] = rect

    def wall_from_point(self, p, direction):
        """
        Returns a line object representing a wall

        Parameters:
            p (tuple): a point on the grid at which to create a wall
                indexed 0,0 as lower left corner
            direction (str): a single character representing the side of
                the cell on which to create a wall

        Returns:
            wall (Line): a graphics module Line object
        """

        # get x,y of the lower left corner of the cell
        x = self.origin[0] + (p[0] * self.grid_width)
        y = self.origin[1] - (p[1] * self.grid_width)

        # get a line object for the requested wall
        bl = Point(x, y)
        tl = Point(x, y - self.grid_width)
        tr = Point(x + self.grid_width, y - self.grid_width)
        br = Point(x + self.grid_width, y)
        if direction == 'u':
            line = Line(tl, tr)
        elif direction == 'r':
            line = Line(tr, br)
        elif direction == 'd':
            line = Line(br, bl)
        elif direction == 'l':
            line = Line(bl, tl)
        else:
            print('bad direction: {}'.format(direction))

        line.setWidth(4)
        return line

    def draw_rob(self):
        """ Draws the robot in the maze at its current location. """
        x = (self.origin[0] + (self.location[0] * self.grid_width) +
             self.grid_width / 2)
        y = (self.origin[1] - (self.location[1] * self.grid_width) -
             self.grid_width / 2)
        if self.heading == 'up':
            p1 = Point(x, y - self.grid_width / 4)
            p2 = Point(x - self.grid_width / 4, y + self.grid_width / 4)
            p3 = Point(x + self.grid_width / 4, y + self.grid_width / 4)
        elif self.heading == 'right':
            p1 = Point(x + self.grid_width / 4, y)
            p2 = Point(x - self.grid_width / 4, y + self.grid_width / 4)
            p3 = Point(x - self.grid_width / 4, y - self.grid_width / 4)
        elif self.heading == 'down':
            p1 = Point(x, y + self.grid_width / 4)
            p2 = Point(x - self.grid_width / 4, y - self.grid_width / 4)
            p3 = Point(x + self.grid_width / 4, y - self.grid_width / 4)
        elif self.heading == 'left':
            p1 = Point(x - self.grid_width / 4, y)
            p2 = Point(x + self.grid_width / 4, y + self.grid_width / 4)
            p3 = Point(x + self.grid_width / 4, y - self.grid_width / 4)
        else:
            print('bad heading: {}'.format(self.heading))

        self.rob_icon.undraw()
        self.rob_icon = Polygon(p1, p2, p3)
        self.rob_icon.setFill('olive drab')
        self.rob_icon.draw(self.win)
        pass

    def initialize_window(self):
        """ Initializes the draw window. """
        s = self.grid_width
        self.win = GraphWin('Maze', (self.maze_dim + 2) * s,
                            (self.maze_dim + 2) * s)

        # Draw outer walls
        bl = Point(self.origin[0], self.origin[1])
        tl = Point(self.origin[0],
                   self.origin[1] - self.maze_dim * s)
        tr = Point(self.origin[0] + self.maze_dim * s,
                   self.origin[1] - self.maze_dim * s)
        br = Point(self.origin[0] + self.maze_dim * s,
                   self.origin[1])
        wall_l = Line(bl, tl)
        wall_t = Line(tl, tr)
        wall_r = Line(tr, br)
        wall_b = Line(br, bl)

        wall_start = self.wall_from_point(self.location, 'r')
        wall_start

        walls = [wall_l, wall_t, wall_r, wall_b, wall_start]
        for wall in walls:
            wall.setWidth(4)
            wall.draw(self.win)

        self.rob_icon = Point(self.origin[0], self.origin[1])
        self.draw_rob()
        pass

    def draw_robot_view(self, sensors):
        """ Uses the sensor data to draw what the robot can see. """
        walls = []

        x, y = self.location[0], self.location[1]
        if self.heading == 'up':
            pt = (x - sensors[0], y)
            walls.append(self.wall_from_point(pt, 'l'))
            pt = (x, y + sensors[1])
            walls.append(self.wall_from_point(pt, 'u'))
            pt = (x + sensors[2], y)
            walls.append(self.wall_from_point(pt, 'r'))

        elif self.heading == 'right':
            pt = (x, y + sensors[0])
            walls.append(self.wall_from_point(pt, 'u'))
            pt = (x + sensors[1], y)
            walls.append(self.wall_from_point(pt, 'r'))
            pt = (x, y - sensors[2])
            walls.append(self.wall_from_point(pt, 'd'))

        elif self.heading == 'down':
            pt = (x + sensors[0], y)
            walls.append(self.wall_from_point(pt, 'r'))
            pt = (self.location[0], y - sensors[1])
            walls.append(self.wall_from_point(pt, 'd'))
            pt = (x - sensors[2], y)
            walls.append(self.wall_from_point(pt, 'l'))

        elif self.heading == 'left':
            pt = (x, y - sensors[0])
            walls.append(self.wall_from_point(pt, 'd'))
            pt = (x - sensors[1], y)
            walls.append(self.wall_from_point(pt, 'l'))
            pt = (x, y + sensors[2])
            walls.append(self.wall_from_point(pt, 'u'))

        else:
            print('bad heading: {}'.format(self.heading))

        for wall in walls:
            wall.draw(self.win)
        pass
    # ------------

    # ------------
    # Update knowledge
    def update_heading_location(self, rotation, movement):
        """
        Updates the robot's heading and location based on the move.

        Parameters:
        rotation (int): The rotation of the next move to be sent in deg.
        movement (int): The number of squares for the next move.
        """
        # Update the heading
        heading_index = self.headings.index(self.heading)
        heading_index = heading_index + int(rotation / 90)
        if heading_index < 0:
            heading_index = len(self.headings) + heading_index
        elif heading_index > len(self.headings) - 1:
            heading_index = heading_index % len(self.headings)
        self.heading = self.headings[heading_index]

        # Update the location
        if self.heading == 'up':
            self.location = [self.location[0], self.location[1] + movement]
        elif self.heading == 'right':
            self.location = [self.location[0] + movement, self.location[1]]
        elif self.heading == 'down':
            self.location = [self.location[0], self.location[1] - movement]
        elif self.heading == 'left':
            self.location = [self.location[0] - movement, self.location[1]]
        pass

    def reset_heading_location(self):
        """ Resets the heading and location to the start. """
        self.location = [0, 0]
        self.heading = 'up'
        pass
    # ------------

    # ------------
    def next_move(self, sensors):
        """
        This function determines the next move that the robot will make.
        Additionally it performs a few more funtions to enable this.
        1. Draw the robot and the maze
        2. Update the graph with sensor data.
        3. Choose the next move.
        4. Update the internal heading and location of the bot

        There are three input methods available:
        tuple - The user inputs a rotation and movement as a tuple.
        arrow - The user can control the bot with the arrow keys.
        search_simple - The robot uses the search_simple alorithm to determine
            it's next move.

        Parameters:
        sensors (list): A list of the distance to the wall on the front, left,
        and right of the robot.

        Returns:
        rotation (int): The amount of rotation required for the move.
        movement (int): The amount of movement required for the move.
        """
        rotations = ['Reset', -90, 0, 90]
        movements = ['Reset', -3, -2, -1, 0, 1, 2, 3]
        method = 'search_simple'

        if self.do_draw:
            self.draw_rob()
            self.draw_robot_view(sensors)
            # self.draw_maze_from_knowledge()

        self.num_moves += 1
        self.update_knowledge(sensors)

        if method == 'tuple':
            rotation, movement = input('enter move (rot, mov): ').split(', ')
            rotation = int(rotation)
            movement = int(movement)

        elif method == 'arrow':
            # print('Use arrow keys to move, q to quit')
            key = self.win.getKey()
            if key == 'q':
                exit('bye')
            elif key in ['r', 'Space']:
                rotation, movement = 'Reset', 'Reset'
            elif key in ['Up', 'w']:
                rotation, movement = 0, 1
            elif key in ['Right', 'd']:
                rotation, movement = 90, 0
            elif key in ['Down', 's']:
                rotation, movement = 0, -1
            elif key in ['Left', 'a']:
                rotation, movement = -90, 0
            else:
                print('invalid key (use arrows or wasd, q to quit)')
                rotation, movement = 0, 0

        elif method == 'search_simple':
            rotation, movement = self.search_simple()

        if rotation not in rotations:
            print('invalid rotation')
            rotation = 0
        if movement not in movements:
            print('invalid movement')
            movement = 0

        if (rotation, movement) == ('Reset', 'Reset'):
            self.reset_heading_location()
            self.score[0] = self.num_moves
            self.num_moves = 0
        else:
            self.update_heading_location(rotation, movement)

        return rotation, movement
