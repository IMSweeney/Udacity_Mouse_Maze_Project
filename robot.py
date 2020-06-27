import numpy as np
from queue import Queue
import time
from graphics import *


class Node():
    def __init__(self, x, y):
        self.location = (x, y)
        self.edges = set()  # each edge is a tuple (other node, distance)

    def add_edge(self, other):
        self.edges.add(other)
        other.edges.add(self)

    def wall_between(self, other):
        return other not in self.edges

    def distance(self, other):
        ''' Manhattan distance between this node and another. '''
        return (np.abs(self.location[0] - other.location[0]) +
                np.abs(self.location[1] - other.location[1]))

    # From: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    def get_path_to(self, other):
        if self == other:
            print('This is an empty path')
            return []

        frontier = Queue()
        frontier.put(self)
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
        return str(self.location)

    def __eq__(self, other):
        return self.location == other.location

    def __hash__(self):
        return hash(self.location)

    def __lt__(self, other):
        return np.random.choice([self, other])


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
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
        self.init_search_simple()

        self.do_draw = True
        if self.do_draw:
            self.grid_width = 40
            self.origin = (self.grid_width,
                           (self.maze_dim - 1 + 2) * self.grid_width)
            self.initialize_window()
            self.init_draw_frontier()

    # ------------
    # Graph functions
    def initialize_map(self):
        ''' returns a list of lists of nodes in the maze. '''
        maze_map = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                maze_map.append(Node(i, j))
        return maze_map

    def get_node(self, location):
        return self.maze_map[location[0] * self.maze_dim + location[1]]

    def update_knowledge(self, sensors):
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
        self.frontier = set()
        self.visited = set()
        self.visited.add(self.cur_node)
        self.search_type = 'find_goal'
        self.explore_percent = 0.8
        self.path = []

    def search_simple(self):
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

        # Now move along that path
        move = self.path[0]
        self.path = self.path[1:]
        return self.move_along_edge(move)

    def score_search_simple(self, node):
        score = 0
        if self.search_type == 'find_goal':
            score += node.distance(self.goal_node)
            score += 2.0 * node.distance(self.cur_node)
        elif self.search_type == 'explore':
            # score += node.distance(self.goal_node)
            score += node.distance(self.cur_node)
            # score += 0.5 * self.area_explored(node)

        return score

    def area_explored(self, node):
        # Max score ~maze_dim for corner node with nothing visited
        score = 0
        for n in self.maze_map:
            if n == node:
                continue
            if n not in self.visited:
                dist = node.distance(n)
                score += 1 / dist

        return score

    def move_along_edge(self, edge):
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
        cell = self.drawn_cells[node]
        if type == 'frontier':
            cell.setFill('yellow')
        if type == 'visited':
            cell.setFill('white')

    def init_draw_frontier(self):
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
                rect.setFill(color_rgb(230, 230, 230))
                self.drawn_cells[node] = rect

        # for cell in self.drawn_cells:
        #     node = cell[0]
        #     rect = cell[1]
        #     if node in self.visited:
        #         rect.setFill(color_rgb(230, 230, 230))
        #     elif node in self.frontier:
        #         rect.setFill('yellow')

    def wall_from_point(self, p, direction):
        '''
        Returns a line object representing a wall

            Parameters:
                p (tuple): a point on the grid at which to create a wall
                    indexed 0,0 as lower left corner
                direction (str): a single character representing the side of
                    the cell on which to create a wall

            Returns:
                wall (Line): a graphics module Line object
        '''

        # get x,y of the lower left corner of the cell
        x = self.origin[0] + (p[0] * self.grid_width)
        y = self.origin[1] - (p[1] * self.grid_width)

        # get a line object for the requested wall
        bl = Point(x, y)
        tl = Point(x, y - self.grid_width)
        tr = Point(x + self.grid_width, y - self.grid_width)
        br = Point(x + self.grid_width, y)
        if direction == 'u':
            return Line(tl, tr)
        elif direction == 'r':
            return Line(tr, br)
        elif direction == 'd':
            return Line(br, bl)
        elif direction == 'l':
            return Line(bl, tl)
        else:
            print('bad direction: {}'.format(direction))

    def draw_rob(self):
        ''' Draws the robot in the maze. '''
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
        self.rob_icon.draw(self.win)
        pass

    def initialize_window(self):
        ''' Initializes the draw window. '''
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
        Line(bl, tl).draw(self.win)
        Line(tl, tr).draw(self.win)
        Line(tr, br).draw(self.win)
        Line(br, bl).draw(self.win)

        wall_r = self.wall_from_point(self.location, 'r')
        wall_r.draw(self.win)

        self.rob_icon = Point(self.origin[0], self.origin[1])
        self.draw_rob()
        pass

    def draw_robot_view(self, sensors):
        ''' Uses the sensor data to draw what the robot can see. '''
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
        ''' Updates the robot's heading and location based on the move. '''
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
        ''' Resets the heading and location to the start. '''
        self.location = [0, 0]
        self.heading = 'up'
        pass
    # ------------

    # ------------
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        rotations = ['Reset', -90, 0, 90]
        movements = ['Reset', -3, -2, -1, 0, 1, 2, 3]
        method = 'search_simple'

        self.draw_rob()
        self.draw_robot_view(sensors)
        # self.draw_maze_from_knowledge()

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
            time.sleep(0.05)
            # key = self.win.getKey()
            # if key == 'q':
            #     exit('bye')

        if rotation not in rotations:
            print('invalid rotation')
            rotation = 0
        if movement not in movements:
            print('invalid movement')
            movement = 0

        # print({'move_type': self.search_type, 'sensors': sensors})

        if (rotation, movement) == ('Reset', 'Reset'):
            self.reset_heading_location()
        else:
            self.update_heading_location(rotation, movement)

        # print('Attempting Move: {}, {}'.format(rotation, movement))
        return rotation, movement


if __name__ == '__main__':
    rob = Robot(12)
    rob.win.close()
