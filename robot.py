import numpy as np
from queue import Queue, PriorityQueue
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


class Node_LPA(Node):
    pass


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
        self.maze_map = self.initialize_map()
        self.cur_node = self.get_node(self.location)
        # just one of the goal nodes but it's fine...
        self.goal_node = self.get_node((int(self.maze_dim / 2),
                                        int(self.maze_dim / 2)))

        self.do_draw = True
        if self.do_draw:
            self.grid_width = 20
            self.origin = (20, (self.maze_dim - 1 + 2) * self.grid_width)
            self.initialize_window()

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
            dist = min(sensors[i], 3)  # Cap edge creation at 3 distance
            while dist > 0:
                other_x = self.location[0] + sensor_directions[i][0] * dist
                other_y = self.location[1] + sensor_directions[i][1] * dist
                other = self.get_node((other_x, other_y))
                self.cur_node.add_edge(other)
                dist -= 1
        pass

    # Pretty much draws walls everywhere since almost all of the edges are
    # blank
    def draw_maze_from_knowledge(self):
        # self.initialize_window()
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                for k, offset in {'u': (0, 1), 'r': (1, 0),
                                  'd': (0, -1), 'l': (-1, 0)}.items():
                    node = self.get_node((i, j))
                    try:
                        other = self.get_node((i + offset[0], j + offset[1]))
                        if node.wall_between(other):
                            wall = self.wall_from_point(node.location, k)
                            wall.draw(self.win)
                    except IndexError:
                        pass
        return

    # ------------
    # Planning algorithms
    #   Resources:
    #       https://www.redblobgames.com/pathfinding/a-star/introduction.html
    #
    def init_search_simple(self):
        self.frontier = set()
        self.visited = set()
        self.visited.add(self.cur_node)

    def search_simple(self):
        # If the frontier does not yet exist, initialize it
        try:
            if self.frontier:
                pass
        except AttributeError:
            self.init_search_simple()
        # self.recalculate_frontier()

        # Check for goal reached
        if self.cur_node == self.goal_node:
            return 'Reset', 'Reset'

        # All this does is remove the current node from the frontier
        self.visited.add(self.cur_node)
        self.frontier -= self.visited

        # Now add new nodes to the frontier
        for edge in self.cur_node.edges:
            if edge not in self.visited:
                self.frontier.add(edge)

        # Choose a new node to explore and move towards it
        frontier = [(n, self.score_search_simple(n)) for n in self.frontier]
        frontier.sort(key=lambda n: n[1])
        current_goal = frontier[0][0]
        print('Current Goal: {}'.format(current_goal))
        path = self.cur_node.get_path_to(current_goal)

        return self.move_along_edge(path[0])

    def score_search_simple(self, node):
        score = 0
        score += node.distance(self.goal_node)
        # score += node.distance(self.cur_node)
        return score

    def search_best_neighbor(self):
        rotation, movement = 0, 0
        edges = self.cur_node.edges
        best_score = self.maze_dim
        best_edge = list(edges)[0]
        for edge in edges:
            score = self.score_edge_greedy(edge)
            if score < best_score:
                best_score = score
                best_edge = edge

        rotation, movement = self.move_along_edge(best_edge)

        return rotation, movement

    def score_edge_greedy(self, edge):
        return self.goal_node.distance(edge)

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
    def wall_from_point(self, p, direction):
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

        print({'loc': self.cur_node, 'heading': self.heading,
               'sensors': sensors, 'edges': self.cur_node.edges})

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

        elif method == 'search_best_neighbor':
            rotation, movement = self.search_best_neighbor()
            key = self.win.getKey()
            if key == 'q':
                exit('bye')

        elif method == 'search_simple':
            rotation, movement = self.search_simple()
            # key = self.win.getKey()
            # if key == 'q':
            #     exit('bye')

        if rotation not in rotations:
            print('invalid rotation')
            rotation = 0
        if movement not in movements:
            print('invalid movement')
            movement = 0

        if (rotation, movement) == ('Reset', 'Reset'):
            self.reset_heading_location()
        else:
            self.update_heading_location(rotation, movement)

        print('Attempting Move: {}, {}'.format(rotation, movement))
        return rotation, movement


if __name__ == '__main__':
    rob = Robot(12)
    rob.win.close()
