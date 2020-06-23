import numpy as np
from graphics import *


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.headings = ['up', 'right', 'down', 'left']
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim

        self.do_draw = True
        if self.do_draw:
            self.grid_width = 20
            self.origin = (20, (self.maze_dim - 1 + 2) * self.grid_width)
            self.initialize_window()

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
        print(self.heading, sensors)
        if self.heading == 'up':
            pt = (self.location[0] - sensors[0], self.location[1])
            self.wall_from_point(pt, 'l').draw(self.win)
            pt = (self.location[0], self.location[1] + sensors[1])
            self.wall_from_point(pt, 'u').draw(self.win)
            pt = (self.location[1] + sensors[2], self.location[1])
            self.wall_from_point(pt, 'r').draw(self.win)

        elif self.heading == 'right':
            pt = (self.location[0], self.location[1] + sensors[0])
            self.wall_from_point(pt, 'u').draw(self.win)
            pt = (self.location[0] + sensors[1], self.location[1])
            self.wall_from_point(pt, 'r').draw(self.win)
            pt = (self.location[1], self.location[1] - sensors[2])
            self.wall_from_point(pt, 'd').draw(self.win)

        elif self.heading == 'down':
            pt = (self.location[0] + sensors[0], self.location[1])
            self.wall_from_point(pt, 'r').draw(self.win)
            pt = (self.location[0], self.location[1] - sensors[1])
            self.wall_from_point(pt, 'd').draw(self.win)
            pt = (self.location[1] - sensors[2], self.location[1])
            self.wall_from_point(pt, 'l').draw(self.win)

        elif self.heading == 'left':
            pt = (self.location[0], self.location[1] - sensors[0])
            self.wall_from_point(pt, 'd').draw(self.win)
            pt = (self.location[0] - sensors[1], self.location[1])
            self.wall_from_point(pt, 'l').draw(self.win)
            pt = (self.location[1], self.location[1] + sensors[2])
            self.wall_from_point(pt, 'u').draw(self.win)

        else:
            print('bad heading: {}'.format(self.heading))
        pass

    def update_location_heading(self, rotation, movement):
        # Update the heading
        heading_index = self.headings.index(self.heading)
        heading_index = heading_index + int(rotation / 90)
        if heading_index < 0:
            heading_index = len(self.headings) - 1 + heading_index
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
        self.draw_robot_view(sensors)
        self.draw_rob()

        rotations = [-90, 0, 90]
        movements = [-3, -2, -1, 0, 1, 2, 3]

        rotation, movement = input('enter move (rot, mov): ').split(', ')
        rotation = int(rotation)
        movement = int(movement)
        if rotation not in rotations:
            rotation = 0
        if movement not in movements:
            movement = 0

        self.update_location_heading(rotation, movement)

        return rotation, movement


if __name__ == '__main__':
    rob = Robot(12)
    input('p')
    rob.win.close()
