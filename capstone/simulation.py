from maze import Maze
import turtle
import sys
from enum import Enum
from heading import Direction

class Color(Enum):
    DEAD_END, CURRENT, EXPLORED, OPTIMAL = -1, 0, 1, 2

    def get_value(self):
        if self.value == 0:
            return 'green'
        if self.value == 1:
            return 'black'
        if self.value == 2:
            return 'blue'
        if self.value == -1:
            return 'red'
        return None


class Simulation(object):

    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.sq_size = 30
        self.origin = self.maze_dim * self.sq_size / -2
        self.window = turtle.Screen()
        self.wally = turtle.Turtle()
        self.wally.speed(0)
        self.wally.hideturtle()
        # self.wally.shape('>')
        self.wally.penup()
        self.draw_maze()

    def get_coordinate(self, location, distance=0.5):
        return self.origin + self.sq_size * (
            location[0] + distance), self.origin + self.sq_size * (
                location[1] + distance)

    def draw_maze(self):
        # self.wally.pensize(5)
        for x in range(self.maze_dim + 1):
            self.wally.goto(self.get_coordinate([x, 0], 0))
            self.wally.pendown()
            self.wally.goto(self.get_coordinate([x, self.maze_dim], 0))
            self.wally.penup()
        for y in range(self.maze_dim + 1):
            self.wally.goto(self.get_coordinate([0, y], 0))
            self.wally.pendown()
            self.wally.goto(self.get_coordinate([self.maze_dim, y], 0))
            self.wally.penup()
        self.draw_goal()

    def pen_wall(self, location, radius, color='white'):
        self.wally.pencolor(color)
        x, y = self.get_coordinate(location, 0)
        self.wally.goto(x, y)
        self.wally.setheading(radius)
        self.wally.pendown()
        self.wally.forward(self.sq_size)
        self.wally.penup()

    def draw_goal(self):
        self.pen_wall([self.maze_dim / 2, self.maze_dim / 2], 0)
        self.pen_wall([self.maze_dim / 2, self.maze_dim / 2], 90)
        self.pen_wall([self.maze_dim / 2 - 1, self.maze_dim / 2], 0)
        self.pen_wall([self.maze_dim / 2, self.maze_dim / 2 - 1], 90)

    def open_wall(self, location, direction):
        x, y = location
        if direction == Direction.N:
            self.pen_wall([x, y + 1], 0, 'white')
        elif direction == Direction.E:
            self.pen_wall([x + 1, y], 90, 'white')
        elif direction == Direction.S:
            self.pen_wall([x, y], 0, 'white')
        else:
            self.pen_wall([x, y], 90, 'white')


    def drop_breadcrumb(self, location, color):
        x, y = self.get_coordinate(location)
        self.wally.goto(x, y)
        if color:
            self.wally.dot(color)

    def move_turtle(self, location, color, value=None):
        if color == Color.OPTIMAL:
            self.wally.pensize(2)
            self.draw_line(location, color.get_value())
        self.wally.pensize(3)
        self.drop_breadcrumb(location, color.get_value())

    def draw_line(self, location, color, position=0.5):
        x, y = self.get_coordinate(location)
        self.wally.pencolor(color)
        self.wally.pendown()
        self.wally.goto(x, y)
        self.wally.penup()

    def reset_turtle(self):
        self.drop_breadcrumb([0, 0], 'green')


