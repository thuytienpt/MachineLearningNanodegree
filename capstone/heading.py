
from enum import Enum

class Heading(object):
    """docstring for Heading"""
    __slots__ = ('location', 'direction')

    def __init__(self, location, direction):
        self.location = location
        self.direction = direction

    def move(self, direction, movement = 1):
        delta = direction.delta()
        return map(lambda i: self.location[i] + delta[i] * movement, range(2))

    def adjust(self, steering, movement = 1):
        direction = self.direction.adjust(steering)
        delta = direction.delta()
        location = self.move(direction, movement)
        return Heading(location, direction)

    def steer(self, direction, movement):
        steering = self.direction.steer(direction)
        if (steering % 4 == 2):
            steering, movement = 0, -movement
        return steering, movement

    def __str__(self):
        print '{} - {}'.format(self.location, self.direction.name)


class Steering(Enum):
    L, S, R = (-1, 0, 1)  # Left, Straight, Right

    def rotate(self):
        return self.value * 90

Delta = [
    [0, 1],  # go north
    [1, 0],  # go east
    [0, -1],  # go south
    [-1, 0],  # go west
]


class Direction(Enum):
    N, E, S, W = range(4)  # North, East, South, West

    def reverse(self):
        return Direction((self.value + 2) % 4)

    def delta(self):
        return Delta[self.value]

    def wall_value(self):
        return 2**self.value

    def adjust(self, steering):
        return Direction((self.value + steering.value) % 4)

    def steer(self, direction):
        diff = direction.value - self.value
        return diff%3 and diff or diff/-3
    # def sensors(self):
