import random
import numpy as np
from heading import Heading, Direction, Steering

LIMITED_STEP = 1
class Search(object):
    """docstring for Search"""

    def __init__(self, _map):
        self.map = _map
        self.prev = None

    def get_legal_neighbours(self):
        adj_cells = self.map.read_current().adj_cells
        possible_directions = filter(lambda direction: adj_cells[direction], adj_cells)
        # possible_directions = filter(lambda direction: direction!= self.map.current.direction.reverse(), possible_directions)

        neighbours = reduce(lambda l, d: l + map(lambda i: [adj_cells[d][i], d, i+1], range(min(self.map.limited_step, len(adj_cells[d])))), possible_directions, [])
        return neighbours

    def get_successor(self):
        # print neighbours
        neighbours = self.get_legal_neighbours()
        print neighbours
        if neighbours:
            return self.policy(neighbours)
        return self.turn_back()

    def turn_back(self):
        raise NotImplementedError()

    def policy(self):
        raise NotImplementedError()

    def _backward(self, next_heading):
        return [self.map.current.location, next_heading[1].reverse(), next_heading[2]]


class Random(Search):
    """docstring for Random"""

    def __init__(self, _map):
        super(Random, self).__init__(_map)
        self.prev = None

    def policy(self, neighbours):
        next_heading = random.choice(neighbours)
        self.prev = self._backward(next_heading)
        return next_heading

    def turn_back(self):
        return self.prev

class WallFollower(Random):
    """docstring for WallFollower"""
    def __init__(self, _map):
        super(WallFollower, self).__init__(_map)

    def get_legal_neighbours(self):
        neighbours = super(WallFollower, self).get_legal_neighbours()
        # print neighbours
        return filter(lambda neighbour: (neighbour[1] != self.map.current.direction.reverse()) and not self.map.read_node(neighbour[0]).deadend, neighbours)

    def policy(self, neighbours):
        # Right hand: R, S, L
        priority_directions = map(lambda i: self.map.current.direction.adjust(Steering(i)), [1, 0, -1])
        neighbours = sorted(neighbours, key = lambda neighbour: priority_directions.index(neighbour[1]))

        print priority_directions, neighbours
        next_heading = neighbours[0]
        self.prev = self._backward(next_heading)
        return next_heading


class DepthFirstSearch(Search):
    """docstring for DepthFirstSearch"""
    def __init__(self, _map):
        super(DepthFirstSearch, self).__init__(_map)
        self.path = list()

    def get_legal_neighbours(self):
        neighbours = super(DepthFirstSearch, self).get_legal_neighbours()
        return filter(lambda neighbour: not self.map.read_node(neighbour[0]).visited, neighbours)

    def policy(self, neighbours):
        self.path += [[self.map.current.location, next_heading[1].reverse(), next_heading[2]]]
        return next_heading

    def turn_back(self):
        print '\t Turnback {}'.format(self.path[-1])
        next_heading = self.path.pop()
        return next_heading


class Heuristic(DepthFirstSearch):
    """docstring for Heuristic"""
    def __init__(self, _map):
        super(Heuristic, self).__init__(_map)
        self.heuristic = (fn =='f1') and self.fScore_1 or ((fn =='f2') and self.fScore_2 or self.fScore_3)

    def policy(self, neighbours):
        # self.map.__print_node__('heuristic')
        print map(lambda neighbour: [neighbour[0], self.heuristic(neighbour)], neighbours)

        next_heading = min(neighbours, key = self.heuristic)
        self.path += [self._backward(next_heading)]
        return next_heading

    def fScore_1(self, cell):
        node = self.map.read_node(cell[0])
        return node.gScore + node.hScore

    def fScore_2(self, cell):
        node = self.map.read_current()
        v1 = np.subtract([self.map.dim, self.map.dim], self.map.current.location)
        v2 = np.subtract([self.map.dim, self.map.dim],cell[0])
        alpha = self.angle(v1, v2)
        # return alpha
        return node.gScore + node.hScore*alpha

    def fScore_3(self, cell):
        node = self.map.read_node(cell[0])
        parent = self.map.read_node(node.parent)
        return node.gScore + node.hScore + parent.hScore + parent.gScore

    @staticmethod
    def angle(a, b = [1, 0]):
        cosine_angle = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
        angle = np.arccos(cosine_angle)
        return angle
        # return np.degrees(angle)

class DynamicProgramming(object):
    """docstring for DynamicProgramming"""
    def __init__(self, _map):
        self.map = _map
        self.change = True
        self.path = []

    def dynamic_programming(self):
        _lst_headings = [
            Heading([x, y], Direction(0))
            for x in range(self.map.dim) for y in range(self.map.dim)
        ]
        while self.change:
            self.change = False
            map(lambda heading: self.value_function(heading), _lst_headings)
            # self.map.__print_node__('cost')

    def value_function(self, heading):
        node = self.map.read_node(heading.location)
        if heading.location in self.map.goal:
            if node.cost > 0:
                self.change = True
                return node.update_policy(0, ('*', 0))
        adj_cells = node.adj_cells
        adjacents = reduce(lambda l, d: l + map(lambda i: [node.adj_cells[d][i], d, i+1], range(len(node.adj_cells[d]))), node.adj_cells, [])
        for location, direction, step in adjacents:
            v2 = self.map.read_node(location).cost + 1
            if node.cost > v2:
                self.change = True
                return node.update_policy(v2, (location, direction, step))

    def find_optimal_path(self):
        self.dynamic_programming()
        current = Heading([0, 0], Direction(0))
        while not current.location in self.map.goal:
            location, direction, step = self.map.read_node(current.location).policy
            self.path += [[location, direction, step]]
            current = Heading(location, direction)
        print 'Optimal Moves = {}'.format(len(self.path))
        print 'Optimal Path = {}'.format(self.path)
