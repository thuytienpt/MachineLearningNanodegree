import numpy as np
from math import sqrt
import heapq
import copy

from globalvariables import *

alpha = 0.3
beta = 0.7
rho = 0.5
# COVERAGE = 50
class Location(object):
    def __init__(self, coord):
        self.coord = coord
        self.parent = None
        self.gScore = 1000
        self.neighbours = list()
        self.distance = sqrt(sum(np.power(self.coord, 2)))

    def heuristic(self, maze_dim):
        return sum(np.absolute(np.subtract(self.coord, (maze_dim-1)/2.))) -1

    def get_angle(self):
        return 45 - np.angle(self.coord[0] + 1j*self.coord[1],deg = True)

    def f_score(self, maze_dim):
        return self.gScore + self.heuristic(maze_dim)
        # return alpha* self.gScore + beta*(self.heuristic(maze_dim))
        # return alpha* self.gScore + beta*(self.heuristic(maze_dim) + rho*self.get_angle())



class AStar(object):
#     """docstring for A_star"""
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.current = Location([0, 0])
        self.current.parent = None
        self.current.gScore = 0
        self.openSet = list()
        self.closedSet = list()
        self.deadends = list()
        self.visited = dict({tuple(self.current.coord): self.current})
        self.path = []
        self.coverage = 0.0
        self.maze = [[copy.deepcopy('_') for i in range(maze_dim)] for _ in range(maze_dim)]
        self.hit_goal = False
        self.roaming_path = list()

    def update_maze(self, symbol):
        x, y = self.current.coord
        self.maze[x][y] = symbol

    def print_maze(self):
        for i in range(self.maze_dim):
            row = [self.maze[j][self.maze_dim-1-i] for j in range(self.maze_dim)]
            print '\t', ' , '.join(row)

    def get_location(self, coord):
        for remaining_coord in self.openSet:
            if remaining_coord == coord:
                return self.visited[tuple(coord)]
        return Location(coord)

    def get_all_possible_neighbours(self, neighbours):
        adj_locations = []
        for coord, location in self.visited.items():
            if location.parent and location.parent.coord == self.current.coord:
                if not location.coord in neighbours:
                    neighbours.append(location.coord)
        # print  'neighbours: ', self.current.coord,'-->', neighbours
        for coord in neighbours:
            if coord in self.closedSet or coord in self.deadends:
                # print coord, ' in closedSet'
                continue
            if coord in self.roaming_path:
                continue
            tentative_gScore = self.current.gScore + 1
            location = self.get_location(coord)
            if tentative_gScore < location.gScore:
                location.gScore = tentative_gScore
                location.parent = self.current
            adj_locations.append(location)
            self.visited.update({tuple(location.coord): location})
        return adj_locations

    def determite_next_location(self, neighbours):
        adj_locations = self.get_all_possible_neighbours(neighbours)

        if adj_locations:
            next_location = min(adj_locations, key = lambda location: location.f_score(self.maze_dim))

            adj_locations.remove(next_location)
            self.current = next_location
            self.closedSet.append(self.current.coord)
            self.coverage += 1./(self.maze_dim**2)*100
            self.openSet += [location.coord for location in adj_locations]
            self.update_maze('*')
        else:
            print 'turn back ', self.deadends
            self.turn_back(self.closedSet)

        # print 'closedSet ' , self.closedSet
        if self.check_in_goalbound():
            self.hit_goal = True
            self.reconstruct_path()
            print 'Coveraged = {:4.3f} %'.format(self.coverage)
        return self.current.coord

    def check_in_goalbound(self):
        return all((i in self.goalbound for i in self.current.coord))

    def roaming(self, neighbours):
        if self.roaming_path and self.coverage > COVERAGE:
            return self.turn_back(self.roaming_path)

        if not self.roaming_path:
            self.roaming_path.append(self.closedSet.pop())
            self.current = self.visited[tuple(self.closedSet[-1])]
            self.roaming_path.append(self.current.coord)
            return self.current.coord

        adj_locations = self.get_all_possible_neighbours(neighbours)
        # adj_locations = filter(lambda location: location.coord not in self.roaming_path, adj_locations)
        if adj_locations:
            self.current = adj_locations[-1]
            adj_locations.pop()
            self.coverage += 1./(self.maze_dim**2)*100
            self.openSet += [location.coord for location in adj_locations]
            self.update_maze('*')
            print ' --> ', self.current.coord
            self.roaming_path.append(self.current.coord)
        else:
            if self.roaming_path[-1] == self.closedSet[-1]:
                self.turn_back(self.closedSet, False)
                self.roaming_path.append(self.current.coord)
            else:
                # self.deadends.append(self.current.coord)
                return self.turn_back(self.roaming_path)
            print self.closedSet

        return self.current.coord


    def turn_back(self, set_location, update_deadend = True):
        # self.deadends.append(self.current.coord)
        # print '\n turn back goal', self.roaming_path
        if update_deadend:
            self.deadends.append(self.current.coord)
            self.update_maze('D')
        if len(set_location) > 1:
            set_location.pop()
        self.update_maze('_')
        self.current = self.visited[tuple(set_location[-1])]
        return self.current.coord



    def reconstruct_path(self):
        curr = self.current
        while  curr:
            coord = curr.coord
            self.path = [coord] + self.path
            curr = self.visited[tuple(coord)].parent

class DepthFirstSearch(Controller):
    def __init__(self, maze_dim):
        Controller.__init__(self.maze_dim)
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.current = Location([0, 0])
        self.deadends = list()
        self.closedSet = list()
        self.path = list()
        self.visited = dict({tuple(self.current.coord): self.current})
        self.coverage = 0

    def get_all_neighbours(self, neighbours):
        for coord in neighbours:
            if coord in self.current.neighbours:
                continue
            self.current.neighbours.append(coord)
        # self.current.neighbours = [list(neighbour) for neighbour in neighbours[idx]]

    def determite_next_location(self, neighbours):
        if neighbours:
            self.get_all_neighbours(neighbours)
        next_location = None
        while len(self.current.neighbours):
            coord = list(self.current.neighbours.pop())
            if coord in self.closedSet or coord in self.deadends:
                continue
            else:
                next_location = Location(coord)
                next_location.parent = self.current
                break

        self.visited.update({tuple(self.current.coord): self.current})
        if next_location:
            self.current = next_location
            self.closedSet.append(self.current.coord)
            self.coverage += 1.
        else:
            self.deadends.append(self.current.coord)
            self.closedSet.pop()
            self.current = self.visited[tuple(self.closedSet[-1])]

        if self.check_in_goalbound():
            self.coverage = self.coverage/(self.maze_dim**2)*100
            self.visited.update({tuple(self.current.coord): self.current})
            self.reconstruct_path()
            print 'Coveraged = {:4.3f} %'.format(self.coverage)
            print 'path', self.path
        return self.current.coord

    def check_in_goalbound(self):
        return all((i in self.goalbound for i in self.current.coord))

    def reconstruct_path(self):
        curr = self.current
        while  curr:
            coord = curr.coord
            self.path = [coord] + self.path
            curr = self.visited[tuple(coord)].parent




class DepthFirstSearch:
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.current = Location([0, 0])
        self.deadends = list()
        self.closedSet = list()
        self.path = list()
        self.visited = dict({tuple(self.current.coord): self.current})
        self.coverage = 0

    def get_all_neighbours(self, neighbours):
        for coord in neighbours:
            if coord in self.current.neighbours:
                continue
            self.current.neighbours.append(coord)
        # self.current.neighbours = [list(neighbour) for neighbour in neighbours[idx]]

    def determite_next_location(self, neighbours):
        if neighbours:
            self.get_all_neighbours(neighbours)
        next_location = None
        while len(self.current.neighbours):
            coord = list(self.current.neighbours.pop())
            if coord in self.closedSet or coord in self.deadends:
                continue
            else:
                next_location = Location(coord)
                next_location.parent = self.current
                break

        self.visited.update({tuple(self.current.coord): self.current})
        if next_location:
            self.current = next_location
            self.closedSet.append(self.current.coord)
            self.coverage += 1.
        else:
            self.deadends.append(self.current.coord)
            self.closedSet.pop()
            self.current = self.visited[tuple(self.closedSet[-1])]

        if self.check_in_goalbound():
            self.coverage = self.coverage/(self.maze_dim**2)*100
            self.visited.update({tuple(self.current.coord): self.current})
            self.reconstruct_path()
            print 'Coveraged = {:4.3f} %'.format(self.coverage)
            print 'path', self.path
        return self.current.coord

    def check_in_goalbound(self):
        return all((i in self.goalbound for i in self.current.coord))

    def reconstruct_path(self):
        curr = self.current
        while  curr:
            coord = curr.coord
            self.path = [coord] + self.path
            curr = self.visited[tuple(coord)].parent


