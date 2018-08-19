import numpy as np
from math import sqrt
import heapq
import copy

from globalvariables import *
alpha = 0.3
beta = 0.7
rho = 0.5


class Grid(object):
    """docstring for Grid"""
    def __init__(self, maze_dim, initial_value = False):
        self.layout = [[initial_value for x in range(maze_dim)] for _ in range(maze_dim)]

    def __getitem__(self, x, y):
        return self.layout[x][y]

    def __setitem__(self, x, y, item):
        self.layout[x][y] = item


class State(object):
    """docstring for State"""
    def __init__(self, maze_dim):
        self.walls = Grid(maze_dim, 15)
        self.costs = Grid(maze_dim, 999)
        self.current = Heading([0, 0], 'u')

class Heading(object):
    """docstring for Heading"""
    def __init__(self, pos, direction):
        self.pos = pos
        self.direction = direction

class ClassName(object):
    """docstring for ClassName"""
    def __init__(self, arg):
        super(ClassName, self).__init__()
        self.arg = arg


class Location(object):
    def __init__(self, coord, maze_dim):
        self.coord = coord
        self.parent = None
        self.g = 1000
        self.h = self.heuristic(maze_dim)
        self.wall_value = 0
        self.visited = False

    def heuristic(self, maze_dim):
        return int(sum(np.absolute(np.array(self.coord) - (maze_dim-1)/2.)) - 1)

    def f_score(self):
        return self.g + self.h
        # return alpha* self.gScore + beta*(self.heuristic(maze_dim))
        # return alpha* self.g + beta*(self.h() + rho*self.get_angle())

    def update_wall_value(self, wall_value):
        self.wall_value += wall_value - (wall_value & self.wall_value)

    def update_parent(self, parent):
        self.parent = parent.coord

    def valid_movement(self, direction):
        return self.wall_value & dir_wall[direction]

    def move(self, direction, steps = 1):
        return list(np.add(self.coord, np.dot(dir_move[direction], steps)))

class Grid(object):
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.cost_step = 1
        self.maze = [[Location([i, j], maze_dim) for j in range(maze_dim)] for i in range(maze_dim)]
        self.value = [[copy.deepcopy(99) for i in range(maze_dim)] for _ in range(maze_dim)]
        self.policy = [[copy.deepcopy('_') for i in range(maze_dim)] for _ in range(maze_dim)]
        self.route = [[copy.deepcopy('_') for i in range(maze_dim)] for _ in range(maze_dim)]
        self.initial_start_location()

    def print_wall(self):
        for i in range(self.maze_dim):
            row = [self.maze[j][self.maze_dim-1-i].wall_value for j in range(self.maze_dim)]
            print row

    def print_heuristic(self):
        for i in range(self.maze_dim):
            row = [self.maze[j][self.maze_dim-1-i].h for j in range(self.maze_dim)]
            print row

    def print_policy(self):
        for i in range(self.maze_dim):
            row = [self.policy[j][self.maze_dim-1-i] for j in range(self.maze_dim)]
            print '\t', ' , '.join(row)

    def print_value(self):
        for i in range(self.maze_dim):
            row = np.array([self.value[j][self.maze_dim-1-i] for j in range(self.maze_dim)])
            print row

    def initial_start_location(self):
        self.maze[0][0].g = 0
        self.maze[0][0].visited = True

    def print_route(self):
        for i in range(self.maze_dim):
            row = [self.route[j][self.maze_dim-1-i] for j in range(self.maze_dim)]
            print '\t', ' , '.join(row)

    def update_goal(self):
        center = self.maze_dim/2 - 1
        self.route[center][center] = '*'
        self.route[center][center+1] = '*'
        self.route[center+1][center] = '*'
        self.route[center+1][center+1] = '*'

    def update_route(self, current, movement, next_direction):
        dir_vector = dir_move[next_direction]
        steps = 0
        while steps <= movement:
            if all((coord in self.goalbound for coord in current)):
                break
            steps += 1
            x, y = current
            self.route[x][y] = dir_delta[next_direction]
            current = list(np.add(current, dir_vector))


    def get_location(self, coord):
        return self.maze[coord[0]][coord[1]]

    def move(self, coord, direction, steps = 1):
        return list(np.add(coord, np.dot(dir_move[direction], steps)))

    def get_wall_value(self, coord):
        return self.maze[coord[0]][coord[1]].wall_value

    # def update_wall(self, current_coord, sensors):
    #     for (d, s) in sensors:
    #         wall_value = [dir_wall[d], dir_wall[dir_reverse[d]]]
    #         for step in range(s + 1):
    #             x, y = self.move(current_coord, d, step)
    #             self.localization(self.)
    #             if (step > 0):
    #                 wall_value += dir_wall[dir_reverse[d]]
    #             if step < s: wall_value += dir_wall[dir_reverse[d]]
    #                 self.maze[x][y].update_wall_value(dir_reverse[d])
    #             if step < s:
    #                 self.maze[x][y].update_wall_value(d)

    def update_maze(self, location):
        x, y = location.coord
        self.maze[x][y] = location
        # print 'update {} : {}, {}'.format(location.coord, location.g, location.parent.coord)


    def optimum_policy(self):
        change = True
        while change:
            change = False
            for x in range(self.maze_dim):
                for y in range(self.maze_dim):
                    if (x in self.goalbound) and (y in self.goalbound):
                        if self.value[x][y] > 0:
                            change = True
                            self.value[x][y] = 0
                            self.policy[x][y] = '*'
                        continue
                    directions = [d for d in dir_wall if self.get_location([x, y]).is_valid_movement(d)]
                    for direction in directions:
                        x2 , y2 = self.move([x, y], direction)
                        v2 = self.value[x2][y2] + self.cost_step
                        if self.value[x][y] > v2:
                            change = True
                            self.value[x][y] = v2
                            self.policy[x][y] = direction


# class Controller(object):
#     def __init__(self, maze_dim):
#         self.maze_dim = maze_dim
#         self.goalbound = [maze_dim/2 - 1, maze_dim/2]
#         self.grid = Grid(maze_dim)
#         self.current = self.grid.maze[0][0]
#         self.heading = ['u']
#         self.coverage = 0
#         self.hit_goal = False
#         self.value_function = True
#         self.path = list([self.current.coord])
#         self.roaming_path = list()
#         self.deadends = list()
#         self.optimal_path = list()
#         self.open = list(self.current.coord)


#     def update_coverage_ercentile(self):
#         self.coverage += 1./(self.maze_dim**2)*100

#     def adjust_current_location(self, next_location):
#         self.update_coverage_ercentile()
#         next_location.visited = True
#         # print '--->', next_location.coord, self.coverage*(self.maze_dim**2)/100
#         if next_location.parent == None:
#             next_location.parent = self.current
#         self.current = next_location
#         self.grid.update_maze(self.current)
#         self.path.append(self.current.coord)
#         if self.hit_goal:
#             self.roaming_path.append(self.current.coord)
#             # print 'roaming : ', self.roaming_path
#             # print 'path    : ', self.path[-len(self.roaming_path):], '\n'

#     def get_all_neighbours(self, directions):
#         neighbours = list()
#         # self.grid.print_wall()
#         directions = filter(lambda d: self.current.is_valid_movement(d), directions)
#         for direction in directions:
#             # if self.current.is_valid_movement(direction):
#             coord = self.grid.move(self.current.coord, direction)
#             neighbour = self.grid.get_location(coord)
#             if neighbour.visited: continue
#             neighbours.append(neighbour)
#         return neighbours

#     def update_current_state_maze(self, sensors):
#         self.grid.update_wall(self.current.coord, sensors)
#         self.current.wall_value = self.grid.get_wall_value(self.current.coord)

#     def check_in_goalbound(self, coord):
#         return all((i  in self.goalbound for i in coord))

#     def check_continute_roaming(self):
#         if not self.check_in_goalbound(self.current.coord): return
#         print 'hit_goal'
#         if not self.value_function:
#             return self.reconstruct_path()
#         if self.coverage > COVERAGE:
#             print 'hit goal and greater than COVERAGE', self.current.parent.coord
#             return self.find_optimal_path()
#         print 'hit goal but less than COVERAGE', self.current.parent.coord
#         self.initial_roaming_state()
#         self.hit_goal = True

#     def initial_roaming_state(self):
#         self.roaming_path = self.path[:-3:-1]
#         self.path.pop()
#         self.current = self.current.parent
#         neighbours = self.get_all_neighbours(dir_wall)
#         if not neighbours:
#             self.turn_back()
#         else:
#             next_location = neighbours.pop()
#             self.adjust_current_location(next_location)
#         self.roaming_path.append(self.current.coord)
#         # print 'roaming : ', self.roaming_path
#         # print 'path ', self.path



#     def determite_next_location(self, neighbours):
#         if self.hit_goal and self.coverage > COVERAGE:
#             return self.turn_back_roaming()
#         if not neighbours:
#             return self.turn_back()
#         next_location = neighbours.pop()
#         self.adjust_current_location(next_location)
#         self.check_continute_roaming()
#         return self.current.coord

#     def turn_back(self):
#         self.path.pop()
#         next_location = self.path[-1]
#         # print '<---', next_location, self.coverage*(self.maze_dim**2)/100
#         # print 'turn_back : {} -> {}'.format(self.current.coord, next_location)
#         if next_location in self.roaming_path:
#             self.roaming_path.pop()
#             # print 'D_roaming: ', self.roaming_path
#         elif self.hit_goal:
#             self.roaming_path.append(next_location)
#             # print 'A_roaming: ', self.roaming_path
#         # print 'path    : ', self.path[-len(self.roaming_path):], '\n'

#         self.current = self.grid.get_location(self.path[-1])
#         return self.current.coord


#     def turn_back_roaming(self):
#         self.roaming_path.pop()
#         self.current = self.grid.get_location(self.roaming_path[-1])
#         if len(self.roaming_path) == 1:
#             self.find_optimal_path()
#         return self.current.coord


#     def reconstruct_path(self):
#         current = self.current
#         prev = None
#         dir_vector = [0, 0]
#         child_route = []
#         while current.parent:
#             prev, current = current, current.parent
#             # coord = current.coord
#             next_dir_vector = list(np.subtract(current.coord, prev.coord))
#             child_route.append(prev.coord)
#             if (len(child_route) == 3) or (next_dir_vector != dir_vector):
#                 self.optimal_path = [child_route[-1]] + self.optimal_path
#                 dir_vector = next_dir_vector
#                 child_route = []

#     def find_optimal_path(self):
#         current = [0, 0]
#         direction = 'u'
#         steps = 0
#         self.grid.optimum_policy()
#         while not self.check_in_goalbound(current):
#             x, y = current
#             steps += 1
#             next_direction = self.grid.policy[x][y]
#             next_location = list(np.add(current, dir_move[next_direction]))
#             prev_location, current = current, next_location
#             if (steps < 4) and (next_direction == direction):
#                 continue
#             steps = 1
#             direction = next_direction
#             self.optimal_path.append(prev_location)
#         self.optimal_path.append(current)
#         # return self.optimal_path


# class DepthFirstSearch(Controller):
#     def __init__(self, maze_dim):
#         Controller.__init__(self, maze_dim)

#     def determite_next_location(self, sensors):
#         super(DepthFirstSearch, self).update_current_state_maze(sensors)
#         neighbours = super(DepthFirstSearch, self).get_all_neighbours(dir_wall)
#         return super(DepthFirstSearch, self).determite_next_location(neighbours)


# class AStar(Controller):
#     def __init__(self, maze_dim):
#         Controller.__init__(self, maze_dim)
#         self.closedSet = list()


#     def determite_next_location(self, sensors):
#         super(AStar, self).update_current_state_maze(sensors)
#         neighbours = super(AStar, self).get_all_neighbours(dir_wall)
#         neighbours = filter(lambda x: self.filter_possible_neighbours(x), neighbours)
#         neighbours = sorted(neighbours, key = lambda location: location.f_score())
#         print map(lambda x: )
#         return super(AStar, self).determite_next_location(neighbours)

#     def filter_possible_neighbours(self, neighbour):
#         if neighbour.coord in self.path:
#             return False
#         tentative_g = self.current.g + 1
#         if tentative_g < neighbour.g:
#             neighbour.g = tentative_g
#             neighbour.parent = self.current
#             self.grid.update_maze(neighbour)
#         return True

#     def get_best_movement(self, adj_locations):
#         return sorted(adj_locations, key = lambda location: location.f_score())

class Controller(object):
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.maze = [[Location([i, j], maze_dim) for j in range(maze_dim)] for i in range(maze_dim)]
        self.value = [[copy.deepcopy(99) for i in range(maze_dim)] for _ in range(maze_dim)]
        self.policy = [[copy.deepcopy('_') for i in range(maze_dim)] for _ in range(maze_dim)]
        self.coverage = 0
        self.hit_goal = False
        self.value_function = True
        self.path = list([self.current.coord])
        self.roaming_path = list()
        self.deadends = list()
        self.optimal_path = list()
        self.open = list(self.current.coord)
        self.closed = list()

    def localization(self, coord = None):
        if coord: return self.maze[coord[0]][coord[1]]
        return self.maze[self.current.coord[0]][self.current.coord[1]]


    def update_coverage_percentile(self):
        self.coverage += 1./(self.maze_dim**2)*100

    def adjust_current_state(self, next_location):
        self.update_coverage_ercentile()
        next_location.visited = True
        # print '--->', next_location.coord, self.coverage*(self.maze_dim**2)/100
        if next_location.parent == None:
            next_location.parent = self.current
        self.current = next_location
        self.grid.update_maze(self.current)
        self.path.append(self.current.coord)
        if self.hit_goal:
            self.roaming_path.append(self.current.coord)
            # print 'roaming : ', self.roaming_path
            # print 'path    : ', self.path[-len(self.roaming_path):], '\n'

    def get_all_neighbours(self, directions):
        neighbours = list()
        directions = filter(lambda d: self.current.is_valid_movement(d), directions)
        for direction in directions:
            if not self.current.valid_movement(direction): continue
            coord = self.grid.move(self.current.coord, direction)
            neighbour = self.grid.get_location(coord)
            if neighbour.visited: continue
            neighbours.append(neighbour)
        return neighbours

    def update_walls(self, sensors):
        for (d, s) in sensors:
            for steps in range(s + 1):
                x, y = self.current.move(d, step)
                wall_value = ((steps > s) and dir_wall[dir_reverse[d]]) + ((steps < s) and dir_wall[d])
                self.maze[x][y].update_wall_value(wall_value)
        self.current = self.localization()


    def check_in_goalbound(self, coord):
        return all((i  in self.goalbound for i in coord))

    def check_continute_roaming(self):
        if not self.check_in_goalbound(self.current.coord): return
        print 'hit_goal'
        if not self.value_function:
            return self.reconstruct_path()
        if self.coverage > COVERAGE:
            print 'hit goal and greater than COVERAGE', self.current.parent.coord
            return self.find_optimal_path()
        print 'hit goal but less than COVERAGE', self.current.parent.coord
        self.initial_roaming_state()
        self.hit_goal = True

    def initial_roaming_state(self):
        self.roaming_path = self.path[:-3:-1]
        self.path.pop()
        self.current = self.current.parent
        neighbours = self.get_all_neighbours(dir_wall)
        if not neighbours:
            self.turn_back()
        else:
            next_location = neighbours.pop()
            self.adjust_current_location(next_location)
        self.roaming_path.append(self.current.coord)
        # print 'roaming : ', self.roaming_path
        # print 'path ', self.path



    def determite_next_location(self, neighbours):
        if self.hit_goal and self.coverage > COVERAGE:
            return self.turn_back_roaming()
        if not neighbours:
            return self.turn_back()
        next_location = neighbours.pop()
        self.adjust_current_location(next_location)
        self.check_continute_roaming()
        return self.current.coord

    def turn_back(self):
        self.path.pop()
        next_location = self.path[-1]
        # print '<---', next_location, self.coverage*(self.maze_dim**2)/100
        # print 'turn_back : {} -> {}'.format(self.current.coord, next_location)
        if next_location in self.roaming_path:
            self.roaming_path.pop()
            # print 'D_roaming: ', self.roaming_path
        elif self.hit_goal:
            self.roaming_path.append(next_location)
            # print 'A_roaming: ', self.roaming_path
        # print 'path    : ', self.path[-len(self.roaming_path):], '\n'

        self.current = self.grid.get_location(self.path[-1])
        return self.current.coord


    def turn_back_roaming(self):
        self.roaming_path.pop()
        self.current = self.grid.get_location(self.roaming_path[-1])
        if len(self.roaming_path) == 1:
            self.find_optimal_path()
        return self.current.coord


    def reconstruct_path(self):
        current = self.current
        prev = None
        dir_vector = [0, 0]
        child_route = []
        while current.parent:
            prev, current = current, current.parent
            # coord = current.coord
            next_dir_vector = list(np.subtract(current.coord, prev.coord))
            child_route.append(prev.coord)
            if (len(child_route) == 3) or (next_dir_vector != dir_vector):
                self.optimal_path = [child_route[-1]] + self.optimal_path
                dir_vector = next_dir_vector
                child_route = []

    def find_optimal_path(self):
        current = [0, 0]
        direction = 'u'
        steps = 0
        self.grid.optimum_policy()
        while not self.check_in_goalbound(current):
            x, y = current
            steps += 1
            next_direction = self.grid.policy[x][y]
            next_location = list(np.add(current, dir_move[next_direction]))
            prev_location, current = current, next_location
            if (steps < 4) and (next_direction == direction):
                continue
            steps = 1
            direction = next_direction
            self.optimal_path.append(prev_location)
        self.optimal_path.append(current)
        # return self.optimal_path



class DepthFirstSearch(Controller):
    def __init__(self, maze_dim):
        Controller.__init__(self, maze_dim)

    def determite_next_location(self, sensors):
        super(AStar, self).update_current_state_maze(sensors)
        directions = sensors.keys().append(dir_reverse[sensors.keys[1]])
        neighbours = super(AStar, self).get_all_neighbours(directions)
        self.currents

• 4. Expand n, generating all its successors.
– If child is not in CLOSED or OPEN, then
– If child is not a goal, then put them at the end of OPEN in some order.
• 5. If n is a goal node, exit successfully with the solution obtained by tracing back pointers from n to s.





    # def determite_next_location(self, next_location):
    #     if self.current.parent:
    #         print 'Next: {} -> {} \n'.format(self.current.parent.coord, self.current.coord)
    #     self.adjust_current_location(next_location)
    #     self.path.append(self.current.coord)

    #     if self.roaming_path:
    #         self.roaming_path.append(self.current)

    #     if self.check_in_goalbound(self.current.coord):
    #         if not self.value_function:
    #             self.reconstruct_path()
    #             return self.current.coord
    #         self.hit_goal = True
    #         # self.reconstruct_path()
    #         if self.coverage < COVERAGE and self.roaming:
    #             print 'hit goal but less than COVERAGE', self.current.parent.coord
    #             self.roaming_path += [self.current, self.current.parent, self.current.parent.parent]
    #             self.current = self.current.parent.parent
    #             return self.current.coord
    #         print ' hit goal and greater than COVERAGE'
    #         self.find_optimal_path()
    #     return self.current.coord


    # def turn_back(self, path):
    #     # print 'turn_back : {} \n'.format( self.path)
    #     # self.current = self.current.parent
    #     # self.deadends.append(self.path.pop())
    #     self.path.pop()
    #     next_location = self.path[-1]

    #     print 'next_back : {} -> {} \n'.format(self.current.coord, next_location)
    #     self.current = self.grid.get_location(next_location)
    #     if self.roaming_path:
    #         if self.current in self.roaming_path:
    #             self.roaming_path.pop()
    #         else:
    #             self.roaming_path.append(self.current)

    #     return self.current.coord