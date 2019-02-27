
import numpy as np
from copy import deepcopy
from simulation import Simulation, Color
from heading import Heading, Direction, Steering

MAX_STEP = 3
class Node(object):
    """docstring for Node"""
    __slots__ = ('visited', 'deadend','gScore', 'hScore', 'fScore', 'cost', 'parent', 'adj_cells', 'policy')

    def __init__(self, distance):
        self.visited = False
        self.deadend = False
        self.gScore = 1000
        self.hScore = distance
        self.fScore = 1000
        self.cost = 1000
        self.policy = '*'
        self.parent = None
        self.adj_cells = {Direction(i): [] for i in range(4)}

    def update_gScore(self, gScore, parent):
        if (self.gScore > gScore + 1):
            self.gScore = gScore + 1
            self.fScore = self.gScore + self.hScore
            self.parent = parent
            self.visited = False


    def update_policy(self, cost, policy):
        self.cost, self.policy = cost, policy


    def __str__(self, attr):
        value = dict({
            'deadend': self.deadend,
            'visited': self.visited,
            'hScore': int(self.hScore),
            'parent': self.parent if self.parent else '  None',
            'gScore': self.gScore,
            'cost': int(self.cost),
        }).get(attr)
        if type(value) == int:
            return '{0:4d}'.format(value)
        return '{0:>4s}'.format(value)

class Map(object):
    """docstring for Map"""
    def __init__(self, maze_dim, simulation, limited_step):
        print ('simulation', simulation)
        self.dim = maze_dim
        self.heuristic = True
        self.limited_step = limited_step
        self.simulator = simulation and Simulation(maze_dim) or None
        self.goal = self.goal_area(maze_dim)
        self.grid = [[
            deepcopy(Node(self.distance([x, y], maze_dim))) for x in range(maze_dim)
        ] for y in range(maze_dim)]
        self.current = self._start()
        self.__print_node__('hScore')

    def mark_walls(self, sensors):
        self.mark_visited()
        if max(sensors) == 0:
            return self.mark_deadend()
        adjacents = []
        for i, distance in enumerate(sensors):
            direction = self.current.direction.adjust(Steering(i - 1))
            # print direction, self.read_current().adj_cells[direction]
            if (distance == 0): continue
            opp_direction = direction.reverse()
            opp_distance = 0 if i % 2 else sensors[2 - i]
            for s in range(0, distance + 1):
                next_heading = self.current.adjust(Steering(i - 1), s)
                if (s < distance):
                    self.update_maze(next_heading)
                range_distance = min(distance - s, MAX_STEP)
                self.determite_adj_cells(next_heading, direction, range_distance)
                range_opp_distance = min(opp_distance + s, MAX_STEP)
                self.determite_adj_cells(next_heading, opp_direction, range_opp_distance)


    def determite_adj_cells(self, heading, direction, range_distance):
        # print '\tDetermite_adj_cell: {} - {} '.format(heading.location, direction)
        node = self.read_node(heading.location)
        if not node.adj_cells[direction]:
            node.adj_cells[direction] = map(lambda i: heading.move(direction, i+1), range(range_distance))
            if self.heuristic:
                map(lambda adj_cell: self.read_node(adj_cell).update_gScore(node.gScore, heading.location), node.adj_cells[direction])


    def mark_deadend(self):
        self.read_current().deadend = True

    def mark_visited(self):
        self.read_current().visited = True

    def _start(self):
        location, direction = [0, 0], Direction.N
        self.read_node(location).gScore = 0
        return Heading(location, direction)

    def hit_goal(self):
        return self.current.location in self.goal


    def adjust(self, next_heading, runtime = 1):
        location, direction, movement = next_heading
        self.update_simulator(self.read_current().deadend and Color.DEAD_END or Color.EXPLORED)
        steering, movement = self.current.steer(direction, movement)
        parent = self.current.location
        # gScore = self.read_current().gScore
        self.current = self.current.adjust(Steering(steering), movement)
        self.update_simulator(runtime==1 and Color.CURRENT or Color.OPTIMAL)
        return steering *90, movement

    def read_node(self, location):
        return self.grid[location[0]][location[1]]

    def read_current(self):
        return self.read_node(self.current.location)

    def update_maze(self, heading):
        if self.simulator:
            self.simulator.open_wall(heading.location, heading.direction)
            # print 'update_wall {} {}'.format(heading.location, heading.direction)

    @staticmethod
    def goal_area(maze_dim):
        return [[maze_dim / 2 - 1 + i, maze_dim / 2 - 1 + j]
                     for i in range(2) for j in range(2)]

    @staticmethod
    def distance(location, dim):
        step = 1
        x, y = [
            min(
                abs(dim / 2 - location[i]),
                abs(dim / 2 - 1 - location[i])) for i in range(2)
        ]
        return np.ceil(float(x)) + np.ceil(float(y))



    def __print_node__(self, attr='visited'):
        print '---{}---'.format(attr.upper())
        for i in range(self.dim):
            row = [self.read_node([j, self.dim - 1 - i]) for j in range(self.dim)]
            if (attr != 'heuristic'):
                print map(lambda node: node.__str__(attr), row)
            else:
                line = '_'*12*self.dim
                r1 = map(lambda node: '{:3d} {:7}'.format(int(node.hScore), (node.__str__('parent'))) , row)
                r2 = map(lambda node: '{:3d} {:6} '.format(node.gScore, int(node.fScore)), row)
                print '{}\n|{}\n|{}'.format(line, '|'.join(r1), '|'.join(r2))


    ### Simulation
    def open_wall(self, location, direction):
        if self.simulator:
            self.simulator.open_wall(location, direction)

    def update_simulator(self, color, location=None):
        if self.simulator:
            location = location or self.current.location
            # print '{} - {}'.format(color, location)
            self.simulator.move_turtle(location, color)

    def exit_simulator(self):
        if self.simulator:
            self.simulator.window.exitonclick()


