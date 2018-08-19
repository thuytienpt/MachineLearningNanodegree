import numpy as np
from globalvariables import *
from controller import DepthFirstSearch, AStar



class Robot(object):
    def __init__(self, maze_dim):

        self.location = [0, 0]
        self.heading = 'u'
        self.maze_dim = maze_dim
        self.goalbound = [maze_dim/2 - 1, maze_dim/2]
        self.run_trial = 1
        self.optimal_path = list()
        self.controller = DepthFirstSearch(maze_dim)
        # self.controller = AStar(maze_dim)

    def adjust_current_location(self, next_location, next_direction):
        self.location = next_location
        self.heading = next_direction
        # print 'Location: {}  \n Robot has discovered {:4.3f} of the maze.'.format(self.location, self.controller.coverage*self.maze_dim*self.maze_dim/100)


    def steering(self, next_location):
        dir_vector = np.subtract(next_location, self.location)
        steps = abs(sum(dir_vector))
        dir_vector = list(dir_vector/steps)
        # print self.location, next_location, dir_vector
        next_direction = [d for d in dir_move if dir_move[d] == dir_vector][0]
        if next_direction == dir_reverse[self.heading]:
            return 0, -steps, self.heading
        rotation = dir_rotation[self.heading][next_direction]
        return rotation, steps, next_direction

    def next_move(self, sensors):
        if self.check_in_goalbound(self.location):
            self.optimal_path = self.controller.optimal_path
            print self.optimal_path
            self.reset_startState()
            return 'Reset', 'Reset'

        if self.run_trial == 1:
            next_location =  self.exploration_run(sensors)
        else:
            next_location = self.optimization_run()
        rotation, movement, next_direction = self.steering(next_location)
        # if self.run_trial == 2:
        #     self.grid.update_route(self.location, movement, next_direction)

            # if not self.optimal_path:
            #     self.grid.print_route()
        self.adjust_current_location(next_location, next_direction)
        return rotation, movement

    def exploration_run(self, sensors):
        sensors = dict({d:s for d, s in zip(dir_sensors[self.heading], sensors)})
        return self.controller.determite_next_location(sensors)

    def optimization_run(self):
        return self.optimal_path.pop(0)

    def check_in_goalbound(self, location):
        return all((coord in self.goalbound for coord in location))

    def reset_startState(self):
        self.location = [0, 0]
        self.heading = 'u'
        self.run_trial = 2

    # def get_best_route(self):
    #     if self.value_function:
    #         self.optimal_path = self.controller.find_optimal_path()
    #     else:
    #         self.optimal_path = self.controller.reconstruct_path()