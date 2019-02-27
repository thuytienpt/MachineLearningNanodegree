import numpy as np
from map import Map
from search import Random, WallFollower, Tremaux, DepthFirstSearch, Heuristic, DynamicProgramming



SIMULATION = 0
SEARCH =  Random
class Robot(object):

    # def __init__(self, maze_dim, search_agent, fn, max_step=1,  simulation = SIMULATION):
    def __init__(self, maze_dim, search_agent = SEARCH, max_step=1, simulation = SIMULATION):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.location = [0, 0]
        self.heading = 'up'
        self.runtime = 1
        self.map = Map(maze_dim, simulation, max_step)
        # self.search_agent = DepthFirstSearch(self.map)
        # self.search_agent = Heuristic(self.map, 'f2')
        # self.search_agent = WallFollower(self.map)
        # self.search_agent = Tremaux(self.map, 'pre')
        self.search_agent = search_agent(self.map)
        self.optimization = DynamicProgramming(self.map)


    def next_move(self, sensors):
        if self.map.hit_goal():
            self.runtime = 2
            self.optimization.find_optimal_path()
            self.map.current = self.map._start()
            return 'Reset', 'Reset'

        if self.runtime == 1:
            next_heading = self.explore(sensors)
        else:
            next_heading = self.exploit()
        rotation, movement =  self.map.adjust(next_heading, self.runtime)

        # print next_heading, rotation, movement, self.map.current.__str__()
        return rotation, movement



    def explore(self, sensors):
        self.map.mark_walls(sensors)
        return self.search_agent.get_successor()


    def exploit(self):
        return self.optimization.path.pop(0)

