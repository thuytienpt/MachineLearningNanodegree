import pandas as pd
import numpy as np
import logging
from maze import Maze
from robot import Robot
import sys
from collections import OrderedDict
from search import DepthFirstSearch, Heuristic


# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

# test and score parameters
max_time = 1000
train_score_mult = 1/30.

# def print_usage():
#     options = OrderedDict({
#         'filename': '[1, 2, 3]',
#         'Algorithm': '[random, dfs, heuristic]',
#         'Max_steps': '[1, 3] (optional, 1 is default)',
#         'heuristic': '[1, 2, 3] (optional, None is default)',
#         'hscore' : '[1, 3] (optional, 1 is default)'
#         'Trials': 'number of trial tests (optional, 10 is default)'
#         })
#     print 'Usage: python test.py filename Algorithm Max_steps heuristic hscore trials'
#     print 'Option: '
#     for opt in options:
#         print '\t{} : {}'.format(opt, options[opt])




def run(filename, search_agent, max_step, fn = None):
    testmaze = Maze(filename)
    testrobot = Robot(testmaze.dim, search_agent, fn, max_step)
    summary = dict({
        'Maze': filename,
        'Search': (search_agent == DepthFirstSearch) and 'dfs'  or 'heuristic',
        'Fn': fn,
        'Max_steps': max_step,
        'Goal': 'No',
        'Moves_1': np.nan,
        'Coverage': np.nan,
        'Moves_2': np.nan,
        'Score': np.nan
    })


    runtimes = []
    total_time = 0
    for run in range(2):
        logging.info("Starting run {}.".format(run))

        # Set the robot in the start position. Note that robot position
        # parameters are independent of the robot itself.
        robot_pos = {'location': [0, 0], 'heading': 'up'}

        run_active = True
        hit_goal = False
        while run_active:
            # check for end of time
            total_time += 1
            if total_time > max_time:
                run_active = False
                logging.info("Allotted time exceeded.")
                break
            if run == 0:
                logging.info('\n{}--{}'.format(total_time, robot_pos['location']))


            # provide robot with sensor information, get actions
            sensing = [testmaze.dist_to_wall(robot_pos['location'], heading)
                       for heading in dir_sensors[robot_pos['heading']]]
            rotation, movement = testrobot.next_move(sensing)
            if (rotation, movement) == ('Reset', 'Reset'):
                if run == 0 and hit_goal:
                    run_active = False
                    # summary = testrobot.summarize()
                    runtimes.append(total_time)
                    logging.info( "Ending first run. Starting next run.")
                    break
                elif run == 0 and not hit_goal:
                    logging.info( "Cannot reset - robot has not hit goal yet.")
                    continue
                else:
                    logging.info( "Cannot reset on runs after the first.")
                    continue

            # perform rotation
            if rotation == -90:
                robot_pos['heading'] = dir_sensors[robot_pos['heading']][0]
            elif rotation == 90:
                robot_pos['heading'] = dir_sensors[robot_pos['heading']][2]
            elif rotation == 0:
                pass
            else:
                logging.info( "Invalid rotation value, no rotation performed.")

            # perform movement
            if abs(movement) > 3:
                print "Movement limited to three squares in a turn."
            movement = max(min(int(movement), 3), -3) # fix to range [-3, 3]
            while movement:
                if movement > 0:
                    if testmaze.is_permissible(robot_pos['location'], robot_pos['heading']):
                        robot_pos['location'][0] += dir_move[robot_pos['heading']][0]
                        robot_pos['location'][1] += dir_move[robot_pos['heading']][1]
                        movement -= 1
                    else:
                        logging.info( "Movement stopped by wall.")
                        movement = 0
                else:
                    rev_heading = dir_reverse[robot_pos['heading']]
                    if testmaze.is_permissible(robot_pos['location'], rev_heading):
                        robot_pos['location'][0] += dir_move[rev_heading][0]
                        robot_pos['location'][1] += dir_move[rev_heading][1]
                        movement += 1
                    else:
                        logging.info( "Movement stopped by wall.")
                        movement = 0

            # check for goal entered
            goal_bounds = [testmaze.dim/2 - 1, testmaze.dim/2]
            if robot_pos['location'][0] in goal_bounds and robot_pos['location'][1] in goal_bounds:
                hit_goal = True
                if run != 0:
                    runtimes.append(total_time - sum(runtimes))
                    run_active = False
                    logging.info( "Goal found; run {} completed!".format(run))
    if len(runtimes) == 2:
        # print "Task complete! Score: {:4.3f}".format(runtimes[1] + train_score_mult*runtimes[0])
        score = round(runtimes[1] + train_score_mult*runtimes[0], 3)
        summary.update({'Goal': 'Yes', 'Moves_1': runtimes[0], 'Moves_2': runtimes[1], 'Score': score})
    return summary


# def  test(filename_idx, search_agent, max_step, heuristic, fn):

#     table =  pd.DataFrame()
#     filename = 'test_maze_0{}.txt'.format(filename_idx)

#     for i in range(trial):
#         result = run(filename, algorithm, max_step, heuristic)
#         table = table.append(result, ignore_index = True)
#     table = table[['Goal', 'Moves_1', 'Coverage', 'Path_Length', 'Moves_2', 'Score']]
#     hit_goal_percent = '{}%'.format((table['Goal']=='Yes').sum()*1./trial * 100)
#     table = table.append(table.mean().round(3), ignore_index = True)
#     table.loc[trial, 'Goal'] = hit_goal_percent
#     table.rename(index={trial:'Average'}, inplace=True)
#     return table

def test_all(trial):
    table = pd.DataFrame()
    col_maze, col_algo, col_step, col_heuristic = list(), list(), list(), list()
    for i in range(3):
        for h in [None, 'Yes']:
            for algorithm in  ['random', 'counter', 'dfs']:
                if h and algorithm == 'random':
                    continue
                for s in [1, 3]:
                    table_test = test(i+1, algorithm, s, None, trial)
                    print '\n Maze {} - {} - {} steps - {}'.format(i+1, algorithm, s, h)
                    print table_test.loc['Average']
                    table = table.append(table_test.loc['Average'], ignore_index = True)
                    col_maze += [str(i+1)]
                    col_algo += [algorithm]
                    col_step += [s]
                    col_heuristic += ['No' if h == None else 'Yes']
                table['Step'] = pd.Series(col_step).values
            table['Algorithm'] = pd.Series(col_algo).values
        table['Heuristic'] = pd.Series(col_heuristic).values
    table['Maze'] = pd.Series(col_maze).values
    table = table[['Maze', 'Algorithm', 'Heuristic', 'Step', 'Goal', 'Moves_1', 'Coverage', 'Path_Length', 'Moves_2', 'Score']]
    log_file = './log_csv/summarize_{}.csv'.format(trial)
    table.to_csv(log_file, na_rep='n/a')
    return table

def test_max_step(algorithm, filename_idx):
    table = pd.DataFrame()
    for i in [1, 3]:
        table_test = test(filename_idx, algorithm, max_step)
        table = table.append(table_test.mean(), ignore_index = True)
        table = pd.concat(table, pd.DataFrame.from_dict({'Step': i}), axis = 1)
    table.rename(index={0:'1_step', 1: '3_step'}, inplace=True)

        # print table
    return table

if __name__ == '__main__':
    _test_case = [
        {'maze': 'test_maze_01.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_01.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_01.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_01.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_01.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_02.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_02.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_02.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_02.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_02.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_03.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_03.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_03.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_03.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_03.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_04.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_04.txt', 'max_step': 1, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_04.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_04.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_04.txt', 'max_step': 1, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_01.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_01.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_01.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_01.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_01.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_02.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_02.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_02.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_02.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_02.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_03.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_03.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_03.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_03.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_03.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f3'},
        {'maze': 'test_maze_04.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'pre'},
        {'maze': 'test_maze_04.txt', 'max_step': 3, 'search': DepthFirstSearch, 'fn': 'post'},
        {'maze': 'test_maze_04.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f1'},
        {'maze': 'test_maze_04.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f2'},
        {'maze': 'test_maze_04.txt', 'max_step': 3, 'search': Heuristic, 'fn': 'f3'},
    ]
    table =  pd.DataFrame()
    for test_case in _test_case:
        result = run(test_case['maze'], test_case['search'], test_case['max_step'], test_case['fn'])
        table = table.append(result, ignore_index = True)
    table = table[['Maze', 'Search', 'Fn', 'Max_steps', 'Goal', 'Moves_1', 'Moves_2', 'Score', 'Coverage']]
    log_file = './log_file/test_1.csv'
    table.to_csv(log_file, na_rep='n/a')
    # return table

    # try:
    #     filename_idx = int(sys.argv[1]), int(sys.argv[2])
    #     algorithm = str(sys.argv[3])
    #     if algorithm == 'heuristic':
    #         max_step, heuristic = int(sys.argv[4]), str(sys.argv[5])
    #     except:
    #         max_step, heuristic = 1, None
    #     logging.debug('trial = {}, algorithm = {}, max_step = {}, heuristic = {},'.format(trial, algorithm, max_step, heuristic))
    #     log_file = '_heuristic_' if( heuristic is not None) else ''
    #     log_file = './log_csv/{}{}_{}steps_M0{}_{}trials.csv'.format(algorithm, log_file, max_step, filename_idx, trial)
    #     table = test(filename_idx, algorithm, max_step, heuristic, trial)
    #     table.to_csv(log_file, na_rep='n/a')
    #     print table
    #     print filename_idx, heuristic, max_step, algorithm
    # except:
    #     if str(sys.argv[1]) == 'all':
    #         trial = int(sys.argv[2])
    #         table = test_all(trial)
    #         print table
    #     else:
    #         print_usage()
    #         sys.exit


# if __name__ == '__main__':
#     try:
#         filename_idx, trial = int(sys.argv[1]), int(sys.argv[2])
#         algorithm = str(sys.argv[3])
#         try:
#             max_step, heuristic = int(sys.argv[4]), str(sys.argv[5])
#             if heuristic == 'None':
#                 heuristic = None
#         except:
#             max_step, heuristic = 1, None
#         logging.debug('trial = {}, algorithm = {}, max_step = {}, heuristic = {},'.format(trial, algorithm, max_step, heuristic))
#         log_file = '_heuristic_' if( heuristic is not None) else ''
#         log_file = './log_csv/{}{}_{}steps_M0{}_{}trials.csv'.format(algorithm, log_file, max_step, filename_idx, trial)
#         table = test(filename_idx, algorithm, max_step, heuristic, trial)
#         table.to_csv(log_file, na_rep='n/a')
#         print table
#         print filename_idx, heuristic, max_step, algorithm
#     except:
#         if str(sys.argv[1]) == 'all':
#             trial = int(sys.argv[2])
#             table = test_all(trial)
#             print table
#         else:
#             print_usage()
#             sys.exit

