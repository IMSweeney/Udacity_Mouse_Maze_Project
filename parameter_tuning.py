"""
This module methods for running the robot through many tests of parameters and
analyzing the data generated from these tests.

@Author: Ian Sweeney
@Date: 07/15/2020
"""
import subprocess
import tqdm
import os
import pandas as pd
import numpy as np


def test_robot(num_runs):
    """ Runs a bunch of runs with of the robot with different mazes. """
    mazes = ['test_maze_01.txt', 'test_maze_02.txt', 'test_maze_03.txt']
    FNULL = open(os.devnull, 'w')

    for i in tqdm.tqdm(range(num_runs)):
        for maze in mazes:
            subprocess.run(['python3', 'tester.py', maze],
                           stdout=FNULL, stderr=FNULL)


def analyze_results():
    """
    Using the data in parameters.csv for all of the runs, find the best set of
    paramers for each maze individually, and for the three mazes combined.
    Print info on these parameters as well as the scores for these "best" runs.
    """

    # Read in and clean the data
    pd.set_option('display.max_columns', 500)
    cols = ['dim', 'score', 'explore_percent',
            'w1_goal', 'w1_self', 'w1_area',
            'w2_goal', 'w2_self', 'w2_area']
    param_cols = list(set(cols) - set(['dim', 'score']))
    data = pd.read_csv('parameters.csv', names=cols)
    data = data.drop_duplicates(subset=param_cols + ['dim'])
    data['params'] = data[param_cols].apply(
        lambda x: '-'.join(x.astype(str)), axis=1
    )

    # Find the best params for each maze
    print('Best params/score for the 12x12')
    best = data[data['dim'] == 12].drop(
        ['params', 'w1_area', 'w2_area'], axis=1)
    print(best.sort_values(by='score').head(), '\n')

    print('Best params/score for the 14x14')
    best = data[data['dim'] == 14].drop(
        ['params', 'w1_area', 'w2_area'], axis=1)
    print(best.sort_values(by='score').head(), '\n')

    print('Best params/score for the 16x16')
    best = data[data['dim'] == 16].drop(
        ['params', 'w1_area', 'w2_area'], axis=1)
    print(best.sort_values(by='score').head(), '\n')

    # Find the params that give the best sum score
    print('Sum score accross all 3 mazes')
    avg_score = data
    avg_score = data.pivot_table(
        index=['explore_percent', 'w1_goal', 'w1_self', 'w2_goal', 'w2_self'],
        columns='dim', values='score', aggfunc=np.sum,
        margins=True, margins_name='score_sum')
    avg_score = avg_score.dropna(how='any')
    print(avg_score.sort_values('score_sum').head(5))


if __name__ == '__main__':
    # test_robot(1000)
    analyze_results()
