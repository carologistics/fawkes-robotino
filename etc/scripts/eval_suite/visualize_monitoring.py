#!/usr/bin/env python3
# vim:fenc=utf-8
##########################################################################
#
#  visualize_monitoring.py: create visualisation of the results of
#  monitoring file created by the clipsWorld during training
#
#  Copyright © 2022
#
##########################################################################
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#

import numpy as np
import pandas as pd
import argparse
import textwrap
from statistics import mean, stdev
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
import os
import sys
from parse import *

header = '''
#####################################################################################
#                                                                                   #
#   Create a visualisation comparing an experimental implementation to a baseline   #
#                                                                                   #
#####################################################################################
'''

parser = argparse.ArgumentParser(description=textwrap.dedent(header),
                                formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument(
    '--path',
    type=str,
    required=True,
    help='path of the root directory containing all the runs of the given experiment')
parser.add_argument(
    '--name',
    type=str,
    help='name of the monitoring file',
    default='.monitor.csv')
parser.add_argument(
    '--dirs',
    nargs='+',
    help='list of directories to evaluate e.g [baseline1, baseline2] ',
    default='baseline')
args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
# validate inputs
if args==None:
    parser.exit(1)


# hide the grid on the given plt axis
def hide_grid(ax):
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)

# check the consistency of the experiment by
# - confirming that the number of baseline and experiment runs is equal
# - comparing the order configurations of each run
# - comparing the machine configurations of each run

#compare the number of runs
def read_monitoring_file():
    all_dfs = {}
    for dir in args.dirs:
        monitoring_file = args.path +"/"+ dir +"/"+ args.name
        print(monitoring_file)
        df = pd.read_csv(monitoring_file,header=1)
        if df is not None:
            all_dfs[dir]=df
            print(f"Inserted df of {dir}")
    return all_dfs

# plot the given coloumn onto the given axis
def create_boxplot(df, plot_title):
  # make invisible the upper and right axis
  ax = plt.gca()
  ax.spines['right'].set_color('none')
  ax.spines['top'].set_color('none')
  ax.yaxis.set_visible(False)
  ax.margins(y=0.1)
  
  # set title
  ax.set_xlabel(plot_title)
  #plt.title(plot_title.replace("_"," "), fontsize=12)
  plt.grid(False)
  red_square = dict(markerfacecolor='r', marker='x')
  plt.boxplot(df, vert=False, flierprops=red_square, whis=0.75, meanline=True, showmeans=True)
  plt.yticks([1],[''])
  return plt


# Start with reading the csv files
all_df = read_monitoring_file()

# create the figure + legend
fig = plt.figure(1, figsize=(10, 8), )
gs = gridspec.GridSpec(3,1)
gs.update(hspace=1.0)
plt.suptitle(f'Set of {len(all_df)} trainings', fontsize=14)
ax = fig.add_subplot(111)


count_rewards=[]
ax = plt.subplot2grid((3,1), (0,0), colspan=1,rowspan=2)

for t in all_df:
  #create_boxplot(df,col,title)
  tmp = all_df[t]
  count_episodes = all_df[t].shape[0]
  print(all_df[t].head())
  print(count_episodes)
  ax.plot(all_df[t]['r'])
  ax.set_xlim(0,count_episodes)
  ax.set_ylim(bottom=0)
  ax.set_xlabel('Games') #or Episodes
  ax.grid(False)

  count_rewards.append(all_df[t]['l'])


reward_df = pd.concat(count_rewards)
lx = fig.add_subplot(111)
ax = plt.subplot2grid((3,1), (2,0), colspan=1, rowspan=1)
create_boxplot(reward_df,'Count steps per episode')

fig.tight_layout()
plt.show()
loc = args.path +"/"+ dir +"/result.pdf"
plt.savefig(loc)
print(f"Goal timeline plot saved to {loc}")
