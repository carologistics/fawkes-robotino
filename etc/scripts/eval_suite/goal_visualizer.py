#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  goal_visualizer.py: create a visualisation of the timeline of goals
#  for the agents in a game based on a game-export created by
#  the parser.py script.
#
#  Copyright © 2022 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

import argparse
import textwrap
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from datetime import datetime, timedelta

time_format = "%H:%M:%S.%f"

header = '''
#####################################################################################
#                                                                                   #
#   Create a goal timeline for each agent in an RCLL game based on a game-export    #
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
    '--ngames',
    type=int,
    default=1,
    help='multiple games in one folder e.g. *_0.log for the first game')
args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
# validate inputs
if args==None:
    parser.exit(1)


# update day if time is between 23 and 00
def check_day(start, end):
  if start.hour == 23 and end.hour ==0:
    end +=timedelta(days=1)
  return start, end

# difference in seconds between starttime given in datetime format and and endtime given string format
def time_string_diff(start, end):
  start, end = check_day(start, datetime.strptime(end, time_format))
  return -(start - end).total_seconds()

# difference in seconds between start and endtime given in string format
def string_string_diff(start, end):
  start, end = check_day(datetime.strptime(start, time_format) , datetime.strptime(end, time_format))
  return -(start - end).total_seconds()

# difference in seconds between start and endtime given in datetime format
def time_diff(start,end):
  start, end = check_day(start, end)
  return -(start - end).total_seconds()

# plot the given goals onto the given axis
def plot_goals(goals, ax, ps, pe, title,goal_color_map):
  plt.title(title)
  ax.set_xlim([0,time_diff(ps,pe)])
  ax.set_ylim([0,0.3])
  for index, row in goals.iterrows():
    hatch = None
    edgecolor = None
    if row['outcome'] == 'FAILED':
      hatch = '//'
      edgecolor = (0.8, 0.2, 0.2, 0.3)

    # duration = -time_string_diff(pe, row['dispatched'])
    duration = time_diff(datetime.strptime(row['dispatched'], time_format),pe)
    if row['outcome'] != 'UNKNOWN':
      duration = string_string_diff(row['dispatched'],row['finished'])
    ax.barh(0, duration, hatch=hatch, edgecolor=edgecolor, align='center',
            left=time_string_diff(ps,row['dispatched']), color=goal_color_map[row['class']])
    if row['outcome'] == 'UNKNOWN':
      ax.barh(0, duration, hatch='//', edgecolor=(0.2,0.2,0.8,0.2), align='center',
              left=time_string_diff(ps,row['dispatched']), color=(0,0,0,0))
    if duration > 30:
      plt.text(time_string_diff(ps,row['dispatched'])+1, 0.14,
                                row['id'],fontsize="4", color=(1,1,1))
  ax.yaxis.set_visible(False)
  ax.margins(y=2)

# plot a legend mapping the colors in the timeline to the goal classes
def plot_legend(color_map):
  colors = [color_map[x] for x in color_map.keys()]
  goals = color_map.keys()
  legend_lines = [Line2D([0],[0], color=color, lw=9) for color in colors]
  plt.legend(legend_lines, goals, loc='upper left', bbox_to_anchor=(0,0.5), ncol=3)


# load the goal data and filter the relevant goals
def generate_goal_visualization(game_export_goals_file, game_export_meta_file,number = None):
  # set the time to compute time differences
  ps = None
  pe = None
  df = None
  df = pd.read_csv(game_export_meta_file)
  for index, row in df.iterrows():
      if row['key'] == 'production:start':
          ps = datetime.strptime(row['value'], time_format)
      if row['key'] == 'production:end':
          pe = datetime.strptime(row['value'], time_format)

  df = pd.read_csv(game_export_goals_file)
  fg = df.loc[(df['dispatched'].notna())
            & (df['subtype'] == 'SIMPLE')
            & ~(df['class'].isin(['SEND-BEACON','EXPIRE-LOCKS',
                  'SPAWN-WP','PROCESS-MPS','REFILL-SHELF','MOVE-OUT-OF-WAY']))
            & (df['outcome'].isin(['COMPLETED','FAILED','UNKNOWN']))]

  number_classes = fg['class'].nunique()
  goal_classes = fg['class'].unique().tolist()

  # create color map
  cmap = plt.cm.get_cmap('viridis',fg['class'].nunique())
  goal_color_map = {}
  for i in range(0, number_classes):
    goal_class = goal_classes[i]
    goal_color_map[goal_class] = cmap(i/number_classes)

  # plot values for each agent
  unique_agents = fg["agent"].unique().tolist()
  grid_size = len(unique_agents)+1

  # create the figure + legend
  fig = plt.figure(1, figsize=(26,10),)
  gridspec.GridSpec(grid_size,1)
  plt.suptitle('Goal Plot')

  ax = plt.subplot2grid((grid_size,1), (0,0), colspan=1, rowspan=1)
  ax.axis('off')
  plot_legend(goal_color_map)

  row = 1
  for agent in unique_agents:
    # get the goals for one agent and plot them
    data = fg.loc[fg['agent'] == agent]
    ax = plt.subplot2grid((grid_size,1), (row,0), colspan=1,rowspan=1)
    plot_goals(data, ax, ps, pe, str(agent), goal_color_map)
    row += 1

  fig.tight_layout()
  if number:
    file_name = args.path+ f"goals{number}.pdf"

  file_name = args.path + "goals"+ (f"{number}.pdf" if number else ".pdf")
  plt.savefig(file_name)
  print(f"Goal timeline plot saved to {file_name}")

#'game-export_meta.csv' game-export_goals.csv
if args.ngames > 1:
  for i in range(0,args.ngames-1):
    generate_goal_visualization(args.path+f'game-export_goals-{i}.csv',args.path+f'game-export_meta-{i}.csv',i)
else:
  generate_goal_visualization(args.path+'game-export_goals.csv',args.path+'game-export_meta.csv')