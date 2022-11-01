#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  parser.py: parse the log files of a fawkes-robotino game and save
#  key data into csv exported data frames for easier processing.
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

import sys
import re
import pandas as pd
from parse import *
import argparse
import textwrap

header = '''
#####################################################################################
#                                                                                   #
#   Parse log files of fawkes-robotino RCLL games                                   #
#                                                                                   #
#####################################################################################
'''

parser = argparse.ArgumentParser(description=textwrap.dedent(header),
                                formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument(
    '--nbots',
    type=int,
    default=3,
    help='number of active fawkes instances, ignore when central')
parser.add_argument(
    '--central',
    type=bool,
    default=False,
    help='agent in central mode')
parser.add_argument(
    '--path',
    type=str,
    default="",
    help='path to the log files')
parser.add_argument(
    '--postfix',
    type=str,
    default="_latest.log",
    help='postfix of the log files "_latest.log" by default but might be changed to include dates')
args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
# validate inputs
if args==None:
    parser.exit(1)


c_bots = args.nbots
prefix=args.path
postfix=args.postfix
fn_bots = prefix+"debug{}"+postfix
fn_refbox = prefix+"refbox"+postfix
fn_refbox_debug = prefix+"refbox-debug"+postfix


def parse_table_line(line):
    line = line.split("|")[1:-1]
    elements = []
    for element in line:
        elements.append(element.strip())
    return elements

def preparse_line(line):
    return line.split("C:")[1:]


def extract_time_from_line(line):
    return parse('D {time} C:{}', line).named["time"]

datasets = {}
total_points = {"CYAN":-1, "MAGENTA": -1}
game_time = {"START":None, "END": None}

# parse the refbox debug log to extract start and end time of the game
with open(fn_refbox_debug) as reader:
    for line in reader.readlines():
        if "net-SetGamePhase PRODUCTION" in line:
            game_time["START"] =  extract_time_from_line(line)
        if "ws-attention-message \"Game Over\" nil 60" in line:
            game_time["END"] =  extract_time_from_line(line)

# parse the refbox log to extract game information
with open(fn_refbox) as reader:
    in_table = False
    table_title = None
    table_header = []
    table = []
    ind = 0
    lines = reader.readlines()

    while ind < len(lines)-2:
        line = lines[ind]
        line_processed = preparse_line(line)
        # parse the summary tables at the end of the log
        if len(line_processed) > 0:
            if not in_table:
                title_match = re.findall(r'--------(\w[\w\-\s]+\w)---------', line_processed[0])
                if len(title_match) > 0:
                    if "|" in lines[ind+2]:
                        in_table = True
                        table_title = title_match[0].lower().replace(" ","_")

                        table_header = parse_table_line(preparse_line(lines[ind+2])[0])
                        ind += 4
                        continue

            if in_table:
                if "|" in line:
                    table.append(dict(zip(table_header, parse_table_line(line_processed[0]))))
                if "|" not in line or ind+1 == len(lines):
                    datasets[table_title] = pd.DataFrame.from_records(table)
                    in_table = False
                    table_title = None
                    table_header = []
                    table = []
        # parse the total points per team from the script
        if "OVERALL TOTAL POINTS" in line:
            key = "MAGENTA"
            if total_points["CYAN"] == -1:
                key = "CYAN"
            total_points[key] = int(line.split(":")[-1].strip())
        ind += 1

# create meta dataset containing meta information of the game
datasets["meta"] = pd.DataFrame([
    {"key":"points:cyan","value":total_points["CYAN"]},
    {"key":"points:magenta","value":total_points["MAGENTA"]},
    {"key":"production:start","value":game_time["START"]},
    {"key":"production:end","value":game_time["END"]}])


# parse the log files of the robots into a dictionary of goals
def parse_robot_log(goals, robot):
    filename = fn_bots.format(robot)
    with open(filename, 'r') as reader:
        for line in reader.readlines():
            if "(goal (id" in line and "==>" in line:
                parsed = parse('D {time} {} ==> {} (goal (id {id:S}) (class {class:S}) '+
                               '(type {type:S}) (sub-type {subtype:S}) (parent {parent:S}) '+
                               '(mode {mode:S}) (outcome {outcome:S}){}(params{params}){}', line).named
                goal_robot_id = parsed['id']+'_r'+str(robot)
                if goal_robot_id not in goals.keys():
                    goals[goal_robot_id] = {"id":parsed["id"],
                                            "class":parsed["class"],
                                            "type":parsed["type"],
                                            "subtype":parsed["subtype"],
                                            "formulated":parsed["time"],
                                            "dispatched":None,
                                            "finished":None,
                                            "outcome":parsed["outcome"],
                                            "params":parsed["params"],
                                            "parent":parsed["parent"],
                                            "agent":"robot"+str(robot)}
                if goal_robot_id in goals.keys():
                    if goals[goal_robot_id]["dispatched"] is None and parsed["mode"] == "DISPATCHED":
                        goals[goal_robot_id]["dispatched"] = parsed["time"]
                    if goals[goal_robot_id]["finished"] is None and parsed["mode"] == "FINISHED":
                        goals[goal_robot_id]["finished"] = parsed["time"]

                    goals[goal_robot_id]["outcome"] = parsed["outcome"]
            # if there is a goal meta get the agent responsible for the goal from it
            if "(goal-meta " in line and "==>" in line and "assigned-to nil" not in line:
                parsed = parse('D {time} {} ==> {} (goal-meta (goal-id {id:S}) (assigned-to {robot:S}){}', line).named
                goal_robot_id = parsed['id']+'_r'+str(robot)
                goals[goal_robot_id]["agent"] = parsed["robot"]

# make case distinction from promises
goals = {}
if args.central:
    parse_robot_log(goals, 11)
else:
    for i in range(1, c_bots+1):
        parse_robot_log(goals, i)

datasets["goals"] = pd.concat([pd.DataFrame(goal, index=[0]) for goal in goals.values()],ignore_index=True)

# export the datasets
for dataset in datasets.keys():
    datasets[dataset].to_csv(prefix+"game-export_"+dataset+".csv")

# print an overivew of the key data extracted
print("--------------- OVERVIEW ---------------")
print("total points:",total_points["CYAN"])
print("deliveries")
print(datasets["production_points"][datasets["production_points"]['reason'].str.contains('Delivered item')][["game-time", "reason"]])
print("orders")
print(datasets["orders"][["id", "complexity", "delivery-period"]])
