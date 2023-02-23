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
import os
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
parser.add_argument(
    '--ngames',
    type=bool,
    default=False,
    help='multiple games in one log file e.g. training central-rl agent')
parser.add_argument(
    '--duration',
    type=int,
    default=1200,
    help='Duration of the game in sec')
args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
# validate inputs
if args==None:
    parser.exit(1)


c_bots = args.nbots
prefix=args.path
postfix=args.postfix
fn_bots = prefix+"debug{}"+postfix
fn_refbox = prefix+"refbox"#+postfix
fn_refbox_debug = prefix+"refbox-debug"#+postfix

datasets = {}
total_points = {"CYAN":-1, "MAGENTA": -1}
game_time = {"START":None, "END": None}


def parse_table_line(line):
    line = line.split("|")[1:-1]
    elements = []
    for element in line:
        elements.append(element.strip())
    return elements

def preparse_line(line):
    return line.split("C:")[1:]


def extract_time_from_line(line):
    parsed_line = parse('{} {time} {}:{}', line)
    if parsed_line:
        return parsed_line.named["time"]
    return -1

# parse the refbox debug log to extract start and end time of the game
def parse_refbox_debug_log(log_file = fn_refbox_debug):
    with open(log_file) as reader:
        for line in reader.readlines():
            if "net-SetGamePhase PRODUCTION" in line:
                print(line)
                game_time["START"] =  extract_time_from_line(line)
            if "ws-attention-message \"Game Over\" nil 60" in line:
                time = extract_time_from_line(line)
                print(line)
                game_time["END"] =  time

    return game_time["START"], game_time["END"]

# parse the refbox log to extract game information
def parse_refbox_log(log_file = fn_refbox):
    with open(log_file) as reader:
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

    return datasets

# parse the log files of the robots into a dictionary of goals
def parse_robot_log(goals, filename, robot):
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
                if goal_robot_id in goals.keys():
                    goals[goal_robot_id]["agent"] = parsed["robot"]
    
def extract_game_split_times(long_file, postfix):
    "Split here >8"
    split_times={}
    with open(long_file+postfix) as reader:
        i = 0
        file_name = long_file+f"_split_{i}.log" 
        small_file = open(file_name, "w")

        for line in reader.readlines():
            if "Split here >8" in line:
                time = extract_time_from_line(line)
                split_times[i]=time
                small_file.close()
                i += 1
                file_name = long_file+f"_split_{i}.log" 
                small_file = open(file_name, "w")
            
            small_file.write(line)
        small_file.close()
        os.remove(file_name)
    return split_times

def split_file_into_games(long_file,postfix,  split_times): #df_game_times):
     #split refbox log
    with open(long_file+postfix) as reader:
        i = 0
        file_name = long_file+f"_split_{i}.log" 
        small_file = open(file_name, "w")

        for line in reader.readlines():
            time = extract_time_from_line(line)
            game_end_time = split_times[i] #df_game_times[i]["END"]
            
            if time != -1 and time > game_end_time:
                print(f"Time greater! {time} > {game_end_time}")
                small_file.close()
                i += 1
                if( i >= game_count):
                    break
                file_name = long_file+f"_split_{i}.log" 
                small_file = open(file_name, "w")
                
            small_file.write(line)
    


game_datasets={}
game_times={}
# split refbox and refbox debug log into multiple files per game
if args.ngames:

    long_file = prefix+"debug11"
    split_times = extract_game_split_times(long_file,postfix)
    print(f"split times: {split_times}")
    game_count = len(split_times)
    # split refbox debug log
    split_file_into_games(fn_refbox_debug,postfix,split_times)
    for i in range(0, game_count):
        file_name = fn_refbox_debug+f"_split_{i}.log" 
        start_time, end_time = parse_refbox_debug_log(file_name)
        print(f"game {i}: start {start_time} end {end_time}")
        # create meta dataset containing meta information of the game
        game_times[i]= {"START": start_time,  "END": end_time} 

    df = pd.DataFrame(game_times)
    df.to_csv(prefix+"overview_game_times.csv")

    # split refbox log
    long_file = prefix+"refbox"
    split_file_into_games(long_file,postfix,split_times)
    for i in range(0, game_count):
        short_name = prefix+f"refbox_split_{i}.log"
        ds = {}
        ds = parse_refbox_log(short_name)
        ds["meta"] = pd.DataFrame([
        {"key":"production:start","value":game_times[i]["START"]},
        {"key":"production:end","value":game_times[i]["END"]}])
        for dataset in ds.keys(): 
            ds[dataset].to_csv(prefix+"game-export_"+dataset+f"-{i}.csv")

    """ with open(fn_refbox_debug) as reader:
        content = reader.read()

    game_over_indicator = "game-over"
    game_start_indicator = "game-switch-back-to-setup"
    game_count = content.count(game_over_indicator)
    print(f"Game count: {game_count}")
    game_start = 0
    split_end =0
    
    for i in range(0,game_count):
        #split refbox debug log
        file_start = content.find(game_start_indicator) 
        split_end = content[file_start+ len(game_start_indicator):].find("game-switch-back-to-setup")
        if split_end == -1 :
            split_end = len(content)-1
        file_end = file_start + split_end
        file_name = fn_refbox_debug+f"_split_{i}.log" 
        with open(file_name, mode="w") as newfile:
            newfile.write(f"Game {i}\n"+content[file_start:file_end])
        start_time, end_time = parse_refbox_debug_log(file_name)
        print(f"game {i}: start {start_time} end {end_time}")
        if end_time == None or (start_time > end_time and end_time > "0:15:0"):
            file_start = content.find("net-SetGamePhase PRODUCTION")+len("net-SetGamePhase PRODUCTION")
            split_end = content[file_start:].find("ws-attention-message \"Game Over\" nil 60")+len("ws-attention-message \"Game Over\" nil 60")+1
            file_name = fn_refbox_debug+f"_split_{i}.log" 
            with open(file_name, mode="w") as newfile:
                newfile.write(f"Game {i}\n"+content[file_start:file_start+split_end])
            start_time, end_time = parse_refbox_debug_log(file_name)
            print(f"game {i}: start {start_time} end {end_time}")

        # create meta dataset containing meta information of the game
        game_times[i]= {"START": start_time,  "END": end_time} 
        
        i +=1
        content = content[file_end-1:]

    df = pd.DataFrame(game_times)
    df.to_csv(prefix+"overview_game_times.csv")

     #split refbox log
    long_file = prefix+"refbox"
    split_file_into_games(long_file,postfix,df)
    for i in range(0, game_count):
        short_name = prefix+f"refbox_split_{i}.log"
        ds = {}
        ds = parse_refbox_log(short_name)
        ds["meta"] = pd.DataFrame([
        {"key":"production:start","value":game_times[i]["START"]},
        {"key":"production:end","value":game_times[i]["END"]}])
        for dataset in ds.keys(): 
            ds[dataset].to_csv(prefix+"game-export_"+dataset+f"-{i}.csv")
     """    
else:
    start_time, end_time = parse_refbox_debug_log(fn_refbox_debug)
    # create meta dataset containing meta information of the game
    print(f"refbox-debug: {datasets}")
    datasets = parse_refbox_log(fn_refbox)
    datasets["meta"] = pd.DataFrame([
        {"key":"production:start","value":game_time["START"]},
        {"key":"production:end","value":game_time["END"]}])
    game_datasets[0]=datasets
    print(f"refbox: {datasets}")



goals = {}
if args.central and args.ngames:
    long_file = prefix+"debug11"
    #split_times = extract_game_split_times(long_file,postfix)
    #split_file_into_games(long_file,postfix,split_times)
    for i in range(0, game_count):
        short_name = prefix+f"debug11_split_{i}.log"
        goals = {}
        parse_robot_log(goals, short_name,1)
        if goals:
            df_goals = pd.concat([pd.DataFrame(goal, index=[0]) for goal in goals.values()],ignore_index=True)
            df_goals.to_csv(prefix+"game-export_goals"+f"-{i}.csv")
elif args.central:
    filename = fn_bots.format(11)
    parse_robot_log(goals, filename,11)
    datasets["goals"] = pd.concat([pd.DataFrame(goal, index=[0]) for goal in goals.values()],ignore_index=True)
    datasets["goals"].to_csv(prefix+"game-export_goals"+f"-{i}.csv")
else:
    for i in range(1, c_bots+1):
        filename = fn_bots.format(i)
        parse_robot_log(goals, filename,i)
        datasets["goals"] = pd.concat([pd.DataFrame(goal, index=[0]) for goal in goals.values()],ignore_index=True)
        datasets["goals"].to_csv(prefix+"game-export_"+goals+f"-{i}.csv")

# export the datasets
"""for i in game_datasets:
    for dataset in game_datasets[i].keys(): #for dataset in datasets.keys():
        datasets[dataset].to_csv(prefix+"game-export_"+dataset+f"-{i}.csv")"""

# print an overivew of the key data extracted
print("--------------- OVERVIEW ---------------")
print("total points:",total_points["CYAN"])
#print("deliveries")
#print(datasets["production_points"][datasets["production_points"]['reason'].str.contains('Delivered item')][["game-time", "reason"]])
#print("orders")
#print(datasets["orders"][["id", "complexity", "delivery-period"]])
