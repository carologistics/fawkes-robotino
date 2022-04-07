#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  xenonite_parser.py: parse log files from the xenonite toy domain
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
#  Read the full text in the LICENSE.GPL file in the doc directory.


import os
import sys
from collections import namedtuple
import matplotlib.pyplot as plt
import numpy
from matplotlib.lines import Line2D
import datetime

"""
Usage: python xenonite_parser.py <path-to-files>

Provide DEBUG-level log files at the given path 
with names "debugN.log" where N is the robot number.
"""

base_goals = [
    "CLEAN-MACHINE",
    "DELIVER-XENONITE",
    "START-MACHINE",
    "FILL-CONTAINER"
]

def create_derivation(text):
    derv = ""
    for word in text.split("-"):
        if len(word) < 3 and word != "TO":
            derv+=word
        else:
            derv+=word[:1]
    return derv


def plot(bars, logs, goals, promises, spacings, outcomes, ids, pmps_machines, title):
    #cnfg
    plt.rcParams['figure.figsize'] = 14, 5

    #rng colors
    colors = dict()
    cmap = plt.cm.twilight

    start = 0.0
    cutoff = 0.9
    steps_base = (cutoff-start)/len(base_goals)
    steps_unforseen = (1-cutoff)/10
    next_color = 0.0+start

    #fill base colors
    for goal in base_goals:
        colors[goal] = cmap(next_color)
        if goal == "PROCESS-MPS":
            colors[goal] = colors[goal][:-1]+(0.3,)
        else:
            colors[goal] = colors[goal][:-1]+(0.8,)

        next_color += steps_base

    #fill additional colors
    for goal_tuple in goals:
        for goal in goal_tuple:
            if goal not in colors:
                colors[goal] = cmap(next_color)
                next_color += steps_unforseen

    #generate the bars
    for i in range(0, len(logs)):
        for j in range(0, len(logs[i])):
            log = [0] * len(logs[i])
            log[j] = logs[i][j]

            if promises[i][j]:
                plt.barh(bars, log, left=spacings[i],  color=colors[goals[i][j]], hatch="//", edgecolor=(0.5,0.5,0.5,0.2), linewidth=0)
                plt.rcParams['hatch.linewidth'] = 3
            else:
                plt.barh(bars, log, left=spacings[i],  color=colors[goals[i][j]])
            if logs[i][j]:
                if not outcomes[i][j]:
                    plt.barh(bars, log, left=spacings[i],  color=(1.0,0,0,0.0), align='center', hatch="\\", linewidth=0, edgecolor=(0.8,0.2,0.2,0.3))

    #generate labels
    for i in range(0, len(logs)):
       for j in range(0, len(logs[i])):
            if goals[i][j] != "NO-OP" and logs[i][j] > 2:
                label_text = create_derivation(goals[i][j])+"\n"+ids[i][j]
                if j == 3:
                    label_text = label_text + "\n"+pmps_machines[i][j]

                if create_derivation(goals[i][j]) == "CM":
                    plt.text(spacings[i][j]+0.2, j-0.05, label_text, fontsize="4", color=(1.0,1.0,1.0))
                elif logs[i][j] < 5:
                    plt.text(spacings[i][j]+0.2, j-0.05, label_text, fontsize="2")
                elif logs[i][j] < 10:
                    plt.text(spacings[i][j]+0.2, j-0.05, label_text, fontsize="4")
                else:
                    plt.text(spacings[i][j]+1, j-0.05, label_text, fontsize="7")

    #generate the fake lines for the legend
    if "NO-OP" in colors:
        del colors["NO-OP"]
    legend_lines = []
    for color in colors:
        legend_lines.append(Line2D([0], [0], color=colors[color], lw=9))
    legend_lines.append(Line2D([0], [0], color=(0,0,0,0.3), lw=9))
    plt.legend(legend_lines, colors.keys(),loc='upper left', bbox_to_anchor=(0, 1.2), ncol=3)

    #create the plot
    plt.title(title, loc="right")
    plt.xlabel('Time')
    plt.ylabel('Robots')
    plt.tight_layout()

    return plt


"""
General Helper Functions
"""

def substr_btw(inp, a, b, alt=""):
    try:
        return inp[inp.index(a)+len(a):inp.index(b, inp.index(a))]
    except:
        if alt != "":
            try:
                return inp[inp.index(a)+len(a):inp.index(alt, inp.index(a))]
            except:
                return False
        else:
            raise

def substr_from(inp, a):
    return inp[inp.index(a)+len(a):]

def substr_until(inp, a):
    return inp[:inp.index(a)]

"""
Typedefs
"""

WorldFact = namedtuple("WorldFact", "fact_id wm_id fact_class type args value time emitter")
Goal = namedtuple("Goal", "goal_id type goal_class parent params precons time outcome dispatched dispatched_time finished_time promised")
ScoringItem = namedtuple("ScoringItem", "time points description")
Delivery = namedtuple("Delivery", "time order deadline late")
Order = namedtuple("Order", "id complexity posted start end")
PromiseUtilization = namedtuple("PromiseUtilization", "goal_id bot promises")
Game = namedtuple("Game", "game_id points deliveries scoreitems starttime endtime promise_utilization")

"""
Specifc Checker Functions
"""

def update_goal_dispatched(goal, new_val):
    return  Goal(goal_id = goal.goal_id, type = goal.type, \
        goal_class = goal.goal_class, parent = goal.parent, \
            params = goal.params, precons=goal.precons, time=goal.time, \
                outcome = goal.outcome, dispatched=new_val, \
                    dispatched_time = goal.dispatched_time, \
                        finished_time=goal.finished_time, promised = goal.promised)

def update_goal_outcome(goal, new_val):
    return  Goal(goal_id = goal.goal_id, type = goal.type, \
        goal_class = goal.goal_class, parent = goal.parent, \
            params = goal.params, precons=goal.precons, time=goal.time, \
                outcome = new_val, dispatched=goal.dispatched, \
                    dispatched_time = goal.dispatched_time, \
                        finished_time=goal.finished_time, promised = goal.promised)

def update_goal_dispatched_time(goal, new_val):
    return  Goal(goal_id = goal.goal_id, type = goal.type, \
        goal_class = goal.goal_class, parent = goal.parent, \
            params = goal.params, precons=goal.precons, time=goal.time, \
                outcome = new_val, dispatched=goal.dispatched, \
                    dispatched_time = new_val, finished_time=goal.finished_time, \
                        promised = goal.promised)

def update_goal_finished_time(goal, new_val):
    return  Goal(goal_id = goal.goal_id, type = goal.type, \
        goal_class = goal.goal_class, parent = goal.parent, \
            params = goal.params, precons=goal.precons, time=goal.time, \
                outcome = new_val, dispatched=goal.dispatched, \
                    dispatched_time = goal.dispatched_time, \
                        finished_time=new_val, promised = goal.promised)

def check_fact_assertion(line, ignore_list):
    type = "promise"
    if ("==>" in line and "(key "+type+" " in line and substr_btw(line, "==> ", " (wm-fact") not in ignore_list and "wm-robmem-sync-map-entry" not in line):
        return type
    type = "domain"
    if ("==>" in line and "key "+type+" fact" in line and "objects-by-type" not in line and substr_btw(line, "==> ", " (wm-fact") not in ignore_list and "wm-sync-map-fact" not in line and "wm-robmem-sync-map-entry" not in line):
        return type
    type = "predicate-medium"
    if ("==>" in line and "key "+type+" " in line and substr_btw(line, "==> ", " (wm-fact") not in ignore_list and "wm-sync-map-fact" not in line and "wm-robmem-sync-map-entry" not in line):
        return type
    return False

def check_corr_assertion(line, lookup_line, type):
    try:
        return ("==>" in lookup_line and "(key "+type+" " in lookup_line and "objects-by-type" not in lookup_line and substr_btw(line, "key "+type+" ", " args?") == substr_btw(lookup_line, "key "+type+" ", " args?") \
            and substr_btw(line, "args? ", ")") == substr_btw(lookup_line, "args? ", ")") and "wm-robmem-sync-map-entry" not in lookup_line and "wm-sync-map-fact" not in lookup_line )
    except:
        return False

def check_goal_assertion(line):
    return "FIRE" in line and ("goal-class-assert" in line) and not "achieve" in line and not "maintain" in line and not "goal-reasoner" in line

def check_goal_fact_assertion(line):
    return "==>" in line and "(goal " in line and "(class" in line and "(id" in line

def is_starttime(line):
    return "==>" in line and "/refbox/phase" in line and "value PRODUCTION" in line

def is_stoptime(line):
    return "==>" in line and "/refbox/phase" in line and "value POST_GAME" in line

"""
Global vars
"""

facts_1 = dict([])
goals_1 = dict([])
p_mps_goals_1 = dict([])
endtime_1 = ""
starttime_1 = ""

facts_2 = dict([])
goals_2 = dict([])
p_mps_goals_2 = dict([])
endtime_2 = ""
starttime_2 = ""

facts_3 = dict([])
goals_3 = dict([])
p_mps_goals_3 = dict([])
endtime_3 = ""
starttime_3 = ""

game = None

"""
Main functionality
"""

def parse_logfile(logfile, facts, goals, process_mps_goals):
    starttime = ""
    endtime = ""
    loglines = logfile.splitlines()

    ignore_list = []

    ind = 0

    prev_line = ""

    for line in loglines:
        #find first assertions of facts and create objects for them
        if (fact_type := check_fact_assertion(line, ignore_list)):
            ignore_list.append(substr_btw(line, "==> ", " (wm-fact"))

            final_assert = None
            if substr_btw(line, "(id ", ")") != "\"\"":
                final_assert = line
            else:
                final_assert_ind = ind+1
                while final_assert is None and final_assert_ind < len(loglines):
                    lookup_line = loglines[final_assert_ind]
                    if check_corr_assertion(line, lookup_line,fact_type):
                        final_assert = lookup_line
                        ignore_list.append(substr_btw(lookup_line, "==> ", " (wm-fact"))
                    final_assert_ind += 1

        #find first assertions (formulations) of goals and create objects for them
        if check_goal_assertion(line):
            rule = line
            goal = loglines[ind+2]
            promised = False

            for i in range(0,2000): #handle edge cases
                if ("from promise" in loglines[ind+i]):
                    promised = True
                if "(goal " in loglines[ind+i] and "==>" in loglines[ind+i]:
                    goal = loglines[ind+i]
                    break

            #handle edge cases where there is no time
            precons = ""
            try:
                precons = substr_from(substr_from(rule, "executive):"), ": ").split(",")
            except:
                precons = substr_from(rule, ": ").split(",")
            if "*" in precons:
                precons = [i for i in precons if i != "*"]



            if "sub-type" in goal:
                goal = Goal(goal_id = substr_btw(goal, "(id ", ")"), type = substr_btw(goal, "(sub-type ", ")"), \
                    goal_class = substr_btw(goal, "(class ", ")"), parent = substr_btw(goal, "(parent ", ")"), \
                        params = substr_btw(goal, "(params", ")").strip(), precons=precons, time=substr_btw(goal, "D ", " CLIPS"), \
                            outcome = substr_btw(goal, "(outcome ", ")"), dispatched=False, dispatched_time="", finished_time="", promised=promised)
            elif "type" in goal:
                goal = Goal(goal_id = substr_btw(goal, "(id ", ")"), type = substr_btw(goal, "(type ", ")"), \
                    goal_class = substr_btw(goal, "(class ", ")"), parent = substr_btw(goal, "(parent ", ")"), \
                        params = substr_btw(goal, "(params", ")").strip(), precons=precons, time=substr_btw(goal, "D ", " CLIPS"), \
                            outcome = substr_btw(goal, "(outcome ", ")"), dispatched=False, dispatched_time="", finished_time="", promised=promised)

            if goal.goal_class == "PROCESS-MPS":
                process_mps_goals[goal.goal_id] = goal
            elif goal.goal_class != "PRODUCTION-MAINTAIN":
                goals[goal.goal_id] = goal

        #updates of goal information (state, times)
        if check_goal_fact_assertion(line) and (goal_id := substr_btw(line, "(id ", ")")) in goals:
            if substr_btw(line, "(mode ", ")") == "DISPATCHED" and not goals[goal_id].dispatched:
                goals[goal_id] = update_goal_dispatched(goals[goal_id], True)
                goals[goal_id] = update_goal_dispatched_time(goals[goal_id], substr_btw(line, "D ", " CLIPS").strip())
                if goals[goal_id].promised:
                   print("Goal dispatched from promise: ", goal_id)
            if substr_btw(line, "(mode ", ")") == "FINISHED":
                goals[goal_id] = update_goal_finished_time(goals[goal_id], substr_btw(line, "D ", " CLIPS").strip())
            if substr_btw(line, "(outcome ", ")") == "REJECTED":
                goals[goal_id] = update_goal_finished_time(goals[goal_id], substr_btw(line, "D ", " CLIPS").strip())
                if goals[goal_id].dispatched:
                    goals[goal_id] = update_goal_dispatched(goals[goal_id], False)
            goals[goal_id] = update_goal_outcome(goals[goal_id], substr_btw(line, "(outcome ", ")"))
        if check_goal_fact_assertion(line) and (goal_id := substr_btw(line, "(id ", ")")) in process_mps_goals:
            if substr_btw(line, "(mode ", ")") == "DISPATCHED" and not process_mps_goals[goal_id].dispatched:
                process_mps_goals[goal_id] = update_goal_dispatched(process_mps_goals[goal_id], True)
                process_mps_goals[goal_id] = update_goal_dispatched_time(process_mps_goals[goal_id], substr_btw(line, "D ", " CLIPS").strip())
            if substr_btw(line, "(mode ", ")") == "FINISHED":
                process_mps_goals[goal_id] = update_goal_finished_time(process_mps_goals[goal_id], substr_btw(line, "D ", " CLIPS").strip())
            process_mps_goals[goal_id] = update_goal_outcome(process_mps_goals[goal_id], substr_btw(line, "(outcome ", ")"))

        #determine start and stoptime
        if is_starttime(line) and starttime == "":
            starttime = substr_btw(line, "D ", " CLIPS")
        if is_stoptime(line) and endtime == "":
            endtime = substr_btw(line, "D ", " CLIPS")
            break

        ind+=1
        prev_line = line

    if endtime == "":
        try:
            endtime = substr_btw(loglines[-2], " ", " CLIPS")
        except:
            endtime = substr_btw(loglines[-2], " ", " Fawkes")


    #fix goal endtimes
    for goal in goals:
        if goals[goal].finished_time == "":
            goals[goal] = update_goal_finished_time(goals[goal], endtime)
    for goal in process_mps_goals:
        if process_mps_goals[goal].finished_time == "":
            process_mps_goals[goal] = update_goal_finished_time(process_mps_goals[goal], endtime)

    return starttime, endtime

def generate_plot(run_goals, run_p_mps_goals, run_facts, run_starttimes, run_endtimes, title):
    graph_logs = []
    graph_goals = []
    graph_promises = []
    graph_outcomes = []
    graph_spacings = []
    graph_ids = []
    pmps_machines = []

    #combine the goals
    combined_goals = run_goals

    bot_starttime = None
    #iterate over bots goals+process mps goals
    for i in range(0, len(combined_goals)):
        goals = combined_goals[i]

        goal_ind = 0
        for goal_key in goals:
            goal = goals[goal_key]
            if goal.dispatched:
                if goal_ind == 0 and i != 3:
                    bot_starttime = datetime.datetime.strptime(goal.dispatched_time, "%H:%M:%S.%f")

                if goal_ind == len(graph_goals):
                    graph_goals.append(["NO-OP","NO-OP","NO-OP"])
                    graph_ids.append(["","",""])
                    pmps_machines.append(["","",""])
                    graph_promises.append([False, False, False])
                    graph_logs.append([0,0,0])
                    graph_outcomes.append([False,False,False])
                    graph_spacings.append([0,0,0])

                graph_goals[goal_ind][i] = goal.goal_class
                graph_ids[goal_ind][i] = goal.goal_id.split("-")[-1]
                #if i == 3:
                #    pmps_machines[goal_ind][i] = goal.goal_id.split("-")[2]+"-"+goal.goal_id.split("-")[3]

                dptime = datetime.datetime.strptime(goal.dispatched_time, "%H:%M:%S.%f")
                edtime = datetime.datetime.strptime(goal.finished_time, "%H:%M:%S.%f")
                graph_logs[goal_ind][i] = (edtime - dptime).seconds
                graph_spacings[goal_ind][i] = (dptime - bot_starttime).seconds

                graph_promises[goal_ind][i] = goal.promised
                graph_outcomes[goal_ind][i] = (goal.outcome == "COMPLETED")

                goal_ind+=1

    return plot(["bot 1", "bot 2", "bot 3"], graph_logs, graph_goals, graph_promises, graph_spacings, graph_outcomes, graph_ids, pmps_machines, title)


#check argument length
if len(sys.argv) > 1:
    path = sys.argv[1]
    if path[-1:] == "/":
        path = path[:-1]
    #file_prefix = path[path.rindex("/")+1:]
    speedup=2
    if len(sys.argv) > 2:
       speedup=float(sys.argv[2])

    log_bot1 = path+"/debug1.log"
    log_bot2 = path+"/debug2.log"
    log_bot3 = path+"/debug3.log"

    #read and parse each file
    if os.path.isfile(log_bot1): #check if file exists /generation successful
        with open(log_bot1,'rb') as file:
            file = file.read().decode('ISO-8859-1')
            starttime_1, endtime_1 = parse_logfile(file, facts_1,goals_1, p_mps_goals_1)
    if os.path.isfile(log_bot2): #check if file exists /generation successful
        with open(log_bot2,'rb') as file:
            file = file.read().decode('ISO-8859-1')
            starttime_2, endtime_2 = parse_logfile(file, facts_2,goals_2, p_mps_goals_2)
    if os.path.isfile(log_bot3): #check if file exists /generation successful
        with open(log_bot3,'rb') as file:
            file = file.read().decode('ISO-8859-1')
            starttime_3, endtime_3 = parse_logfile(file, facts_3,goals_3, p_mps_goals_3)

    #pack results
    run_goals = [goals_1,goals_2,goals_3]
    run_p_mps_goals = [p_mps_goals_1,p_mps_goals_2,p_mps_goals_3]
    run_p_mps_goals_combined = dict([])
    meh = 0
    for goals in run_p_mps_goals:
        for goal_id in goals:
            run_p_mps_goals_combined[goal_id+"_"+str(meh)] = goals[goal_id]
        meh += 1

    run_facts = [facts_1,facts_2,facts_3]
    run_starttimes = [starttime_1,starttime_2,starttime_3]
    run_endtimes = [endtime_1,endtime_2,endtime_3]

    #determine the promise utilization
    bot = 0
    promise_utilization = {"dispatched":[],"formulated":[]}
    for goals in run_goals:
        bot+=1
        for goal in goals:
            if goals[goal].promised:
                promises_used = []

                promised_goal = PromiseUtilization(goal_id = goals[goal].goal_id, bot = bot, promises = promises_used)
                if goals[goal].dispatched:
                    promise_utilization["dispatched"].append(promised_goal)
                else:
                    promise_utilization["formulated"].append(promised_goal)

    #fill game variable
    game = Game(game_id="", points = {}, deliveries = [], scoreitems = [], starttime="", endtime="", promise_utilization=promise_utilization)

    """
    OUTPUTTY STUFFS
    """
    #generate the report strings
    dispatched_counter = 0
    rejected_counter = 0
    rejected_promised_counter = 0
    formulated_counter = 0
    formulated_promised_counter = 0
    dispatched_promised_counter = 0
    dispatched_failed_counter = 0
    dispatched_failed_promised_counter = 0
    for i in range(0, len(run_goals)):
        goals = run_goals[i]
        for goal_key in goals:
            goal = goals[goal_key]
            formulated_counter += 1
            if goal.promised:
                formulated_promised_counter += 1
            if goal.outcome == "REJECTED":
                rejected_counter += 1
                if goal.promised:
                    rejected_promised_counter += 1
            if goal.dispatched:
                dispatched_counter += 1
                if goal.promised:
                    dispatched_promised_counter +=1
                if goal.outcome != "COMPLETED":
                    dispatched_failed_counter +=1
                    if goal.promised:
                        dispatched_failed_promised_counter +=1
    
    out_string = ""
    out_string += f'FORMULATED GOALS:    {str(formulated_counter)}\n'
    out_string += f'  > PROMISED: {str(formulated_promised_counter)}\n'

    out_string += f'REJECTED GOALS:    {str(rejected_counter)}\n'
    out_string += f'  > PROMISED: {str(rejected_promised_counter)}\n'

    out_string += f'DISPATCHED GOALS:    {str(dispatched_counter)}\n'
    out_string += f'  > FAILED:   {str(dispatched_failed_counter)}\n'
    out_string += f'  > PROMISED: {str(dispatched_promised_counter)}\n'
    out_string += f'    - FAILED: {str(dispatched_failed_promised_counter)}\n'


    out_string += '\n\n\n-- RESULTS --\n\n\n'


    first_goal = None
    failed_goals = 0

    robot_busy_time = dict()
    machine_instruct_time = 0

    #print the results
    for i in range(0, len(run_goals)):
        robot = f'Robot {str(i+1)}'
        out_string += f'{robot}:\n\n'
        robot_busy_time[robot]=0
        goals = run_goals[i]
        p_mps_goals = run_p_mps_goals[i]
        facts = run_facts[i]


        for goal_key in goals:
            goal = goals[goal_key]
            if first_goal is None:
                first_goal = goal

            out_string += f'{goal.time}: {goal_key}:\n'
            if goal.dispatched:
                if not "WAIT" in goal_key:
                    dispatch_start = datetime.datetime.strptime(goal.dispatched_time, '%H:%M:%S.%f')
                    dispatch_end = datetime.datetime.strptime(goal.finished_time, '%H:%M:%S.%f')
                    goal_duration = (dispatch_end - dispatch_start).total_seconds()*speedup
                    robot_busy_time[robot] +=goal_duration
                out_string += f'  > DISPATCHED {goal_key} ({goal.dispatched_time} - {goal.finished_time})\n'
                out_string += f'  > {goal.outcome}\n'
                if goal.outcome != "COMPLETED":
                    failed_goals+=1

            else:
                out_string += f'  > FORMULATED\n'

            if goal.promised:
                out_string += "  > PROMISES:\n"
                for precon in goal.precons:
                    if precon in facts and (facts[precon].fact_class == "promise" or facts[precon].fact_class == "predicate-medium" and facts[precon].emitter != ""):
                        out_string += f'      - {facts[precon].type}\n'
        out_string += '\n\n\n'
        for goal_key in p_mps_goals:
            goal = p_mps_goals[goal_key]
            if goal.dispatched:
                dispatch_start = datetime.datetime.strptime(goal.dispatched_time, '%H:%M:%S.%f')
                dispatch_end = datetime.datetime.strptime(goal.finished_time, '%H:%M:%S.%f')
                goal_duration = (dispatch_end - dispatch_start).total_seconds()*speedup
                machine_instruct_time +=goal_duration
                out_string += f'{goal.time}: {goal_key}:\n'
                out_string += f'  > DISPATCHED ({goal.dispatched_time} - {goal.finished_time})\n'
                out_string += f'  > {goal.outcome}\n'
                if goal.promised:
                    out_string += "  > PROMISES:\n"
                    for precon in goal.precons:
                        if precon in facts and (facts[precon].fact_class == "promise" or facts[precon].fact_class == "predicate-medium" and facts[precon].emitter != ""):
                            out_string += f'      - {facts[precon].type}\n'

        out_string += '\n\n\n'
    total_busy_time = 0
    for robot, time in robot_busy_time.items():
        total_busy_time+=time

    out_string += f'BUSY TIME: {total_busy_time}\n'
    for robot, time in robot_busy_time.items():
        out_string += f'  > {robot}: {time}\n'

    #generate the plot
    title = ""
    plt = generate_plot(run_goals, run_p_mps_goals_combined, run_facts, run_starttimes, run_endtimes, title)

    #to file
    out_file = open(path+"/report.txt", "w")
    out_file.write(out_string)
    out_file.close()
    plt.savefig(path+"/report.pdf")

    #to stdout
    if len(sys.argv) > 2 and sys.argv[2] == "--print":
        print(out_string)
        plt.show()
