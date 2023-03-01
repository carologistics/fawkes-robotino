#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  exp_visualizer.py: create visualisation of the results of a fawkes experiment
#  Given a compatibly structured directory (i.e. one created by the eval.sh script,
#  with outputs in CSV format from parser.py) with n baseline and n experiment runs,
#  create a visual comparison of the order deliveries of both approaches and
#  compare the performance in points of each by average, stdev, and max.
#  The directories in the given root directory must be of the form <baseline-name><game-number>
#  and <experiment-name><game-number>. E.g. for the second game of the baseline, with
#  baseline-name "baseline", the directory should be named "baseline2".
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
    '--baseline-prefix',
    type=str,
    help='prefix string of directories containing the baseline results, e.g for directories baseline1, baseline2, etc. this would be "baseline"',
    default='baseline')
parser.add_argument(
    '--experiment-prefix',
    type=str,
    help='prefix string of directories containing the experimental results, e.g for directories experiment1, experiment2, etc. this would be "experiment"',
    default='experiment')
parser.add_argument(
    '--training-prefix',
    type=str,
    help='prefix string of directories containing the experimental results, e.g for directories training1, training2, etc. this would be "training"',
    default='training')
parser.add_argument(
    '--path',
    type=str,
    required=True,
    help='path of the root directory containing all the runs of the given experiment')
parser.add_argument(
    '--game_time',
    type=int,
    default=1200,
    help='number of seconds in the game')
parser.add_argument(
    '--ngames',
    type=int,
    default=1,
    help='multiple games in one folder e.g. *_0.log for the first game')
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
def consistency_check():
    number_dicts = len(next(os.walk(args.path))[1])
    number_baseline = len([x for x in next(os.walk(args.path))[1] if args.baseline_prefix in x])
    number_experiment = len([x for x in next(os.walk(args.path))[1] if args.experiment_prefix in x])

    if number_baseline != number_experiment or number_baseline+number_experiment != number_dicts:
        print("inconsistent: number of runs do not match")
        return False

    #compare the order configurations
    last_df = None
    run_dicts = next(os.walk(args.path))[1]
    for dict in run_dicts:
        df = pd.read_csv(args.path+"/"+dict+"/"+"game-export_orders.csv").drop(["quantity-delivered"],axis=1)
        if last_df is not None:
            if not df.equals(last_df):
                print("inconsistent: order configurations not equal")
                print(last_df)
                print(df)
                return False
        last_df = df

    #compare the machine configurations
    last_df = None
    run_dirs = next(os.walk(args.path))[1]
    for dir in run_dirs:
        df = pd.read_csv(args.path+"/"+dir+"/"+"game-export_field_layout.csv").drop(["down-period"],axis=1)
        if last_df is not None:
            if not df.equals(last_df):
                print("inconsistent: machine configurations not equal")
                print(last_df)
                print(df)
                return False
        last_df = df

    return True

# print an analysis of the data given a summary operation
def print_summary(data, operation, title):
    print(title)
    if operation is None:
        print("Points:          ", data["baseline"]["points"],":",data["experiment"]["points"])
        print("Deliveries:      ", data["baseline"]["deliveries"],":",data["experiment"]["deliveries"])
        print("Late Deliveries: ", data["baseline"]["late"],":",data["experiment"]["late"])
    else:
        print("Points:         ",operation(data["baseline"]["points"]),":",operation(data["experiment"]["points"]))
        print("Deliveries:     ",operation(data["baseline"]["deliveries"]),":",operation(data["experiment"]["deliveries"]))
        print("Late Deliveries:",operation(data["baseline"]["late"]),":",operation(data["experiment"]["late"]))


# plot the results of applying an oper
def plot_summary(plt, grid_pos, data, operation, title):
    ax = plt.subplot2grid((6,2), grid_pos, colspan=1, rowspan=1)
    hide_grid(ax)
    ax.text(0, 0.8, title, weight='bold',fontsize=14)

    ax.text(0, 0.4, "{:4.2f}:{:4.2f}".format(operation(data["baseline"]["points"]),operation(data["experiment"]["points"])), weight='bold')
    ax.text(0,0.2, "points")
    ax.text(0.33, 0.4, "{:4.2f}:{:4.2f}".format(operation(data["baseline"]["deliveries"]),operation(data["experiment"]["deliveries"])), weight='bold')
    ax.text(0.33,0.2, "deliveries")
    ax.text(0.66, 0.4, "{:4.2f}:{:4.2f}".format(operation(data["baseline"]["late"]),operation(data["experiment"]["late"])), weight='bold')
    ax.text(0.66,0.2, "late deliveries")

def load_results_baseline_experiment():
    # parse the data and compute the values of interest
    if consistency_check():
        print("data consistent, generating overview plot")
        # load orders and meta into a dictionary
        results = {
            "baseline": {  "points": [], "deliveries": [], "late":[],
                        "runs":[{"name":name,"path":args.path+"/"+name,
                        "orders":pd.read_csv(args.path+"/"+name+"/"+"game-export_orders.csv"),
                        "points":pd.read_csv(args.path+"/"+name+"/"+"game-export_production_points.csv"),
                        "goals":pd.read_csv(args.path+"/"+name+"/"+"game-export_goals.csv"),
                        "meta":pd.read_csv(args.path+"/"+name+"/"+"game-export_meta.csv")} for name in next(os.walk(args.path))[1] if args.baseline_prefix in name]},
            "experiment": {"points": [], "deliveries": [], "late":[],
                        "runs":[{"name":name,"path":args.path+"/"+name,
                        "orders":pd.read_csv(args.path+"/"+name+"/"+"game-export_orders.csv"),
                        "points":pd.read_csv(args.path+"/"+name+"/"+"game-export_production_points.csv"),
                        "goals":pd.read_csv(args.path+"/"+name+"/"+"game-export_goals.csv"),
                        "meta":pd.read_csv(args.path+"/"+name+"/"+"game-export_meta.csv")} for name in next(os.walk(args.path))[1] if args.experiment_prefix in name]}
        }

        for run in results["baseline"]["runs"]:
            results["baseline"]["points"].append(int(run["meta"].loc[run["meta"]['key'] == "points:cyan"]["value"].iloc[0]))
            results["baseline"]["deliveries"].append(len(run["points"].loc[run["points"]['reason'].str.contains('Delivered item')].index))
            results["baseline"]["late"].append(len(run["points"].loc[run["points"]['reason'].str.contains('late')].index))
        for run in results["experiment"]["runs"]:
            results["experiment"]["points"].append(int(run["meta"].loc[run["meta"]['key'] == "points:cyan"]["value"].iloc[0]))
            results["experiment"]["deliveries"].append(len(run["points"].loc[run["points"]['reason'].str.contains('Delivered item')].index))
            results["experiment"]["late"].append(len(run["points"].loc[run["points"]['reason'].str.contains('late')].index))

        print_summary(results, None, "Results (baseline:experiment) for "+args.path)
        print_summary(results, mean, "Mean")
        print_summary(results, max, "Max")
        print_summary(results, stdev, "Stdev")
        generate_timeplot(results)

def load_results_training():
    print("In load results training")
    for name in next(os.walk(args.path))[1]:
        if args.training_prefix in name:
    #for i in range(0,args.ngames):
            print(f"Loading data of game {i}")
            # load orders and meta into a dictionary
            results = {
                f"{name}": {  "points": [], "deliveries": [], "late":[],
                            "runs":[{"name":name,"path":args.path+"/"+name,
                            "orders":pd.read_csv(args.path+"/"+name+"/"+f"game-export_orders-{i}.csv"),
                            "points":pd.read_csv(args.path+"/"+name+"/"+f"game-export_production_points-{i}.csv"),
                            "goals":pd.read_csv(args.path+"/"+name+"/"+f"game-export_goals-{i}.csv"),
                            "meta":pd.read_csv(args.path+"/"+name+"/"+f"game-export_meta-{i}.csv")} for i in range(0,args.ngames)]}
            }

            for run in results[f"{name}"]["runs"]:
                results[f"{name}"]["points"].append(int(run["meta"].loc[run["meta"]['key'] == "points:cyan"]["value"].iloc[0]))
                results[f"{name}"]["deliveries"].append(len(run["points"].loc[run["points"]['reason'].str.contains('Delivered item')].index))
                results[f"{name}"]["late"].append(len(run["points"].loc[run["points"]['reason'].str.contains('late')].index))
            
            print_summary(results, None, "Results (baseline:experiment) for "+args.path)
            print_summary(results, mean, "Mean")
            print_summary(results, max, "Max")
            print_summary(results, stdev, "Stdev")
            generate_timeplot(results)



def generate_timeplot(results, ):
    # generate timeplot data for deliveriy periods
    timepoints_start = []
    annotations_start = []
    timepoints_end = []
    annotations_end = []

    for index, row in results["baseline"]["runs"][0]["orders"].iterrows():
        values = [int(x) for x in row["delivery-period"].split(" ")]
        if values[0] <= args.game_time and values[1] <= args.game_time:
            timepoints_start.append(values[0])
            #annotations_start.append("O"+str(row["id"])+" start")
            annotations_start.append("O"+str(row["id"]))
            timepoints_end.append(values[1])
            #annotations_end.append("O"+str(row["id"])+" end")
            annotations_end.append("O"+str(row["id"]))

    # Choose some nice levels
    []
    levels = np.tile([x for i in range(1,int(len(timepoints_start)/2)+1) for x in (i,-i)],
                    int(np.ceil(len(timepoints_start)/((len(timepoints_start)/2+1)*2))))[:len(timepoints_start)]


    # set up gridplot
    fig = plt.figure(1,figsize=(15, 9),)
    gridspec.GridSpec(6,2)
    plt.suptitle('Experiment "'+args.path+'"', fontsize=18)

    # plot the order timeframes
    ax = plt.subplot2grid((6,2), (0,0), colspan=1, rowspan=2)
    plt.title('Delivery Windows')
    ax.vlines(timepoints_start, 0, levels, color="tab:blue")  # The vertical stems.
    ax.vlines(timepoints_end, 0, levels, color="tab:red")  # The vertical stems.
    ax.plot([0,args.game_time], [0,0], "-",
            color="k", markerfacecolor="w")  # Baseline and markers on it.

    # annotate lines with end and start points
    for d, l, r in zip(timepoints_start, levels, annotations_start):
        ax.annotate(r, xy=(d, l),
                    xytext=(-3, np.sign(l)*3), textcoords="offset points",
                    horizontalalignment="right",
                    verticalalignment="bottom" if l > 0 else "top")
    for d, l, r in zip(timepoints_end, levels, annotations_end):
        ax.annotate(r, xy=(d, l),
                    xytext=(-3, np.sign(l)*3), textcoords="offset points",
                    horizontalalignment="right",
                    verticalalignment="bottom" if l > 0 else "top")

    # remove y axis and spines
    ax.yaxis.set_visible(False)
    ax.spines[["left", "top", "right"]].set_visible(False)
    ax.margins(y=0.1)

    # plot the delivery times for the baseline
    ax = plt.subplot2grid((6,2), (2,0), colspan=1, rowspan=2)
    plt.title('Baseline Deliveries')
    ax.yaxis.set_visible(False)
    ax.spines[["left", "top", "right"]].set_visible(False)
    ax.margins(y=0.1)

    num_lines = len(results["baseline"]["runs"])

    for run in results["baseline"]["runs"]:
        ax.plot([0,args.game_time], [num_lines,num_lines], "-",
                color="k", markerfacecolor="w")  # Baseline and markers on it.
        timepoints = []
        annotations = []
        for index, row in run["points"].loc[run["points"]['reason'].str.contains('Delivered item')].iterrows():
            res = None
            if "late" in row["reason"]:
                res = parse("{} order {orderid:d}{}",row["reason"])
            else:
                res = parse("{} order {orderid:d}",row["reason"])
            timepoints.append(row["game-time"])
            annotations.append(res.named["orderid"])

        ax.vlines(timepoints, num_lines, [num_lines+0.5 for x in timepoints], color="tab:blue")  # The vertical stems.

        for d, l, r in zip(timepoints, [num_lines+0.5 for x in timepoints], annotations):
            ax.annotate(r, xy=(d, l),
                        xytext=(-1, -10), textcoords="offset points",
                        horizontalalignment="right",
                        verticalalignment="bottom" if l > 0 else "top")

        num_lines -= 1


    # plot the delivery times for the experiment
    ax = plt.subplot2grid((6,2), (4,0), colspan=1, rowspan=2)
    plt.title('Experiment Deliveries')
    ax.yaxis.set_visible(False)
    ax.spines[["left", "top", "right"]].set_visible(False)
    ax.margins(y=0.1)

    num_lines = len(results["experiment"]["runs"])

    for run in results["experiment"]["runs"]:
        ax.plot([0,args.game_time], [num_lines,num_lines], "-",
                color="k", markerfacecolor="w")  # Baseline and markers on it.
        timepoints = []
        annotations = []
        for index, row in run["points"].loc[run["points"]['reason'].str.contains('Delivered item')].iterrows():
            res = None
            if "late" in row["reason"]:
                res = parse("{} order {orderid:d}{}",row["reason"])
            else:
                res = parse("{} order {orderid:d}",row["reason"])
            timepoints.append(row["game-time"])
            annotations.append(res.named["orderid"])

        ax.vlines(timepoints, num_lines, [num_lines+0.5 for x in timepoints], color="tab:blue")  # The vertical stems.

        for d, l, r in zip(timepoints, [num_lines+0.5 for x in timepoints], annotations):
            ax.annotate(r, xy=(d, l),
                        xytext=(-1, -10), textcoords="offset points",
                        horizontalalignment="right",
                        verticalalignment="bottom" if l > 0 else "top")

        num_lines -= 1

    # key stats highlight
    ax = plt.subplot2grid((6,2), (1,1), colspan=1, rowspan=1)
    hide_grid(ax)
    ax.text(0, 0.8, 'Key Statistics', weight='bold',fontsize=16)
    ax.text(0, 0.0, 'Contains overviews of the key performance statistics w.r.t to a\nstandard game of the RCLL: points, number of deliveries and number\nof late deliveries. Read (value baseline):(value experiment)',fontsize=12)

    # plot the raw data
    ax = plt.subplot2grid((6,2), (2,1), colspan=1, rowspan=1)
    hide_grid(ax)
    ax.text(0, 0.8, 'Raw Values', weight='bold',fontsize=14)
    ax.text(0, 0.4, str(results["baseline"]["points"])+":", weight='bold')
    ax.text(0, 0.2, str(results["experiment"]["points"]), weight='bold')
    ax.text(0,0.0, "points")
    ax.text(0.33, 0.4,str(results["baseline"]["deliveries"])+":", weight='bold')
    ax.text(0.33, 0.2,str(results["experiment"]["deliveries"]), weight='bold')
    ax.text(0.33,0.0, "deliveries")
    ax.text(0.66, 0.4,str(results["baseline"]["late"])+":", weight='bold')
    ax.text(0.66, 0.2,str(results["experiment"]["late"]), weight='bold')
    ax.text(0.66,0.0, "late deliveries")

    # plot the key performance indicators over the data
    plot_summary(plt, (3,1), results, mean, "Mean")
    plot_summary(plt, (4,1), results, max, "Max")
    plot_summary(plt, (5,1), results, stdev, "Stdev")

    # visualize the orders
    ax = plt.subplot2grid((6,2), (0,1), colspan=1, rowspan=1)
    hide_grid(ax)
    plt.title('Orders')

    left_spacing = 0
    spacing_step = 1.0/len(results["baseline"]["runs"][0]["orders"])
    for index, row in results["baseline"]["runs"][0]["orders"].iterrows():
        height_spacing = 0.2
        ax.add_patch(patches.Rectangle((left_spacing, 0), spacing_step-4*(spacing_step/10), 0.2, linewidth=1,facecolor=row["base-color"][5:]))
        if type(row["ring-colors"])!=float:
            for color in row["ring-colors"].split(" "):
                ax.add_patch(patches.Rectangle((left_spacing, height_spacing), spacing_step-4*(spacing_step/10), 0.1, linewidth=1,facecolor=color[5:]))
                height_spacing += 0.1

        ax.add_patch(patches.Rectangle((left_spacing, height_spacing), spacing_step-4*(spacing_step/10), 0.05, linewidth=1,facecolor=row["cap-color"][4:]))
        ax.text(left_spacing,-0.2, "O"+str(row["id"])+" ("+row["complexity"]+")", fontsize=9)
        ax.text(left_spacing,-0.4, row["delivery-period"].replace(" ", "-"), fontsize=7)
        left_spacing+=spacing_step

    fig.tight_layout()
    plt.savefig(args.path+"/"+"result.pdf")
    print("Summary plot saved at "+args.path+"/"+"result.pdf")

if args.ngames == 1:
    load_results_baseline_experiment()
else:
    load_results_training()