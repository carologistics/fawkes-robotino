#!/usr/bin/env python3
# ***************************************************************************
# *  Script to generate map after measuring lengths physically
# *
# *  Created:  May 10 16:30:00 2019
# *  Copyright  2019  Morian Sonnet [Morian.Sonnet@rwth-aachen.de]
# *
# ****************************************************************************
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
#
import getopt
import re  # for verifying correct input string
import sys

import numpy as np
from PIL import Image
from PIL import ImageDraw

# define regex for the different elements

# a float number, e.g. -2.453 +2.3344 45
# used to give lengths of line elements, as well as coordinates
reg_num = r"[-+]?\d+(?:\.\d*)?"

# a float number, with an optional appending asterisk *
# describes a painted line element (with the asterisk, the line element is highlighted in red)
reg_wall = reg_num + r"\*?"

# a float number, with a definite appending asterisk *
# describes a highlighted line element
reg_wall_red = reg_num + r"\*"

# a float number, embedded in square brackets []. The number is followed by an optional asterisk *
# describes a vacancy (with an asterisk, the vacancy is highlighted as a red dotted line
reg_empty = r"\[" + reg_num + r"\*?\]"

# a float number, appended with a definite asterisk *, embedded in square brackets []
# describes a highlighted vacancy
reg_empty_red = r"\[" + reg_num + r"\*\]"

# two float numbers, separated by a comma ,, embedded in parantheses ()
# describes the starting position of a line streak
reg_coordinate = r"\(" + reg_num + r"," + reg_num + r"\)"

# multiple line elements and/or vacancies, finally ending with a semicolon
# describes multiple line elements and/or vacancies in the same direction
# thus a so called same direction element
reg_same_dir_element = r"(?:(?:" + reg_empty + r"|" + reg_wall + r"|)+;)"

# multiple same direction elements, each consisting of multiple line elements and/or vacancies
# Prepended by a coordinate
# describes an entire line streak, the first same direction element is in x direction
# with each following same direction element, the direction is switched
reg_long_line = reg_coordinate + reg_same_dir_element + "+"

# the equivalent of one meter in pixel
pixel_per_meter = 20

# margin in x and y direction, relative to the most outer line element or vacancy
x_margin = 10
y_margin = 10


def update_limits(x, y, min_x, min_y, max_x, max_y):
    return min(x, min_x), min(y, min_y), max(x, max_x), max(y, max_y)


# parse a command string
# @param command The command string
# @return lines list of normal drawn line elements, each with format [(xs,ys),(xe,ye)]
# @return red_line list of highlighted line elements
# @return red_dotted_lines ligh of highlighted vacancies
# @return (min/max)_(x/y) minimum/maximum x/y position of a line element or vacancy
# all output data is given in pixels
def parse(command):
    line_streaks = re.findall(reg_long_line, command)
    lines = []  # format is [[(xs,ys),(xe,ye)],...]
    red_lines = []  # format is [[(xs,ys),(xe,ye)],...]
    red_dotted_lines = []  # format is [[(xs,ys),(xe,ye)],...]
    min_x = min_y = float("inf")  # used to generate bounding box
    max_x = max_y = -float("inf")

    # line_streak is one complete line streak
    for line_streak in line_streaks:
        direction = (
            0  # 0 is in x direction, 1 in y direction, first same dir element in line streak is always in x direction
        )
        x, y = [
            round(float(num) * pixel_per_meter)
            for num in re.search(r"\((" + reg_num + "),(" + reg_num + r")\)", line_streak).groups()
        ]  # extract starting coordinate

        min_x, min_y, max_x, max_y = update_limits(x, y, min_x, min_y, max_x, max_y)  # update limits

        same_dir_elements = re.findall(
            reg_same_dir_element, line_streak
        )  # extract all same direction elements in this line streak
        # same_dir_element is one same direction element
        for same_dir_element in same_dir_elements:
            line_elements = re.findall(
                reg_wall + "|" + reg_empty, same_dir_element
            )  # extract all line elements and vacancies in this same dir element
            for line_element in line_elements:
                x_new = x
                y_new = y

                distance = round(
                    float(re.search(reg_num, line_element).group()) * pixel_per_meter
                )  # compute distance of this line element

                if direction == 0:  # compute end coordinate of this line element
                    x_new = x + distance
                else:
                    y_new = y + distance

                # decide what type of line element is given here
                # and convert line element in output format
                if (
                    re.match(reg_wall_red, line_element) and re.match(reg_wall, line_element).start() == 0
                ):  # this is a highlighted wall
                    red_lines.append([(x, y), (x_new, y_new)])
                elif (
                    re.match(reg_wall, line_element) and re.match(reg_wall, line_element).start() == 0
                ):  # this is a wall, should add
                    lines.append([(x, y), (x_new, y_new)])
                elif re.match(reg_empty_red, line_element):  # this is a vacancy, but highlighted
                    red_dotted_lines.append([(x, y), (x_new, y_new)])

                # update coordinates
                x = x_new
                y = y_new

                # update limits
                min_x, min_y, max_x, max_y = update_limits(x, y, min_x, min_y, max_x, max_y)
            direction = 1 if direction == 0 else 0  # switch direction for next same dir element
    return lines, red_lines, red_dotted_lines, min_x, min_y, max_x, max_y


# Verify a command string
# @param command The command string to test
# @return True if the command string is ok, False is not
def verify(command):
    match_object = re.match("^(" + reg_long_line + r"\s*)+$", command)
    if match_object is None:
        print("Your command is not correct!")
        return False
    else:
        print("Your command is verified")
        return True


# Paint the line elements, as returned from parse
# @param sizes The size in pixels of the complete image
# @param interpet_xy function to convert pixel coordinates in the position of the result image
def paint(lines, red_lines, red_dotted_lines, sizes, interpret_xy):
    im = Image.new("RGB", sizes, color=(255, 255, 255))  # create new image buffer
    draw = ImageDraw.Draw(im)  # create new drawer
    for line in lines:
        draw.line([interpret_xy(line[0]), interpret_xy(line[1])], fill=(0, 0, 0))  # create black line for normal wall
    for line in red_lines:
        draw.line(
            [interpret_xy(line[0]), interpret_xy(line[1])], fill=(255, 0, 0), width=3
        )  # create red line for highlighted wall
    for line in red_dotted_lines:
        line_start = np.array(interpret_xy(line[0]))  # starting position
        line_end = np.array(interpret_xy(line[1]))  # end position
        line_length = np.sqrt(np.sum((line_end - line_start) ** 2))  # length of line element
        line_fraction_per_dot = 4 / line_length  # results in a point every four pixels
        dots = [
            tuple(fraction * line_end + (1 - fraction) * line_start)
            for fraction in np.arange(0, 1, line_fraction_per_dot)
        ]  # positions of all dots
        bboxes = [
            [(round(dot[0] - 1), round(dot[1] - 1)), (round(dot[0] + 1), round(dot[1] + 1))] for dot in dots
        ]  # bounding boxes for all dots
        for bbox in bboxes:  # draw highlighted vacancy
            draw.ellipse(bbox, fill=(255, 0, 0))
    return im


# Obtain origin position in meter
# this follows ROS instructions, thus the position of the left bottom pixel relative to origin
def get_origin(sizes, interpret_xy):
    origin = list(interpret_xy((0, 0)))  # origin in pixel relative to left top pixel
    origin[1] = sizes[1] - origin[1]  # origin in pixel relative to left bottom pixel
    return tuple([-x / pixel_per_meter for x in origin])  # left bottom pixel in meter relative to origin


# Paint a complete command string, without verification first
# @return image The map image
# @return origin The origin of the map, needed for integration into ROS move_base and the Fawkes amcl plugin
def paint_command(command):
    lines, red_lines, red_dotted_lines, min_x, min_y, max_x, max_y = parse(command)
    sizes = [max_x - min_x + x_margin * 2, max_y - min_y + y_margin * 2]

    def interpret_xy(xy):
        return (round(xy[0] - min_x + x_margin), round(sizes[1] - (xy[1] - min_y + y_margin)))

    image = paint(lines, red_lines, red_dotted_lines, sizes, interpret_xy)
    origin = get_origin(sizes, interpret_xy)

    return image, origin


if __name__ == "__main__":

    cmd_line_args = sys.argv[1:]

    short_options = "hc:s:"
    options = ["help", "command=", "save="]

    try:
        args, _ = getopt.getopt(cmd_line_args, short_options, options)
    except getopt.GetoptError:  # Use getopt.error in Python 3
        print("Options could not be parsed")

    command = ""
    file_output = "map.png"
    print_help = False
    for arg, value in args:
        if arg in ["--help", "-h"]:
            print_help = True
        if arg in ["--command", "-c"]:
            command = value
        if arg in ["--save", "-s"]:
            file_output = value
    if len(args) == 0:
        print_help = True

    if print_help:
        print(
            "Usage: script [--command <command>] [--help] [--save <file>]\n\n"
            + "The <command> is parsed and the output is written to <file>\n"
            + "Default output is map.png\n\n"
            + "Each command consists of multiple line streaks. Each line streak\n"
            + "starts with a coordinate, e.g. (2,3) and continues with\n"
            + "lengths, which can either be painted (just the number without brackets)\n"
            + "or invisible (number inside rectangular brackets).\n"
            + "These lengths are applied first in x direction, switching into y direction\n"
            + "is done by a semicolon. Each line streak should be ended by a semicolon.\n\n"
            + "One example is shown in the file map_example.png. It was created using the command\n"
            + "(-7,0)3;1;[-1]-2;1[4.5]1.5;14;-1.5[-4.5]-1;-2[-1];-1;3;(-2,0)4;"
        )
        sys.exit(0)

    if command != "" and verify(command):
        image, origin = paint_command(command)
        image.save(file_output)
        print("origin is at " + str(origin))
