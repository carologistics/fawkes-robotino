#!/usr/bin/env python3


#***************************************************************************
#*  Script to generate map after measuring lengths physically
#*
#*  Created:  May 10 16:30:00 2019
#*  Copyright  2019  Morian Sonnet [Morian.Sonnet@rwth-aachen.de]
#*
#****************************************************************************

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


from PIL import ImageDraw, Image

import re # for verifying correct input string

import sys
import getopt

reg_num = r"[-+]?\d+(?:\.\d*)?"
reg_wall = reg_num
reg_empty = r"\[" + reg_num + "\]"
reg_coordinate = r"\("+reg_num+r","+reg_num+r"\)"
reg_same_dir_element = r"(?:(?:"+ reg_empty +r"|" +reg_wall+r"|)+;)"
reg_long_line = reg_coordinate+ reg_same_dir_element+"+"

pixel_per_meter=20
x_margin=10
y_margin=10

def update_limits(x,y,min_x,min_y, max_x, max_y):
    return min(x,min_x), min(y,min_y), max(x,max_x), max(y, max_y)
    

def parse(command):
    part_commands = re.findall(reg_long_line,command)
    lines = [] # format is [[(xs,ys),(xe,ye)],...]
    min_x = min_y = float("inf")      # used to generate bounding box    
    max_x = max_y = -float("inf")
    
    for part_command in part_commands:
        dir = 0 # 0 is in x direction, 1 in y direction
        x, y = [round(float(num)*pixel_per_meter) for num in re.search("\(("+reg_num+"),("+reg_num+")\)",part_command).groups()]
        min_x, min_y, max_x, max_y = update_limits(x,y,min_x,min_y, max_x, max_y) 
        same_dir_elements = re.findall(reg_same_dir_element,part_command)
        for same_dir_element in same_dir_elements:
            for line_element in re.findall(reg_wall+"|"+reg_empty,same_dir_element):
                x_new = x
                y_new = y
                distance = round(float(re.search(reg_num,line_element).group())*pixel_per_meter)
                if dir == 0:
                    x_new = x+distance
                else:
                    y_new = y+distance
                
                if re.match(reg_empty,line_element)==None: # this is a wall, should add
                    lines.append([(x,y),(x_new,y_new)])
                x = x_new
                y = y_new
                min_x, min_y, max_x, max_y = update_limits(x,y,min_x,min_y, max_x, max_y)
            dir = 1 if dir == 0 else 0
    return lines, min_x, min_y, max_x, max_y 
        
def verify(command):
    match_object = re.match("^("+reg_long_line+"\s*)+$", command)
    if(match_object==None):
        print("Your command is not correct!")
        return False
    else:
        print("Your command is verified")
        return True
    
def paint(lines, sizes, interpret_xy):
    im = Image.new('1',sizes,color=1)
    draw = ImageDraw.Draw(im)
    for line in lines:
        draw.line([interpret_xy(line[0]),interpret_xy(line[1])],fill=0)
    return im

def get_origin(sizes,interpret_xy):
    origin = list(interpret_xy((0,0))) #origin in pixel relative to left top pixel
    origin[1] = sizes[1]-origin[1] # origin in pixel relative to left bottom pixel
    return  tuple([-x/pixel_per_meter for x in origin]) # left bottom pixel in meter relative to origin

    
if __name__ == "__main__":
    
    cmd_line_args = sys.argv[1:]
    
    short_options = 'hc:s:'
    options = ["help", "command=","save="]
    
    try:
        args,_ = getopt.getopt(cmd_line_args, short_options, options)
    except:
        print("Options could not be parsed")
    
    command = ""
    file_output = "map.png"
    for arg,value in args:
        if arg in ['--help','-h']:
            print("Usage: script [--command <command>] [--help] [--save <file>]\n\n"+
                    "The <command> is parsed and the output is written to <file>\n"+
                    "Default output is map.png\n\n"+
                    "Each command consists of multiple line streaks. Each line streak\n"+
                    "starts with a coordinate, e.g. (2,3) and continues with\n"+
                    "lengths, which can either be painted (just the number without brackets)\n"+
                    "or invisible (number inside rectangular brackets).\n"+
                    "These lengths are applied first in x direction, switching into y direction\n"+
                    "is done by a semicolon. Each line streak should be ended by a semicolon.\n\n"+
                    "One example is shown in the file map_example.png. It was created using the command\n"+
                    "(-7,0)3;1;[-1]-2;1[4.5]1.5;14;-1.5[-4.5]-1;-2[-1];-1;3;(-2,0)4;")
            sys.exit(0)
        if arg in ['--command','-c']:
            command = value
        if arg in ['--save','-s']:
            file_output = value
            
    
    if command != "" and verify(command):
        lines, min_x, min_y, max_x, max_y = parse(command)
        sizes = [max_x-min_x + x_margin*2, max_y-min_y + y_margin*2]    
        interpret_xy = lambda xy: (xy[0]-min_x+x_margin,sizes[1]-(xy[1]-min_y+y_margin))
        image = paint(lines, sizes, interpret_xy)
        image.save(file_output)
        origin = get_origin(sizes,interpret_xy)
        print("origin is at "+str(origin))
    

    
