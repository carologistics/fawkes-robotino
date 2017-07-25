#!/usr/bin/env python3

import os
import sys
import re

import argparse

def to_seconds(hour, minute, second):
    return hour*3600+minute*60+second

def converttime(starttime, time):
    if starttime <=time:
        return time - starttime
    else:
        return 86400 + time - starttime


class Line:
    """A simple class to save attributes"""
    def __init__(self,content,rel_line_counter,abs_line_counter,time=None):
        self.cont = content
        self.relc = rel_line_counter
        self.absc = abs_line_counter
        self.attr = set([])
        self.time = time
        self.check_for_blackboard()
        self.check_for_fire()
        self.check_for_retract()
        self.check_for_assert()
        self.check_for_calling_skill()
        self.check_for_trying()
        self.check_for_found()
        self.check_for_ff_feature_request()
        self.check_for_sync_template()
        self.check_for_skill_statechange()
        
        self.return_time()


        self.stripped_content = self.strip_content()

    def check_for_blackboard(self):
        if(re.search(r"BBCLIPS",self.cont)!= None):
            self.attr.add('bb_msg')
    def check_for_fire(self):
        if(re.search(r"FIRE",self.cont)!= None):
            self.attr.add('fire')
    def check_for_retract(self):
        if(re.search(r"==>",self.cont)!= None):
            self.attr.add('retract')
    def check_for_assert(self):
        if(re.search(r"<==",self.cont)!= None):
            self.attr.add('assert')
    def check_for_calling_skill(self):
        if(re.search(r"Calling skill",self.cont)!= None):
            self.attr.add('calling_skill')
    def check_for_trying(self):
        if(re.search(r"Trying CLIPS",self.cont)!= None):
            self.attr.add('trying_to_load_clp')
    def check_for_found(self):
        if(re.search(r"Found CLIPS",self.cont)!= None):
            self.attr.add('found_clp')
    def check_for_ff_feature_request(self):
        if(re.search(r"Requesting.*feature",self.cont)!= None):
            self.attr.add('ff_feature_request')
    def check_for_sync_template(self):
        if(re.search(r"Synchronizing template",self.cont)!= None):
            self.attr.add('synch_template')
    def check_for_skill_statechange(self):
        if(re.search(r"Skill.*is.*was",self.cont)!= None):
            self.attr.add('skill_statechange')
    
    def return_time(self):
        time = re.search(r"[a-zA-Z]\s([0-9]*):([0-9]*):([0-9]*)",self.cont)
        self.time = to_seconds(time.group(1),time.group(2),time.group(3))
        print(str(self.time)+"\n")
    
    def strip_content(self):
        return re.sub(r"(.*agent\)*:)",'',self.cont)

def extract_from_file(filename):
    #abs_line_counter = 0
    rel_line_counter = 0
    with open(filename, "r") as debug_file:
        game_list = []
        debug_list = []
        for i,line in enumerate(debug_file):
           abs_line_counter = i #abs_line_counter +1
           line = line.lstrip().rstrip()
           line = re.search('.*CLIPS.*',line)
           if( line != None ):
               if(re.search(r'Trying CLIPS file .*/fawkes-robotino/fawkes/src/plugins/clips/clips/ff-config.clp',line.group())!=None):
                   if(debug_list!=[]):
                       game_list.append(debug_list)
                       debug_list=[]
                       debug_list.append(Line(line.group(),rel_line_counter,abs_line_counter))
                       rel_line_counter = 0
                       
               else:
                    debug_list.append(Line(line.group(),rel_line_counter,abs_line_counter))
                    rel_line_counter = rel_line_counter +1


        game_list.append(debug_list)
    return game_list

def interactive(game_list):
    if(len(game_list)==1):
        print("There is 1 game in this logfile: "+ filename)
    else:
        print("There are "+ str(len(game_list)) + " games in this logfile: "+ filename)
    
    print("\n")
    print("Please use the flag -h or --help for how this programm can be used\n")
    print("")

filename = "../../../bin/debug1.log"
#if(len(sys.argv)==1):
#    filename = "../../../bin/debug1.log"
#else:
#    filename=sys.argv[1]
game_list = extract_from_file(filename)
#interactive(game_list)
start = False
retract_dict = {}
assert_dict = {}
switch = None
game_number=0
starttime = 0
starttime = to_seconds(13,14,45)#game_list[game_number][0].time
print("Startzeit: "+ str(starttime))
for i,line in enumerate(game_list[game_number]):
    #print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.cont)
    #print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.stripped_content)
    if 'f-0' in line.stripped_content:
        start=True
    if start:
        if 'assert' in line.attr or 'retract' in line.attr:
            switch = 'assert'
            temp = re.search('(<\=\=\s)(f\-[0-9]*)\s*(.*)',line.stripped_content)
            if temp == None:
                switch = 'retract'
                temp = re.search('(\=\=>\s)(f\-[0-9]*)\s*(.*)',line.stripped_content)

            a,b = temp.group(2), temp.group(3)
            fused = {i:[a,b]}
            if switch == 'assert':
                assert_dict.update(fused)
            else:
                retract_dict.update(fused)

def current_factbase(assert_dict, retract_dict, point_in_time):
    factbase = []
    for timestep in range(point_in_time+1):
        if timestep in assert_dict:
            factbase.append(assert_dict[timestep])
        elif timestep in retract_dict:
            if retract_dict[timestep] in factbase:
                factbase.remove(retract_dict[timestep])
    return factbase
    #print ('Facts')
    #for f_ in factbase:
    #    print(f_[1])
#current_factbase(assert_dict,retract_dict,1500)

# Was stand wann und wann in der factbase
# Filter: Positivliste
# -c contains
# -s slotname
# -g game
# -i info
# -t time counted from game start
# -T time absolute
parser=argparse.ArgumentParser()
parser.add_argument("-c", "--contains", type=str, help="Filters whether a keyword is in the output")
parser.add_argument("-s", "--slotname", type=str, help="Filters for a specific slotname")
parser.add_argument("-g", "--game", type=int, help="Shows a specific game")
parser.add_argument("-l", "--list", action="store_true", help="Lists all games found")
parser.add_argument("-t", "--time", type=int, help="Shows factbase at a specific point in time, counted from the start")
parser.add_argument("-T", "--Time", type=int, help="Shows factbase at a specific point in time")
args = parser.parse_args()



def list_games():
    print("There are " + str(len(game_list))+ " games that can be found in " + filename)
    print("\nFor more information use -h or --help")
def contains(cfilter):
    fbase = current_factbase(asser_dict, retract_dict, 1500) 
    print(fbase)

def slotname(sfilter):
    pass

if args.list:
    list_games()
if args.game!=None:
    if type(args.game) is int and args.game >= 0:
        current_factbase(assert_dict,retract_dict,1500)

