import os
import sys
import re

class Line:
    """A simple class to save attributes"""
    def __init__(self,content,rel_line_counter,abs_line_counter):
        self.cont = content
        self.relc = rel_line_counter
        self.absc = abs_line_counter
        self.attr = set([])
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
           rel_line_counter = rel_line_counter +1
           line = line.lstrip().rstrip()
           line = re.search('.*CLIPS.*',line)
           if( line != None ):
               if(re.search(r'Trying CLIPS file .*/fawkes-robotino/fawkes/src/plugins/clips/clips/ff-config.clp',line.group())!=None):
                   if(debug_list!=[]):
                       rel_line_counter = 1
                       game_list.append(debug_list)
                       debug_list=[]
                       debug_list.append(Line(line.group(),rel_line_counter,abs_line_counter))
                       
               else:
                    debug_list.append(Line(line.group(),rel_line_counter,abs_line_counter))


        game_list.append(debug_list)
    return game_list

def interactive(game_list):
    if(len(game_list)==1):
        print("There is 1 game in this logfile: "+ filename)
    else:
        print("There are "+ str(len(game_list)) + " games in this logfile: "+ filename)
    
    print("\n")
    print("You can operate on these games as following:\n")
    print("")


if(len(sys.argv)==1):
    filename = "../../../bin/debug1.log"
else:
    filename=sys.argv[1]
game_list = extract_from_file(filename)
interactive(game_list)
start = False
retract_dict = {}
assert_dict = {}
switch = None
for i,line in enumerate(game_list[0]):
#    print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.cont)
#    print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.stripped_content)
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
#print(assert_dict)
#print(retract_dict)
def current_factbase(assert_dict, retract_dict, point_in_time):
    factbase = []
    for timestep in range(point_in_time+1):
        if timestep in assert_dict:
            factbase.append(assert_dict[timestep])
        elif timestep in retract_dict:
            if retract_dict[timestep] in factbase:
                factbase.remove(retract_dict[timestep])
    print ('Facts')
    for f_ in factbase:
        print(f_[1])
current_factbase(assert_dict,retract_dict,1500)

# Was stand wann und wann in der factbase
# Filter: Positivliste
# -c contains
# -s slotname
# -g game
# -i info
# -t time counted from game start
# -T time absolute
