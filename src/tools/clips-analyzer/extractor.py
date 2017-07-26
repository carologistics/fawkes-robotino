#!/usr/bin/env python3

import argparse
import re
import sys
import sqlite3

db_games = sqlite3.connect('data/analyzer')
cursor = db_games.cursor()
cursor.execute('''CREATE TABLE IF NOT EXISTS games_meta(id INTEGER PRIMARY KEY, game_table_name TEXT)''')
db_games.commit()


# self.cont = content
# self.relc = rel_line_counter
# self.absc = abs_line_counter
# self.attr = set([])
# self.time = time
# self.game_time = None
# self.name = None
def create_games_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "(id INTEGER PRIMARY KEY,cont TEXT, relc INTEGER, absc INTEGER,"
                + " attr TEXT, time INTEGER, gametime INTEGER, name TEXT)")
    tb_exists = "SELECT name FROM sqlite_master WHERE type='table' AND name='games_meta'"
    if not crs.execute(tb_exists).fetchone():
        crs.execute("INSERT INTO game" + str(number) + "(game_table_name) VALUES(?)", ("game" + str(number)))
    db.commit()


def to_seconds(hour, minute, second):
    return hour * 3600 + minute * 60 + second


def convert_time(starttime, time):
    if starttime <= time:
        return time - starttime
    else:
        return 86400 + time - starttime


class Line:
    """A simple class to save attributes"""

    def __init__(self, content, rel_line_counter, abs_line_counter, time=None):
        self.cont = content
        self.relc = rel_line_counter
        self.absc = abs_line_counter
        self.attr = None
        self.time = time
        self.game_time = None
        self.name = None
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
        if re.search(r"BBCLIPS", self.cont) is not None:
            self.attr = 'bb_msg'

    def check_for_fire(self):
        if re.search(r"FIRE", self.cont) is not None:
            self.attr = 'fire'
            rule = re.search(r"FIRE\s*[0-9]*\s*([\w\-]*)", self.cont)
            self.name = rule.group(1)

    def check_for_retract(self):
        if re.search(r"==>", self.cont) is not None:
            self.attr = 'retract'

    def check_for_assert(self):
        if re.search(r"<==", self.cont) is not None:
            self.attr = 'assert'

    def check_for_calling_skill(self):
        if re.search(r"Calling skill", self.cont) is not None:
            self.attr = 'calling_skill'

    def check_for_trying(self):
        if re.search(r"Trying CLIPS", self.cont) is not None:
            self.attr = 'trying_to_load_clp'

    def check_for_found(self):
        if re.search(r"Found CLIPS", self.cont) is not None:
            self.attr = 'found_clp'

    def check_for_ff_feature_request(self):
        if re.search(r"Requesting.*feature", self.cont) is not None:
            self.attr = 'ff_feature_request'

    def check_for_sync_template(self):
        if re.search(r"Synchronizing template", self.cont) is not None:
            self.attr = 'sync_template'

    def check_for_skill_statechange(self):
        if re.search(r"Skill.*is.*was", self.cont) is not None:
            self.attr = 'skill_statechange'

    def return_time(self):
        time = re.search(r"[a-zA-Z]\s([0-9]*):([0-9]*):([0-9]*)", self.cont)
        self.time = to_seconds(int(time.group(1)), int(time.group(2)), int(time.group(3)))

    def strip_content(self):
        return re.sub(r"(.*agent\)*:)", '', self.cont)

    def calculate_gametime(self, starttime):
        self.game_time = convert_time(starttime, self.time)
        # print(self.game_time)


def extract_from_file(filename):
    # abs_line_counter = 0
    rel_line_counter = 0
    with open(filename, "r") as debug_file:
        game_list = []
        debug_list = []
        for i, line in enumerate(debug_file):
            abs_line_counter = i  # abs_line_counter +1
            rel_line_counter = rel_line_counter + 1
            line = line.lstrip().rstrip()
            line = re.search('.*CLIPS.*', line)
            if line is not None:
                if (re.search(r'Trying CLIPS file .*/fawkes-robotino/fawkes/src/plugins/clips/clips/ff-config.clp',
                              line.group()) is not None):
                    if debug_list:
                        game_list.append(debug_list)
                        debug_list = []
                        debug_list.append(Line(line.group(), rel_line_counter, abs_line_counter))
                        rel_line_counter = 0
                    else:
                        debug_list.append(Line(line.group(), rel_line_counter, abs_line_counter))
                        rel_line_counter = rel_line_counter + 1
                        # rel_line_counter = 1
                        # game_list.append(debug_list)
                        # debug_list=[]
                        # debug_list.append(Line(line.group(),rel_line_counter,abs_line_counter))

                else:
                    debug_list.append(Line(line.group(), rel_line_counter, abs_line_counter))

        game_list.append(debug_list)
    return game_list


def interactive(game_list):
    if len(game_list) == 1:
        print("There is 1 game in this logfile: " + filename)
    else:
        print("There are " + str(len(game_list)) + " games in this logfile: " + filename)

    print("\n")
    print("Please use the flag -h or --help for how this programm can be used\n")
    print("")


filename = "../../../bin/debug1.log"
game_list = extract_from_file(filename)
print("You can operate on these games as following:\n")
print("")

game_list = extract_from_file(filename)
interactive(game_list)
start = False
retract_dict = {}
assert_dict = {}
switch = None
game_number = 2
starttime = 0

starttime = game_list[game_number][0].time
for i in range(len(game_list)):
    for line in game_list[i]:
        line.calculate_gametime(starttime)
for line in game_list[game_number]:
    if "fire" == line.attr:
        pass
        # print(str(line.game_time) + " sec: Rule " + line.name)


def show_rules_around(game, game_time, timediff=0):
    time_min = game_time - timediff
    time_max = game_time + timediff

    for line in game:
        if "fire" == line.attr:
            if time_min <= line.game_time <= time_max:
                print(str(line.game_time) + " sec: Rule " + line.name)


game = game_list[game_number]
#show_rules_around(game, 123, 100)


def show_times_when_rule_fired(game, rulename):
    print("Rule " + rulename + " was fired at these times:")
    for line in game:
        if line.name is not None:
            if rulename in line.name:
                print(str(line.game_time) + " sec = " + str(line.game_time//60)+" min and "
                      +str(line.game_time%60)+" sec -> id: " + str(line.relc))


#print("Startzeit: " + str(starttime))
for i, line in enumerate(game_list[game_number]):
    # print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.cont)
    # print("Relative line: "+ str(line.relc) + ", absolute line: "+ str(line.absc)+ ", attribute: "+ str(line.attr) +", content: "+ line.stripped_content)
    if 'f-0' in line.stripped_content:
        start = True
    if start:
        if 'assert' == line.attr or 'retract' == line.attr:
            switch = 'assert'
            temp = re.search('(<\=\=\s)(f\-[0-9]*)\s*(.*)', line.stripped_content)
            if temp == None:
                switch = 'retract'
                temp = re.search('(\=\=>\s)(f\-[0-9]*)\s*(.*)', line.stripped_content)

            a, b = temp.group(2), temp.group(3)
            fused = {i: [a, b]}
            if switch == 'assert':
                assert_dict.update(fused)
            else:
                retract_dict.update(fused)


# print(assert_dict)
# print(retract_dict)
def current_factbase(assert_dict, retract_dict, point_in_time):
    factbase = []
    for timestep in range(point_in_time + 1):
        if timestep in assert_dict:
            factbase.append(assert_dict[timestep])
        elif timestep in retract_dict:
            if retract_dict[timestep] in factbase:
                factbase.remove(retract_dict[timestep])
    return factbase
    # print ('Facts')
    # for f_ in factbase:
    #    print(f_[1])


# current_factbase(assert_dict,retract_dict,1500)

# Was stand wann und wann in der factbase
# Filter: Positivliste
# -c contains
# -s slotname
# -g game
# -i info
# -t time counted from game start
# -T time absolute
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--contains", type=str, help="Filters whether a keyword is in the output")
parser.add_argument("-s", "--slotname", type=str, help="Filters for a specific slotname")
parser.add_argument("-g", "--game", type=int, help="Shows a specific game")
parser.add_argument("-l", "--list", action="store_true", help="Lists all games found")
parser.add_argument("-t", "--time", type=int, help="Shows factbase at a specific point in time, counted from the start")
parser.add_argument("-T", "--Time", type=int, help="Shows rules at a given time")
parser.add_argument("-d", "--diff", type=int, help="Provides a certain fuzziness")
parser.add_argument("-r", "--rule", type=str, help="Shows time of rules fired")
parser.add_argument("-i", "--id", type=int, help="Shows factbase corresponding to this id ")
args = parser.parse_args()


def list_games():
    print("There are " + str(len(game_list)) + " games that can be found in " + filename)
    print("\nFor more information use -h or --help")


def contains(cfilter):
    fbase = current_factbase(assert_dict, retract_dict, 1500)
    print(fbase)


def slotname(sfilter):
    pass


if args.list:
    list_games()
if args.game is not None:
    if type(args.game) is int and args.game >= 0:
        if args.rule is not None:
            show_times_when_rule_fired(game_list[args.game], args.rule)
        if args.Time is not None:
            difftime = 0
            if args.diff is not None:
                difftime = args.diff
            show_rules_around(game_list[args.game], args.Time, difftime)
        if args.id is not None:
            fb = current_factbase(assert_dict, retract_dict, args.id)
            if args.contains is not None:
                new_fb = list(filter(lambda x: args.contains in x[1], fb))
                for f in new_fb:
                    print(f[0] + " : " + f[1])
            else:
                for f in fb:
                    print(f[0] + " : " + f[1])



