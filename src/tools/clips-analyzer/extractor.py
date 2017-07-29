#!/usr/bin/env python3

import argparse
import re
import sys
import sqlite3

db_games = sqlite3.connect('data/analyzer')
cursor = db_games.cursor()
cursor.execute('''CREATE TABLE IF NOT EXISTS games_meta(id INTEGER PRIMARY KEY, game_table_name TEXT)''')
db_games.commit()


def create_games_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "(id INTEGER PRIMARY KEY,cont TEXT, relc INTEGER, absc INTEGER,"
                + " attr TEXT, time INTEGER, gametime INTEGER, name TEXT)")
    tb_exists = "SELECT name FROM sqlite_master WHERE type='table' AND name='games_meta'"
    if not crs.execute(tb_exists).fetchone():
        crs.execute("INSERT INTO games_meta(game_table_name) VALUES(?);", ("game" + str(number)))
    db.commit()


def create_games_asserts_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "_asserts(id INTEGER PRIMARY KEY,name TEXT, content TEXT, relc INTEGER)")
    db.commit()


def create_games_retracts_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "_retracts(id INTEGER PRIMARY KEY,name TEXT, content TEXT, relc INTEGER)")
    db.commit()

def create_games_factbase_changes_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "_factbase_changes(id INTEGER PRIMARY KEY,name TEXT, content TEXT, relc INTEGER, type TEXT)")
    db.commit()

def create_games_rules_fired_table(db, crs, number):
    crs.execute("CREATE TABLE IF NOT EXISTS game" + str(
        number) + "_rules_fired(id INTEGER PRIMARY KEY,name TEXT, gametime INTEGER, relc INTEGER, facts TEXT)")
    db.commit()


def merge_list_of_lists_with_order(list1, list2, field):
    new_list = []
    pos_temp = 0
    pos1, pos2 = 0, 0
    #Merged list nach relc.

    for pos1 in range(len(list1)):
        for pos2 in range(pos_temp, len(list2)):
            if list1[pos1][field] <= list2[pos2][field]:
                new_list.append(list1[pos1])
                break
            else:
                new_list.append(list2[pos2])
                pos_temp = pos2 + 1
    return new_list


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
            self.attr = 'assert'

    def check_for_assert(self):
        if re.search(r"<==", self.cont) is not None:
            self.attr = 'retract'

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
                    rel_line_counter = 1;
                    if len(debug_list) > 0:
                        game_list.append(debug_list)
                        debug_list = [Line(line.group(), rel_line_counter, abs_line_counter)]

                        # rel_line_counter = 0
                        # else:
                        #     debug_list = [Line(line.group(), rel_line_counter, abs_line_counter)]
                        #     rel_line_counter = rel_line_counter + 1

                else:
                    debug_list.append(Line(line.group(), rel_line_counter, abs_line_counter))

        game_list.append(debug_list)
    return game_list


def interactive(game_list):
    if len(game_list) == 1:
        print("There is 1 game in this logfile: " + filename)
    else:
        print("There are " + str(len(game_list)) + " games in this logfile: " + filename)
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


def show_rules_around(current_game, game_time, timediff=0):
    time_min = game_time - timediff
    time_max = game_time + timediff

    for g in current_game:
        if "fire" == g.attr:
            if time_min <= g.game_time <= time_max:
                print(str(g.game_time) + " sec: Rule " + g.name)


# game = game_list[game_number]

for i in range(len(game_list)):
    create_games_table(db_games, cursor, i)
    create_games_asserts_table(db_games, cursor, i)
    create_games_retracts_table(db_games, cursor, i)
    create_games_factbase_changes_table(db_games, cursor, i)
    create_games_rules_fired_table(db_games, cursor, i)
    cursor.execute("INSERT INTO games_meta(game_table_name) VALUES(?)", ("game" + str(i),))
    db_games.commit()

for i in range(len(game_list)):
    starttime = game_list[i][0].time
    for line in game_list[i]:
        line.calculate_gametime(starttime)
        if line.name is not None:
            cursor.execute("INSERT INTO  game" + str(i) + "(cont, relc, absc, attr, time, gametime, name)" \
                                                          "VALUES(?,?,?,?,?,?,?)", \
                           (line.cont, line.relc, line.absc, line.attr, line.time, line.game_time, line.name))
        else:
            cursor.execute("INSERT INTO  game" + str(i) + "(cont, relc, absc, attr, time, gametime)" \
                                                          "VALUES(?,?,?,?,?,?)", \
                           (line.cont, line.relc, line.absc, line.attr, line.time, line.game_time))
    db_games.commit()

for i in range(len(game_list)):
    t = ('assert',)
    fact_list = cursor.execute('SELECT * FROM game' + str(i) + ' WHERE attr=?', t)
    assert_list = []
    switch = False
    for fact in fact_list:
        f = re.search(r"==>\s*(f-[0-9]*)\s*\((.*)\)", fact[1])
        if "f-0" == f.group(1):
            switch = True
        if switch:
            assert_list.append((f.group(1), f.group(2), fact[2]))
    cursor.executemany("INSERT INTO  game" + str(i) + "_asserts(name, content, relc) VALUES(?,?,?)", assert_list)
    db_games.commit()

    t = ('retract',)
    fact_list = cursor.execute('SELECT * FROM game' + str(i) + ' WHERE attr=?', t)
    retract_list = []
    switch = False
    for fact in fact_list:
        f = re.search(r"<==\s*(f-[0-9]*)\s*\((.*)\)", fact[1])
        if "f-0" == f.group(1):
            switch = True
        if switch:
            retract_list.append((f.group(1), f.group(2), fact[2]))
    cursor.executemany("INSERT INTO  game" + str(i) + "_retracts(name, content, relc) VALUES(?,?,?)", retract_list)
    db_games.commit()

    t = ('fire',)
    temp_list = cursor.execute('SELECT * FROM game' + str(i) + ' WHERE attr=?', t)
    rule_list = []
    for rule in temp_list:
        facts = re.search(r"agent\):.*:\s*(.*)", rule[1])
        rule_list.append((rule[7], rule[2], rule[6], facts.group(1)))
    cursor.executemany("INSERT INTO  game" + str(i) + "_rules_fired(name, relc, gametime, facts) VALUES(?,?,?,?)",
                       rule_list)
    db_games.commit()

#Create assert and retract table
for i in range(len(game_list)):
    a = ('assert',)
    r = ('retract',)
    asserts = cursor.execute('SELECT * FROM game' + str(i) + "_asserts ORDER BY relc ")
    new_asserts = [list(x+('assert',)) for x in asserts]
    retracts = cursor.execute('SELECT * FROM game' + str(i) + "_retracts ORDER BY relc")
    new_retracts = [list(x+('retract',)) for x in retracts]
    merged_list = merge_list_of_lists_with_order(new_asserts, new_retracts, 3)
    new_merged_list = [x[1:] for x in merged_list]
    start = 0
    for j, m in enumerate(new_merged_list):
        if m[-1] == 'assert':
            start = j
            break
    new_merged_list = new_merged_list[start:]
    cursor.executemany("INSERT INTO  game" + str(i) + "_factbase_changes(name, content, relc, type) VALUES(?,?,?,?)",
                       new_merged_list)
    db_games.commit()


def get_factbase_until_rule_fired(game,crs, rule_id):
    r = crs.execute('SELECT * FROM '+game+'_rules_fired WHERE id = '+str(rule_id))
    r = [x[3] for x in r]
    for i,x in enumerate(r):
        pass
    changes = crs.execute('SELECT * FROM '+game+'_factbase_changes WHERE relc < '+ str(r[0]))
    factbase = []
    for change in changes:
        if 'assert' == change[4]:
            factbase.append(change[2])
        if 'retract' == change[4]:
            factbase.remove(change[2])
    return factbase


def show_times_when_rule_fired(game, rulename):
    print("Rule " + rulename + " was fired at these times:")
    for g in game:
        if g.name is not None:
            if rulename in g.name:
                print(str(g.game_time) + " sec = " + str(g.game_time // 60) + " min and "
                      + str(g.game_time % 60) + " sec -> id: " + str(g.relc))


for i, line in enumerate(game_list[game_number]):
    if 'f-0' in line.stripped_content:
        start = True
    if start:
        if 'assert' == line.attr or 'retract' == line.attr:
            switch = 'assert'
            temp = re.search('(<\=\=\s)(f\-[0-9]*)\s*(.*)', line.stripped_content)
            if temp is None:
                switch = 'retract'
                temp = re.search('(\=\=>\s)(f\-[0-9]*)\s*(.*)', line.stripped_content)

            a, b = temp.group(2), temp.group(3)
            fused = {i: [a, b]}
            if switch == 'assert':
                assert_dict.update(fused)
            else:
                retract_dict.update(fused)


# print(assert_dict)

def current_factbase(assert_dictionary, retract_dictionary, point_in_time):
    factbase = []
    for timestep in range(point_in_time + 1):
        if timestep in assert_dictionary:
            factbase.append(assert_dictionary[timestep])
        elif timestep in retract_dictionary:
            if retract_dictionary[timestep] in factbase:
                factbase.remove(retract_dictionary[timestep])
    return factbase


# Was stand wann und wann in der factbase
# Filter: Positivliste
# -c contains
# -s slotname
# -g game
# -i info
# -t time counted from game start
# -T time absolute
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--filter", type=str, help="Filters whether a keyword is in the output")
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
            if args.filter is not None:
                new_fb = list(filter(lambda x: args.filter in x[1], fb))
                for f in new_fb:
                    print(f[0] + " : " + f[1])
            else:
                for f in fb:
                    print(f[0] + " : " + f[1])
