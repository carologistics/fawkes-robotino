##########################################################################
#
#  gamestate_loader.py: manage recorded games
#
#  Copyright © 2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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


"""
Import and export recorded games from fawkes robotino. 
List recorded games, find the starttime, find times to jump into.
Create gamereports for the refbox configuration and config files
for fawkes robotino. 
"""

import pymongo
import bson
import subprocess
import os
import datetime
import signal
import argparse
import sys
from time import perf_counter
from multiprocessing import Process
from datetime import timezone
from copy import deepcopy

recovery_coll_name = "gamestate_recovery"


def export_db(client, game, filename):
    print("Exporting game "+filename+" ...")
    coll = client["fawkes"][recovery_coll_name]
    results = coll.find({"game": game})

    export = client["fawkes"]["gamestate_recovery_export"]
    for result in results:
        result.pop("_id")
        x = export.insert_one(result)

    mongodump = subprocess.Popen(['mongodump', '--host', '"localhost"', '--port', '"27050"',
                                 '--db', '"fawkes"', '--collection', '"gamestate_recovery_export"', '--out', filename])
    mongodump.wait()

    client["fawkes"].drop_collection("gamestate_recovery_export")
    print("Export completed!")


def import_db(client, filename):
    client["fawkes"].drop_collection("gamestate_recovery_export")
    print("Restoring from "+filename+" ...")
    mongorestore = subprocess.Popen(
        ['mongorestore', filename, "--host", '"localhost"', '--port', '"27050"'])
    mongorestore.wait()

    print("Copying into main collection")
    coll = client["fawkes"][recovery_coll_name]
    export = client["fawkes"]["gamestate_recovery_export"]
    results = export.find({})

    for result in results:
        result.pop("_id")
        x = coll.insert_one(result)

    client["fawkes"].drop_collection("gamestate_recovery_export")
    print("Import completed!")


def add_empty_strings_to_array(arr, nmbr):
    for i in range(0, nmbr):
        arr.append("")


def parse_fact_args(fact):
    fact["args"] = {}
    if "?" in fact["id"]:
        args = fact["id"].split("?")[1].split("&")
        for arg in args:
            fact["args"][arg.split("=")[0]] = arg.split("=")[1]


def parse_refbox_order_fact(fact):
    parsed = {}
    parsed["id"] = int(fact["id"].split("/")[3][1:])
    parsed["field"] = fact["id"].split("/")[4]
    parsed["value"] = fact["value"]
    return parsed


def get_machine_arr_index(arr, name):
    ind = 0
    for m in arr:
        if "name" in m and m["name"] == name:
            return ind
        ind += 1
    return ind


def test_instance():
    client = pymongo.MongoClient(
        "mongodb://localhost:27050/", connectTimeoutMS=20)
    client.list_database_names()


def check_connection(client):
    print("testing connection")
    try:
        if "fawkes" in client.list_database_names():
            if recovery_coll_name in client["fawkes"].list_collection_names():
                return True
            else:
                print("No saved games could be found, maybe you need to restore first")
                return True
        return False
    except pymongo.errors.ServerSelectionTimeoutError:
        return False


# argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("mode", nargs=1,  choices=["list", "suggest", "starttime", "create", "export", "import"],
                    help="mode under which the script is supposed to run. list: lists the available games; suggest: suggests times based on rule firings (default 3s around the time the game mode ist switched to running);create: creates a gamereport, config files and database dumps; import/export: import and export entire games for later use")
parser.add_argument(
    "-p", "--path", help="alternative path to the mongodb directory")
parser.add_argument(
    "-t", "--time", help="[suggest] time around which to search (default game start)")
parser.add_argument("-d", "--delta", type=int,
                    help="[suggest] delta in seconds for the search (default 3s)")
parser.add_argument(
    "-g", "--game", help="[suggest,starttime,create,export] game on which to execute the action (default latest)")
parser.add_argument(
    "-s", "--phase", help="[starttime] select the phase for which to find the starttime (default RUNNING)")
parser.add_argument("-i", "--importp",
                    help="[import] path for importing a gamelog")
args = parser.parse_args()

# set defaults
db_path = "/tmp/mongodb-robot-memory-local-1"
search_delta = 1
search_phase = "PRODUCTION"
timeout = 5
importp = "game_export_test"

# overwrite defaults if given as argument
if args.path:
    db_path = args.path
if args.delta:
    search_delta = args.delta
if args.phase:
    search_phase = args.phase.upper()
if args.importp:
    importp = args.importp

# create the mongod instance
mongod = subprocess.Popen(['mongod', '--bind_ip', '0.0.0.0', '--port',
                          '27050', '--dbpath', db_path], stdout=subprocess.DEVNULL)

# test the connection
p = Process(target=test_instance)
p.start()
p.join(timeout)
if p.is_alive():
    print("Timeout, couldn't start Mongod instance, check path")
    p.kill()
    exit(0)

# connect to the instance
try:
    client = pymongo.MongoClient(
        "mongodb://localhost:27050/", connectTimeoutMS=20)
except pymongo.errors.ServerSelectionTimeoutError:
    print("Couldn't connect to mongodb instance, maybe the path was incorrect?", file=sys.stderr)
    exit(0)

# check if the instance is conforming
if not check_connection(client):
    if args.path:
        print("This database looks wrong, check your arguments!", file=sys.stderr)
    else:
        print("This database looks wrong, are you sure it is on the default path?", file=sys.stderr)
else:
    print("Database and configuration looks good!\n\n")


#import mode
if "import" in args.mode:
    import_db(client, importp)
else:
    # get the collection
    coll = client["fawkes"][recovery_coll_name]

    # define the game
    try:
        game = coll.find({}).distinct("game")[-1].isoformat(sep=" ")
        if args.game:
            game = args.game
        game = datetime.datetime.fromisoformat(game)
    except:
        print("No games could be found, you can only use import mode")

    # export mode
    if "export" in args.mode:
        export_db(client, game, game.isoformat()[:-3].replace("T", " ")+"Z")

    # list mode
    elif "list" in args.mode:
        results = coll.find({}).distinct("game")
        stringified_results = []
        print("Recorded games: ")
        for fact in results:
            print(fact)
            stringified_results.append(fact.isoformat(sep=" "))
    else:
        # define the time
        time = coll.find({"game": game, "gamephase": search_phase})[0]["set"]
        if args.time:
            time = datetime.datetime.fromisoformat(args.time)

        # starttime mode
        if "starttime" in args.mode:
            print("Starttimes for game phase "+search_phase)
            results = coll.find({"game": game, "gamephase": search_phase})
            print(game)
            for fact in results:
                print(fact["set"].isoformat(sep=" "))

        # suggest mode
        elif "suggest" in args.mode:
            print("Suggested timepoints and corresponding rules ")
            results = coll.find({"game": game, "fired": {'$lt': time+datetime.timedelta(
                seconds=search_delta), '$gte': time + datetime.timedelta(seconds=-search_delta)}})
            for fact in results:
                print(fact["fired"].isoformat(sep=" ") + " - " + fact["rule"])

        # create mode
        elif "create" in args.mode:
            gamereport = {
                "orders": [],
                "machines": [],
                "points": [],
                "total-points": [bson.Int64(0), bson.Int64(0)],
                "teams": ["Carologistics", ""],
                "phase-points-magenta": {"EXPLORATION": 0, "PRODUCTION": 0, "WHACK_A_MOLE_CHALLENGE": 0},
                "phase-points-cyan": {"EXPLORATION": 0, "PRODUCTION": 0, "WHACK_A_MOLE_CHALLENGE": 0},
                "ring-specs": [],
                "report-version": 1.0,
                "gamestate/SETUP": {
                    "refbox-mode": "STANDALONE",
                    "state": "RUNNING",
                    "prev-state": "WAIT_START",
                    "phase": "SETUP",
                    "prev-phase": "PRE_GAME",
                    "over-time": "FALSE",
                    "teams": ["Carologistics", ""],
                    "points": [0, 0],
                    "end-time": [0, 0]
                },
                "gamestate/PRODUCTION": {
                    "refbox-mode": "STANDALONE",
                    "state": "RUNNING",
                    "prev-state": "WAIT_START",
                    "phase": "PRODUCTION",
                    "prev-phase": "PRODUCTION",
                    "over-time": "FALSE",
                    "teams": ["Carologistics", ""],
                    "points": [0, 0],
                    "end-time": [0, 0]
                }
            }

            # prefill
            for i in range(0, 9):  # because the *9 operator creates linked copies ooof
                gamereport["orders"].append({"activate-at": 0, "quantity-delivered": [bson.Int64(0), bson.Int64(0)], "ring-colors": [
                ], "id": bson.Int64(i+1), "delivery-period": [bson.Int64(0), bson.Int64(0)], "active": "FALSE", "allow-overtime": "FALSE"})
                if i == 8:
                    gamereport["orders"][i]["allow-overtime"] = "TRUE"

            for team in ["MAGENTA", "CYAN"]:

                for name in ["-CS1", "-CS2", "-BS", "-DS", "-RS1", "-RS2", "-SS"]:
                    name = team[0]+name
                    gamereport["machines"].append({
                        "name": name,
                        "team": team,
                        "pose": [0, 0, 0],
                        "rotation": bson.Int64(0),
                        "productions": bson.Int64(0),
                        "proc-time": bson.Int64(0),
                        "down-period": [-1, -1],
                        "actual-lights": ["GREEN-ON", "YELLOW-ON", "RED-ON"],
                        "desired-lights": ["GREEN-ON", "YELLOW-ON", "RED-ON"],
                        "task": "nil",
                        "proc-start": 0,
                        "broken-reason": "",
                        "pose-time": [bson.Int64(0), bson.Int64(0)],
                        "prep-blink-start": 0,
                        "bases-used": bson.Int64(0),
                        "wait-for-product-since": 0,
                        "bs-color": "BASE_RED",
                        "ds-gate": bson.Int64(0),
                        "ds-order": bson.Int64(0),
                        "ds-last-gate": bson.Int64(0),
                        "ss-operation": "STORE",
                        "ss-shelf-slot": [bson.Int64(0), bson.Int64(0)],
                        "ss-wp-description": "",
                        "mps-busy": "FALSE",
                        "mps-ready": "TRUE",
                        "broken-since": 0,
                        "rs-ring-colors": [],
                        "bs-side": "INPUT",
                        "zone": "M_Z25"
                    })

            # time sensitive information
            results = coll.find({"game": game, "fact": {"$exists": True}, "asserted": {
                                "$lte": time}, "$or": [{"retracted": {"$gte": time}}, {"retracted": "FALSE"}]})

            for fact in results:
                parse_fact_args(fact["fact"])
                fact = fact["fact"]
                id = fact["id"]

                # time-sensitive order information
                if "/domain/fact/quantity-delivered" in id:
                    ind = 0
                    ind = 1 if fact["args"]["team"] == "MAGENTA" else ind
                    gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["quantity-delivered"][ind] = bson.Int64(int(fact["value"]))

                # time-sensitive machine information
                if "/domain/fact/mps-state" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["state"] = fact["args"]["s"]
                    if fact["args"]["s"] == "IDLE":
                        gamereport["machines"][get_machine_arr_index(
                            gamereport["machines"], fact["args"]["m"])]["idle-since"] = -1
                    else:
                        gamereport["machines"][get_machine_arr_index(
                            gamereport["machines"], fact["args"]["m"])]["idle-since"] = 0
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["prev-state"] = fact["args"]["s"]
                if "/domain/fact/rs-filled-with" in id:
                    n = fact["args"]["n"]
                    if n == "ZERO":
                        n = 0
                    if n == "ONE":
                        n = 1
                    if n == "TWO":
                        n = 2
                    if n == "THREE":
                        n = 3
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["mps-base-counter"] = bson.Int64(n)
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["bases-added"] = bson.Int64(n)
                if "/domain/fact/cs-buffered" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["cs-retrieved"] = fact["args"]["col"]
                if "/domain/fact/cs-can-perform" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["cs-operation"] = fact["args"]["op"]
                if "/domain/fact/rs-prepared-color" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["rs-ring-color"] = fact["args"]["col"]

                # team and game information
                if "/refbox/points" in id:
                    if "cyan" in id:
                        gamereport["total-points"][0] = bson.Int64(
                            fact["value"])
                        gamereport["gamestate/PRODUCTION"]["points"][0] = int(
                            fact["value"])
                        gamereport["phase-points-cyan"]["PRODUCTION"] = int(
                            fact["value"])
                    else:
                        gamereport["total-points"][1] = bson.Int64(
                            fact["value"])
                        gamereport["gamestate/PRODUCTION"]["points"][1] = int(
                            fact["value"])
                        gamereport["phase-points-magenta"]["PRODUCTION"] = int(
                            fact["value"])

                # gametime production phase (last gamtime update we got?)
                if "/refbox/game-time" in id:
                    gamereport["gamestate/PRODUCTION"]["game-time"] = float(
                        fact["values"][0])+float(fact["values"][1])/1000000

            # time insensitive information
            results = coll.distinct("fact.id")
            for result in results:
                facts = (coll.find({"game": game, "fact.id": result}))

                fact = facts[0]["fact"]
                parse_fact_args(fact)
                id = fact["id"]

                # order information
                if "/refbox/order/" in id:
                    #delivery-begin, delivery-end, quantity-requested
                    order_fact = parse_refbox_order_fact(fact)
                    if order_fact["field"] == "delivery-begin":
                        gamereport["orders"][order_fact["id"] -
                                             1]["delivery-period"][0] = bson.Int64(order_fact["value"])
                    elif order_fact["field"] == "delivery-end":
                        gamereport["orders"][order_fact["id"] -
                                             1]["delivery-period"][1] = bson.Int64(order_fact["value"])
                    else:
                        gamereport["orders"][order_fact["id"] -
                                             1][order_fact["field"]] = bson.Int64(order_fact["value"])
                if "/domain/fact/order-complexity" in id:
                    if "comp=" in id:  # for some reason there is two sets, one with comp the other with com
                        gamereport["orders"][int(
                            fact["args"]["ord"][1:])-1]["complexity"] = fact["args"]["comp"]
                if "/domain/fact/order-gate" in id:
                    gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["delivery-gate"] = bson.Int64(fact["args"]["gate"].split("-")[1])
                if "/order/meta/competitive" in id:
                    gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["competitive"] = fact["value"]
                # colors
                if "/domain/fact/order-base-color" in id:
                    gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["base-color"] = fact["args"]["col"]
                if "/domain/fact/order-cap-color" in id:
                    gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["cap-color"] = fact["args"]["col"]
                if "/domain/fact/order-ring1-color" in id and fact["args"]["col"] != "RING_NONE":
                    ring_arr = gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["ring-colors"]
                    len_ring_arr = len(ring_arr)
                    if len_ring_arr < 1:
                        add_empty_strings_to_array(ring_arr, 1)
                    ring_arr[0] = fact["args"]["col"]
                if "/domain/fact/order-ring2-color" in id and fact["args"]["col"] != "RING_NONE":
                    ring_arr = gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["ring-colors"]
                    len_ring_arr = len(ring_arr)
                    if len_ring_arr < 2:
                        add_empty_strings_to_array(ring_arr, 2-len_ring_arr)
                    ring_arr[1] = fact["args"]["col"]
                if "/domain/fact/order-ring3-color" in id and fact["args"]["col"] != "RING_NONE":
                    ring_arr = gamereport["orders"][int(
                        fact["args"]["ord"][1:])-1]["ring-colors"]
                    len_ring_arr = len(ring_arr)
                    if len_ring_arr < 3:
                        add_empty_strings_to_array(ring_arr, 3-len_ring_arr)
                    ring_arr[2] = fact["args"]["col"]

                # machine information (only player color, maybe mirror)
                if "field-ground-truth/orientation" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["rotation"] = bson.Int64(fact["value"])
                if "field-ground-truth/mtype" in id:
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["mtype"] = fact["value"]
                if "field-ground-truth/zone" in id:
                    # TODO Mirror
                    gamereport["machines"][get_machine_arr_index(
                        gamereport["machines"], fact["args"]["m"])]["zone"] = fact["value"].replace("-", "_")
                if "/domain/fact/rs-ring-spec" in id:
                    if fact["args"]["rn"] != "ZERO":
                        if fact["args"]["r"] not in gamereport["machines"][get_machine_arr_index(gamereport["machines"], fact["args"]["m"])]["rs-ring-colors"]:
                            gamereport["machines"][get_machine_arr_index(
                                gamereport["machines"], fact["args"]["m"])]["rs-ring-colors"].append(fact["args"]["r"])

                # ringspec
                if "/domain/fact/rs-ring-spec" in id:
                    if fact["args"]["rn"] == "ZERO":
                        gamereport["ring-specs"].append(
                            {"color": fact["args"]["r"], "req-bases": bson.Int64(0)})
                    if fact["args"]["rn"] == "ONE":
                        gamereport["ring-specs"].append(
                            {"color": fact["args"]["r"], "req-bases": bson.Int64(1)})
                    if fact["args"]["rn"] == "TWO":
                        gamereport["ring-specs"].append(
                            {"color": fact["args"]["r"], "req-bases": bson.Int64(2)})

            # calculate the start-range and duration-range
            for order in gamereport["orders"]:
                start = order["delivery-period"][0]
                duration = order["delivery-period"][1] - start
                order["start-range"] = [bson.Int64(start), bson.Int64(start)]
                order["duration-range"] = [
                    bson.Int64(duration), bson.Int64(duration)]

            # get start time and generate timestamp version of it
            gamereport["start-time"] = coll.find({"game": game, "fact.value": "RUNNING"}).sort(
                "asserted", pymongo.ASCENDING)[0]["asserted"]
            timestamp = gamereport["start-time"].replace(
                tzinfo=timezone.utc).timestamp()
            mills = int((timestamp - int(timestamp))*1000000)
            gamereport["start-timestamp"] = [
                bson.Int64(int(timestamp)), bson.Int64(mills)]

            # get times for phases
            gamereport["gamestate/SETUP"]["start-time"] = [
                int(timestamp), mills]
            gamereport["gamestate/PRODUCTION"]["start-time"] = [
                int(timestamp), mills]
            production_time = coll.find({"game": game, "gamephase": "PRODUCTION"})[
                0]["set"].replace(tzinfo=timezone.utc).timestamp()
            setup_time = coll.find({"game": game, "gamephase": "SETUP"})[
                0]["set"].replace(tzinfo=timezone.utc).timestamp()
            gamereport["gamestate/SETUP"]["game-time"] = production_time - setup_time
            gamereport["gamestate/SETUP"]["last-time"] = gamereport["gamestate/SETUP"]["start-time"]
            gamereport["gamestate/SETUP"]["last-time"][0] = int(
                gamereport["gamestate/SETUP"]["last-time"][0] + gamereport["gamestate/SETUP"]["game-time"])
            gamereport["gamestate/SETUP"]["cont-time"] = gamereport["gamestate/SETUP"]["game-time"]
            timestamp = coll.find({"game": game, "gamephase": "PRODUCTION"})[
                0]["set"].replace(tzinfo=timezone.utc).timestamp()
            mills = int((timestamp - int(timestamp))*1000000)
            gamereport["gamestate/SETUP"]["end-time"] = [int(timestamp), mills]

            gamereport["gamestate/PRODUCTION"]["last-time"] = gamereport["gamestate/SETUP"]["start-time"]
            gamereport["gamestate/PRODUCTION"]["last-time"][0] = int(gamereport["gamestate/PRODUCTION"]["last-time"]
                                                                     [0] + gamereport["gamestate/SETUP"]["game-time"] + gamereport["gamestate/PRODUCTION"]["game-time"])
            gamereport["gamestate/PRODUCTION"]["cont-time"] = gamereport["gamestate/PRODUCTION"]["game-time"]

            # calculate the activation times from the time we got the information on them
            results = coll.find(
                {"game": game, "fact.id": {"$regex": "/domain/fact/order-base-color"}})
            for result in results:
                parse_fact_args(result["fact"])
                order = gamereport["orders"][int(
                    result["fact"]["args"]["ord"][1:])-1]
                start = order["delivery-period"][0]
                order["activate-at"] = bson.Int64(
                    int((result["asserted"] - gamereport["start-time"]).total_seconds()))
                order["activation-range"] = [bson.Int64(max(
                    0, start-order["activate-at"])), bson.Int64(max(0, start-order["activate-at"]))]

            # caclulate the idle time for idling machines:
            for machine in gamereport["machines"]:
                if "idle-since" in machine:
                    if machine["idle-since"] == -1:
                        machine["idle-since"] = int(
                            (time - gamereport["start-time"]).total_seconds())

            # storage station placeholders
            gamereport["machine-ss-shelf-slots"] = []
            ss_slot = {
                "move-to": [],
                "is-accessible": "TRUE",
                "is-filled": "FALSE",
                "num-payments": bson.Int64(0),
                "description": "",
                "last-payed": 0
            }
            for i in range(0, 6):
                for j in range(0, 8):
                    ss_slot_ins = deepcopy(ss_slot)
                    ss_slot_ins["position"] = [bson.Int64(i), bson.Int64(j)]
                    ss_slot_ins["name"] = "M-SS"
                    gamereport["machine-ss-shelf-slots"].append(ss_slot_ins)
                    ss_slot_ins = deepcopy(ss_slot)
                    ss_slot_ins["position"] = [bson.Int64(i), bson.Int64(j)]
                    ss_slot_ins["name"] = "C-SS"
                    gamereport["machine-ss-shelf-slots"].append(ss_slot_ins)
            # report name
            gamereport["report-name"] = "generated"

            # write gamereport to main mongodb instance
            try:
                default_client = pymongo.MongoClient(
                    "mongodb://localhost:27017/", connectTimeoutMS=20)
            except pymongo.errors.ServerSelectionTimeoutError:
                print("Couldn't connect to the default mongodb instance.",
                      file=sys.stderr)
                exit(0)

            gr_coll = default_client["rcll"]["game_report"]
            gr_coll.insert_one(gamereport)
            print("Wrote gamereport to database")

            # create config file
            print("Generating config file: gamestate-loader.yaml)\n(put this into your 'conf.d' directory of your fawkes-robotino install)")
            with open('gamestate-loader-template.yaml', 'r') as file:
                filedata = file.read()

            # Replace the target string
            filedata = filedata.replace('gx', game.isoformat()[
                                        :-3].replace("T", " ")+"Z")
            filedata = filedata.replace('ty', time.isoformat()[
                                        :-3].replace("T", " ")+"Z")

            # Write the file out again
            with open('gamestate-loader.yaml', 'w') as file:
                file.write(filedata)

# kill the mongod instance
os.killpg(os.getpgid(mongod.pid), signal.SIGTERM)
