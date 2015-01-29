#!/usr/bin/python

# Generates result file from automated simulation run data

import pymongo
import sys
from pymongo import Connection


# read arguments
OUTPUTFILE = sys.argv[1]
COLLECTION = sys.argv[2]

# connect to db and get collection
conn = Connection()
db = conn['gazsim-runs']
col = db[COLLECTION]

print("Generating result file %s" % OUTPUTFILE)
print("Using collection %s" % COLLECTION)

print("#################\n#################\nGame Reports\n#################\n#################\n")
print("Number of recoreded games: %d" % col.count())
# #f = open(OUTPUTFILE, 'w')
for game in col.find():
    print("#################\nGame: %s\n#################\n" % game["configuration"])
    print("Refbox Summery:\n %s\n" % game["refbox_game_summery"])
    #f.write("%f %f\n" % (pose["translation"][0], pose["translation"][1]))
