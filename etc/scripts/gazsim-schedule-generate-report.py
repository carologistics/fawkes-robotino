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

# result string
res = ""

def printAppend(str):
    global res
    res = res+str
    

print("Generating result file %s" % OUTPUTFILE)
print("Using collection %s" % COLLECTION)

numGames = col.count()

printAppend("#################\n#################\nTeam Reports:\n#################\n#################\n")
# collect team data from games:
teamVictory = {}
teamDraws = {}
teamLosses = {}
teamAvgPoints = {}
teamAvgPointsExp = {}
teamAvgPointsProd = {}
for game in col.find():
    cyan = game["configuration"].split("-vs-")[0]
    magenta = game["configuration"].split("-vs-")[1].split("_run")[0]
    if cyan in teamAvgPoints:
        teamAvgPoints[cyan] += game["total-points-cyan"]/numGames
        teamAvgPointsExp[cyan] += game["exp-points-cyan"]/numGames
        teamAvgPointsProd[cyan] += game["prod-points-cyan"]/numGames
    else:
        teamAvgPoints[cyan] = game["total-points-cyan"]/numGames
        teamAvgPointsExp[cyan] = game["exp-points-cyan"]/numGames
        teamAvgPointsProd[cyan] = game["prod-points-cyan"]/numGames
        teamVictory[cyan] = 0
        teamDraws[cyan] = 0
        teamLosses[cyan] = 0

    if magenta in teamAvgPoints:
        teamAvgPoints[magenta] += game["total-points-magenta"]/numGames
        teamAvgPointsExp[magenta] += game["exp-points-magenta"]/numGames
        teamAvgPointsProd[magenta] += game["prod-points-magenta"]/numGames        
    else:
        teamAvgPoints[magenta] = game["total-points-magenta"]/numGames
        teamAvgPointsExp[magenta] = game["exp-points-magenta"]/numGames
        teamAvgPointsProd[magenta] = game["prod-points-magenta"]/numGames
        teamVictory[magenta] = 0
        teamDraws[magenta] = 0
        teamLosses[magenta] = 0
    
    if game["total-points-cyan"] > game["total-points-magenta"]:
        teamVictory[cyan] += 1
        teamLosses[magenta] += 1
    else:
        if game["total-points-cyan"] < game["total-points-magenta"]:
            teamVictory[magenta] += 1
            teamLosses[cyan] += 1
        else:
            teamDraws[cyan] += 1
            teamDraws[magenta] += 1

for team in teamVictory:
	printAppend("\n#################\nTeam: %s\n#################\n" % team)
        printAppend("Victories: %d\n" % teamVictory[team])
        printAppend("Draws: %d\n" % teamDraws[team])
        printAppend("Losses: %d\n" % teamLosses[team])
        printAppend("Average Points: %d\n" % teamAvgPoints[team])
        printAppend("Average Points Exploration: %d\n" % teamAvgPointsExp[team])
        printAppend("Average Points Production: %d\n" % teamAvgPointsProd[team])


printAppend("\n\n#################\n#################\nGame Reports:\n#################\n#################\n")
printAppend("Number of recoreded games: %d\n" % numGames)
# #f = open(OUTPUTFILE, 'w')
for game in col.find():
    printAppend("#################\nGame: %s\n#################\n" % game["configuration"])
    printAppend("Refbox Summery:\n %s\n" % game["refbox_game_summery"])
    #f.write("%f %f\n" % (pose["translation"][0], pose["translation"][1]))

print(res)
