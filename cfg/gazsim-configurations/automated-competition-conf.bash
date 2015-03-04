# Configuration file to setup an automated competition in the simulation
# between different branches or configurations
# File is written in bash and used by etc/gazsim-schedule.bash

# Name of competition (used as folder-name for logs, date is appeded)
COMPETITION_NAME=testruns

# Number of games for each team pair
NUM_RUNS=1 

# Run Gazebo headless (without gui)
HEADLESS=false

# Log Path (also the code is checked out there)
COMPETITION_LOG_PATH=~/gazsim-automated-competition

# Record replays?
REPLAY=true

# Teams competing in this simulated competition
# Add teams by calling the function addTeam and replace parameters according to your team
# @param team-name team name without spaces or strange symbols
# @param fawkes-robotino-branch Your branch in fawkes-robotino
# @param fawkes-branch Your branch in fawkes
# @param configuration path to your configuration files: cfg(gazsim-configurations/$CONFIGURATION/robotinoX.yaml
# @param number-robots With how many robots do you want to play (between 1 and 3)
# @param additional-plugins Additional plugins to load (e.g. which agent plugins)
# e.g.:
# addTeam "TeamName"  origin/your-fawkes-robotino-branch origin/your-fawkes-branch configuration number-robots clips,clips-agent,...

addTeam "Lab-1"  origin/LabPRogramR_1/production origin/common/praktikum-openprs default 1 "openprs,openprs-agent"
addTeam "Lab-2"  origin/labprogramr_2/production origin/common/praktikum-openprs default 1 "openprs,openprs-agent"
addTeam "ClipsAgent"  origin/common/praktikum-openprs origin/common/praktikum-openprs default 1 "clips,clips-agent,clips-protobuf,clips-motor-switch,clips-webview,clips-navgraph,agent-monitor"
