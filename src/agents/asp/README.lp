% This file doesn't contain any program code, but just documentation on how to use the test script and what are the
% files for.
%
% TL;DR: Just run clingo <instance-file>.
%
% Overview over the files:
% exploration.lp     Defines the task for exploring, but also the locations to do so, basicly it contains everything
%                    which is special for the exploration phase.
% facts.lp           Some simple facts, mostly in program base, but some for program ourTeam, see below.
% functions.lp       Contains external functions implemented in Python for offline testing.
% goal.lp            Defines our goal and possible heuristics to help clingo finding a good plan.
% instance-*.lp      Instances for testing. The naming of the files follows a schema.
%                    On the one hand there are the instance-exploration*.lp in which only the exploration phase will be
%                    tested.
%                    On the other hand the format is instance-rR-oO-cX.Y*.lp, the instance uses R robots to handle O
%                    orders. The most complicated product uses X rings and the most complicated color needs Y additional
%                    bases to be hand out.
%                    If a file ends with the suffix -check.lp there is an answer encoded in the instance and invalid
%                    encoding changes should result in infeasibility.
%                    See also sample-instance.lp
% machines.lp        Handles the state of the machines.
% orders.lp          Contains the programs for order placement.
% products.lp        Handles the state of the products.
% README.lp          You are here.
% robot.lp           Handles the state of the robots.
% sample-instance.lp Explains the format of the instance files.
% script.lp          Handles the control flow for offline testing, baiscly replaces fawkes.
% state.lp           Handles the task selection and the common implications. A.k.a. the Brain, no Pinky here, Narf!
% taskDeliver.lp     Contains the tasks to deliver a finished product.
% taskFeedRS.lp      Contains the task to feed a product in any state in a ring station for the advanced colors.
% taskGetBase.lp     Contains the task to get a new base from the base machine.
% taskGetProduct.lp  Contains the task to fetch a product from a ring or cap station after it is processed.
% taskGoto.lp        Contains the task to goto to an location in preparation for a following task.
% taskMountCap.lp    Contains the task to mount a cap on a product.
% taskMountRing.lp   Contains the tasks to mount a ring on a product.
% taskPrepareCS.lp   Contains the task to prepare a cap station.
%
% Explanation of the script:
%
% The script uses psutil, if available to print the memory consumtion. Install through pip install psutil or see
% https://github.com/giampaolo/psutil
%
% The script is tested with Python 3.4.3, but should also run on Python 2.
%
% Explanation of the encoding:
%
% Overview of common programs:
% ourTeam(t)         We know our team color, by that we can compute some facts. We could compute these facts regardless
%                    of the team color, but than the modelling would be a bit more difficult and we had unneeded facts,
%                    simply consuming memory and computation power.
%
% Common Variables:
% GT                 Gametime, after application of the time resolution. The program calculates in these time steps.
% L                  Locations a robot can drive to, mostly machine sides to perform tasks.
% O                  The order number.
% P                  A product, a pair of the robot name who brought the product to life and an (robot) unique index.
% Q                  The quantity number of an order.
% R                  Robot-name, sometimes ring(-color)
% T                  Unique task name, the executive gets all information it needs from the parameters of T.
