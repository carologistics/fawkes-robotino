% This file doesn't contain any program code, but just documentation.
%
% Overview over the files:
% delivery.lp     Contains the tasks to deliver a finished product.
% exploration.lp  Only grounded if there is at least one machine to find, contains tasks for the exploration phase and
%                 declares driveDuration as external.
% facts.lp        Some simple facts, mostly in program base, but some for program ourTeam, see below.
% feedRS.lp       Contains the task to feed a product in any state in a ring station for the advanced colors.
% getBase.lp      Contains the task to get a new base from the base machine.
% getProduct.lp   Contains the task to fetch a product from a ring or cap station after it is processed.
% goal.lp         Defines our goal and possible heuristics to help clingo finding a good plan.
% machines.lp     Handles the state of the machines.
% mountCap.lp     Contains the task to mount a cap on a product.
% mountRing.lp    Contains the tasks to mount a ring on a product.
% orders.lp       Contains the programs for order placement.
% prepareCS.lp    Contains the task to prepare a cap station.
% products.lp     Handles the state of the products.
% robot.lp        Handles the state of the robots.
% script.lp       Contains external functions implemented in Python for offline testing.
% startHere.lp    You are here.
% state.lp        Handles the task selection and the common implications. A.k.a. the Brain, no Pinky here, Narf!
%
% Overview of common programs:
% transition(gt)  The game advances another time step.
% ourTeam(t)      We know our team color, by that we can compute some facts. We could compute these facts regardless of
%                 the team color, but than the modelling would be a bit more difficult and we had unneeded facts, simply
%                 consuming memory and computation power.
%
% Common Variables:
% GT              Gametime, after application of the time resolution. The program calculates in these time steps.
% L               Locations a robot can drive to, mostly machine sides to perform tasks.
% O               The order number.
% P               A product, a pair of the robot name who brought the product to life and an (robot) unique index.
% Q               The quantity number of an order.
% R               Robot-name, sometimes ring(-color)
% T               Unique task name, the executive gets all information it needs from the parameters of T.
