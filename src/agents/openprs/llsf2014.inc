
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;Declare and load agent-specific elements
declare op_predicate  say
declare op_predicate  ppgoto
declare op_predicate  conf-load-rcll2017

declare op_predicate  instruct-mps
declare op_predicate  send-instruction
declare be  instruction-sent-ack
declare ff last-attempted-instruction 1

declare cwp         peer-id

declare ff   		team-color 0
declare ff 	 		game-time 0
declare ff 			game-state 0
declare ff			game-phase 0
declare ff 			machine 1
declare ff 			ring-station 1
declare ff 			cap-station 1
declare ff			order 1
declare ff			ring 1 					;holds how much material for this ring 
declare ff			agent-state 0

declare symbol public
declare symbol private
declare symbol CYAN
declare symbol MAGENTA

;/////////////
;Agent States
;/////////////
declare symbol IDLE
declare symbol WORKING
declare symbol SKILL-EXECUTION
declare symbol SKILL-FINISHED
declare symbol SKILL-FAILED
declare symbol STEP-RUNNING
declare symbol STEP-PENDING
declare symbol STEP-FINISHED
declare symbol STEP-FAILED

declare ff holding 0


declare ff workpiece 1

;Material Types
declare symbol CAP
declare symbol BASE
declare symbol RING
declare symbol NONE

;Colors for the Material
declare symbol BLACK
declare symbol GRAY
declare symbol BLUE
declare symbol GREEN
declare symbol ORANG
declare symbol YELLOW
declare symbol RED
declare symbol SILVER

;Cap Station
declare symbol CAP_LOADED
declare symbol CAP_NOT_LOADED

declare predicate template
declare cwp in-production

;//////////////////
;TEMPORAL RELATED
;/////////////////
;Records the time stamp the agent first 
;realized that machine's state was READY_AT_OUTPUT
declare ff ready-at-output-since 1 


add (game-state null)
add (game-phase null)
add (holding NONE)

add (template (workpiece id (Base color) (rings color color color) (cap color) (place machine side)))

include "skills.inc"
include "steps.inc"
load opf "net.opf"
load opf "rcll2017_config.opf"
load opf "llsf2014.opf"
load db  "llsf2014.db"

load opf "production.opf"
load opf "mps-instructions.opf"
load opf "steps.opf"
load opf "skills.opf"
load opf "debug.opf"
load opf "worldmodel.opf"
load opf "workpiece-operations.opf"