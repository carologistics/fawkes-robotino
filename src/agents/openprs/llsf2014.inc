
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; declare and load agent-specific elements
declare op_predicate  say
declare op_predicate  ppgoto
declare op_predicate  conf-load-rcll2017
declare op_predicate  bring-product-to
declare op_predicate  get-product-from
declare op_predicate  instruct-mps
declare op_predicate  send-instruction

declare be  instruction-sent-ack

declare cwp           peer-id

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

;Agent States
declare symbol IDLE
declare symbol WORKING
declare symbol SKILL-EXECUTION

;Colors for the caps
declare symbol BLACK
declare symbol GRAY
declare symbol BLUE
declare symbol GREEN
declare symbol ORANG
declare symbol YELLOW
declare symbol RED
declare symbol SILVER




add (game-state null)
add (game-phase null)

load opf "rcll2017_config.opf"
load db  "llsf2014.db"
load opf "llsf2014.opf"
load opf "production.opf"
load opf "net.opf"
load opf "mps-instructions.opf"
load opf "steps.opf"
load opf "skills.opf"