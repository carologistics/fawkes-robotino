
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
declare cwp           peer-id
declare ff   		team-color 0
declare ff 	 		game-time 0
declare ff 			game-state 0
declare ff			game-phase 0
declare ff 			machine 1
declare ff 			ring-station 1
declare ff			order 1
declare symbol public
declare symbol private
declare symbol CYAN
declare symbol MAGENTA

add (game-state null)
add (game-phase null)

load db  "llsf2014.db"
load opf "llsf2014.opf"
load opf "net.opf"
