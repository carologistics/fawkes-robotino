
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; declare and load agent-specific elements
declare op_predicate  say
declare cwp           peer-id
declare predicate     team-color

declare symbol public
declare symbol private
declare symbol CYAN
declare symbol MAGENTA

load db  "llsf2014.db"
load opf "llsf2014.opf"
load opf "net.opf"
