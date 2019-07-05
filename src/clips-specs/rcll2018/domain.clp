;---------------------------------------------------------------------------
;  domain.clp - Domain configuration
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;             2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defrule domain-load
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "rcll2018/domain.pddl"))
  (assert (domain-loaded))
)

(deffunction domain-load-local-facts (?self)
	"Initialize facts that are not synced."

	(assert
	  (domain-fact (name self) (param-values ?self))
	  (domain-object (name R-1) (type robot))
    (domain-object (name R-2) (type robot))
    (domain-object (name R-3) (type robot))
    (domain-fact (name at) (param-values ?self START INPUT))
    (domain-fact (name robot-waiting) (param-values ?self))
	)
)

(defrule domain-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  (wm-fact (key config rcll robot-name) (value ?robot-name))
  =>
  (domain-load-local-facts ?robot-name)
  (printout info "Initializing worldmodel" crlf)
  (assert (domain-facts-loaded))
)

