;---------------------------------------------------------------------------
;  util.clp - utilities needed for executive rcll agent
;
;  Created: Tue 19 Apr 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(deffunction get-param-by-arg (?params ?arg)
	"Extract the argument named in ?arg.
   @param ?params the paramter list
   @param ?arg name of argument to extract

   @return returns nil if value is not specified, or a single or
           multi-field value depending on the argument value."
	(bind ?l (member$ ?arg ?params))
	(bind ?L (length$ ?params))
	(if ?l then
		(while (<= (+ ?l 1) ?L) do
			(if (eq (nth$ ?l ?params) ?arg) then
				(return (nth$ (+ ?l 1) ?params))
			)
		)
	)
	(return nil)
)
