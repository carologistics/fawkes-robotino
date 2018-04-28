;---------------------------------------------------------------------------
;  resource-locking.clp - Functions and templates for resource locking
;
;  Created: Sat 28 Apr 2018 09:08:43 CEST
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(defglobal
  ?*GOAL-COMMITMENT-AUTO-RENEW* = FALSE
)

(deffunction goal-to-lock (?goal $?params)
  "Get a multi-field of locks that is needed to commit to a specific goal."
  (switch ?goal
    (case FILL-CAP then
      (return (create$ (sym-cat resource- (nth$ 4 ?params)))))
    (case CLEAR-CS then
      (return (create$ (sym-cat resource- (nth$ 4 ?params))
                       (sym-cat resource- (nth$ 6 ?params)))))
    (case DISCARD-UNKNOWN then
      (return (create$ (sym-cat resource- (nth$ 4 ?params)))))
    (case PRODUCE-C0 then
      (return (create$ (sym-cat resource- (nth$ 10 ?params))
                       (sym-cat resource- (nth$ 14 ?params)))))
    (case DELIVER then
      (return (create$ (sym-cat resource- (nth$ 4 ?params))
                       (sym-cat resource- (nth$ 6 ?params))
                       (sym-cat resource- (nth$ 8 ?params)))))
    (default (return (create$)))
    ;(default (printout error "Cannot get lock names of " ?goal ","
    ;                         " unknown goal" crlf)
  )
)
