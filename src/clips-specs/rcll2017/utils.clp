;---------------------------------------------------------------------------
;  utils.clp - Robotino agent decision testing -- utility functions
;
;  Created: Sun Jun 17 12:19:34 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2013  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane."
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)

(deffunction is-working ($?out-of-order)
  "Check if a machine is not out of order"
  (return (eq (nth$ 1 ?out-of-order) 0))
)

(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(deffunction get-input (?mps)
  "Return the navgraph point of the input side of the given mps"
  (return (str-cat ?mps "-I"))
)

(deffunction get-output (?mps)
  "Return the navgraph point of the output side of the given mps"
  (return (str-cat ?mps "-O"))
)

(deffunction str-split (?string ?sep)
	(bind ?s ?string)
	(bind ?rv (create$))
	(while (> (str-length ?s) 0)
		(bind ?idx (str-index ?sep ?s))
		(if ?idx then
			(bind ?s2 (sub-string 1 (- ?idx 1) ?s))
			(bind ?rv (append$ ?rv ?s2))
			(bind ?s (sub-string (+ ?idx (str-length ?sep)) (str-length ?s) ?s))
		 else
		 	(bind ?rv (append$ ?rv ?s))
			(bind ?s "")
		)		
	)
	(return ?rv)
)

(deffunction str-join (?sep $?strings)
	(bind ?rv "")
	(bind ?locsep "")
	(foreach ?s ?strings
		(bind ?rv (str-cat ?rv ?locsep ?s))
		(bind ?locsep ?sep)			 
	)
	(return ?rv)
)


(deffunction utils-remove-prefix (?string ?prefix)
  "Removes a prefix from a string or symbol by its length"
  (bind ?res (sub-string (+ 1 (str-length (str-cat ?prefix))) 
			 (str-length (str-cat ?string))
			 (str-cat ?string)))
  (if (eq (type ?string) SYMBOL) then
    (return (sym-cat ?res))
    else
    (return ?res)
  )
)

