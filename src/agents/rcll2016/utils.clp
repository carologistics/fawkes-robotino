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

(deffunction get-light-signal-side (?mps)
  "Return the navgraph point of the side where the light-signal is mounted"
  (if (any-factp ((?machine machine)) (and (eq ?machine:name ?mps)
					   (eq ?machine:mtype DS)))
    then
    ; for DS
    (return (get-input ?mps))
    else
    ; for all other mps
    (return (get-output ?mps))
  )
)

(deffunction create-multifield-with-length-and-entry (?length ?entry)
  "Creates a Multifield with ?length times the entry ?entry"
  (bind $?res (create$ ))
  (loop-for-count (?i 1 24) do
    (bind $?res (insert$ ?res 1 ?entry))
  )
  (return ?res)
)

(deffunction create-multifield-with-length (?length)
  "Creates a Multifield with length ?length"
  (return (create-multifield-with-length-and-entry ?length RANDOM-SYMBOL))
)

(deffunction navgraph-add-all-new-tags ()
  "send all new tags to the navgraph generator"
  (bind ?any-tag-to-add FALSE)
  (delayed-do-for-all-facts ((?ft found-tag)) (not (any-factp ((?added-f navgraph-added-for-mps)) (eq ?ft:name ?added-f:name)))
    (bind ?any-tag-to-add TRUE)
    ; report tag position to navgraph generator
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
    (blackboard-set-msg-field ?msg "name" (str-cat ?ft:name))
    (blackboard-set-msg-field ?msg "side" ?ft:side)
    (blackboard-set-msg-field ?msg "frame" ?ft:frame)
    (blackboard-set-msg-multifield ?msg "tag_translation" ?ft:trans)
    (blackboard-set-msg-multifield ?msg "tag_rotation" ?ft:rot)
    (blackboard-send-msg ?msg)
    (printout t "Send UpdateStationByTagMessage: id " (str-cat ?ft:name) 
	      " side " ?ft:side
	      " frame " ?ft:frame
	      " trans " ?ft:trans
	      " rot " ?ft:rot  crlf)
   
    (assert (navgraph-added-for-mps (name ?ft:name)))
  )
  (if ?any-tag-to-add
    then
    ; send compute message so we can drive to the output
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
    (bind ?compute-msg-id (blackboard-send-msg ?msg))

    ; save the last compute-msg-id to know when it was processed
    (do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
      (retract ?lncm)
    )
    (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
    else
    (printout t "There are no tags to add" crlf)
  )
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

(deffunction utils-get-2d-center (?x1 ?y1 ?x2 ?y2)
  (return (create$ (/ (+ ?x1 ?x2) 2) (/ (+ ?y1 ?y2) 2)))
)

(deffunction round-down (?x)
  (bind ?round (round ?x))
  (if (< ?x ?round) then
    (return (- ?round 1))
  )
  (return ?round)
)

(deffunction string-from-multifield ($?mf)
  (bind ?res "")
  (progn$ (?f ?mf)
    (bind ?res (str-cat ?res ?f ","))
  )
)

(deffunction utils-get-zone-edges (?zone)
  "returns the edges of a zone"
  ; compute cyan edges first and mirrow them later if neccessary
  (bind ?zone (eval (sub-string 2 (str-length (str-cat ?zone)) (str-cat ?zone))))
  (bind ?zone-cyan ?zone)
  (if (> ?zone-cyan 12) then
    (bind ?zone-cyan (- ?zone 12))
  )
  (bind ?zone-x (round-down (/ (- ?zone-cyan 1) 4)))
  (bind ?zone-y (mod (- ?zone-cyan 1) 4))
  
  (bind ?y-min (* ?zone-y ?*ZONE-HEIGHT*))
  (bind ?y-max (* (+ ?zone-y 1) ?*ZONE-HEIGHT*))

  (bind ?x-min (* ?zone-x ?*ZONE-WIDTH*))
  (bind ?x-max (* (+ ?zone-x 1) ?*ZONE-WIDTH*))
  
  (if (> ?zone 12) then
    (bind ?x-min (- 0 ?x-min))
    (bind ?x-max (- 0 ?x-max))
  )
  (return (create$ ?x-min ?x-max ?y-min ?y-max))
)

(deffunction utils-get-tags-str-still-to-explore (?team)
  ; get string with all tags we still need to explore
  (bind ?search-tags "{")
  (delayed-do-for-all-facts ((?tag tag-matching)) (and (eq ?tag:team ?team)
                                                       (not (any-factp ((?found found-tag)) (eq ?found:name ?tag:machine))))
    (bind ?search-tags (str-cat ?search-tags ?tag:tag-id ","))
  )
  (bind ?search-tags (str-cat ?search-tags "}"))
  (return ?search-tags)
)

(deffunction escape-if-string (?f ?slot $?new-value)
  ; this function ads \" \" around a string to prevent transforming the string value
  ; into a Symbol when using the eval function
	(if (> (length$ ?new-value) 0)
	 then
		(bind ?value (nth$ 1 ?new-value))
	 else
		(bind ?value (fact-slot-value ?f ?slot))
	)
	(bind ?value-type (type ?value))
	(bind ?slot-type  (nth$ 1 (deftemplate-slot-types (fact-relation ?f) ?slot)))
	;(printout t "Value type: " ?value-type "  -- Slot " ?slot " has types " ?slot-types crlf)
  (if (eq STRING ?value-type ?slot-type) then
    (return (str-cat "\"" ?value "\""))
  )
  (return ?value)
)

(deffunction dyn-mod (?f $?arg) ;arg contains slot and value pairs
  ; dynamic modify function that takes the fact-adress, the slot name and the new value
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (neq (mod (length$ ?arg) 2) 0) then
    (printout error "dyn-mod needs to be called with slot-value pairs" crlf)
    (return)
  )
  (bind ?slots-to-change (create$))
  (bind ?values-to-set (create$))
  (progn$ (?field ?arg)
    (if (eq (mod ?field-index 2) 1)
      then
      (bind ?slots-to-change (insert$ ?slots-to-change 1 ?field))
      else
      (bind ?values-to-set (insert$ ?values-to-set 1 ?field))
    )
  )
  ; dont modify if set to current value
  (bind ?different-slots (create$))
  (bind ?different-values (create$))
  (progn$ (?slot ?slots-to-change)
    (if (neq (fact-slot-value ?f ?slot) (nth$ ?slot-index ?values-to-set)) then
      (bind ?different-slots (insert$ ?different-slots 1 ?slot))
      (bind ?different-values (insert$ ?different-values 1 (nth$ ?slot-index ?values-to-set)))
    )
  )
  (bind ?slots-to-change ?different-slots)
  (bind ?values-to-set ?different-values)
  ; return if there is nothing to modify
  (if (eq 0 (length$ ?slots-to-change)) then
    (return ?f)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?slot (fact-slot-names ?f))
    (if (not (member$ ?slot ?slots-to-change))
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?slot)
        then ;copy multifield
        (bind ?acom (str-cat ?acom "(" ?slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?slot))
                             "))"))
        else ;copy singlefield
				(bind ?acom (str-cat ?acom "(" ?slot " " 
														 (escape-if-string ?f ?slot) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           (escape-if-string ?f ?slot (nth$ (member$ ?slot ?slots-to-change) ?values-to-set))
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-add-to-multifield (?f ?slot ?value)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the new value that should be added to the multifield in the slot
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string ?f ?cur-slot) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ "
                           (implode$ (fact-slot-value ?f ?cur-slot))
                           " " (escape-if-string ?f ?cur-slot ?value) ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-remove-from-multifield (?f ?slot ?value)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the value that should be removed from the multifield in the slot
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ "
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string ?f ?cur-slot) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ "
                           (implode$ (delete-member$ (fact-slot-value ?f ?cur-slot) ?value))
                           ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-override-multifield (?f ?slot $?multifield)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the new list
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  ; skip overriding if there is no change
  (if (eq (string-from-multifield ?multifield)
          (string-from-multifield (fact-slot-value ?f ?slot))) then
    (return ?f)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string ?f ?cur-slot) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ " (implode$ ?multifield) ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-assert (?fact)
  ; assert a fact from a string
  (bind ?assertstr (str-cat "(assert " ?fact ")"))
  (bind ?res (eval ?assertstr))
  (if (not ?res) then
    (printout error "Assert failed : " ?fact crlf)
  )
  (return ?res)
)