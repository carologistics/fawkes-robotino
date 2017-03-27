;---------------------------------------------------------------------------
;  worldmodel-synchronization.clp - Synchronize the worldmodel with the other robots
;
;  Created: Mon Mar 10 15:03:07 2014
;  Copyright  2014  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction wm-check-synchronizability (?fact ?slot ?value)
  ; check if an attempted synchronization is valid
  (if (not (fact-existp ?fact))
    then
    (printout error "Fact " ?fact " does not exist anymore. Sync failed. Slot: "
              ?slot " Value: " ?value crlf)
    (return FALSE)
  )
  (if (not (member$ ?slot (fact-slot-names ?fact)))
    then
    (printout error "Slot " ?slot " does not exist for fact " ?fact ". Sync failed." crlf)
    (return FALSE)
  )
  (if (not (member$ sync-id (fact-slot-names ?fact)))
    then
    (printout error "Fact " ?fact " does not have a sync-id. Sync failed. Slot: "
              ?slot " Value: " ?value crlf)
    (return FALSE)
  )
  (return TRUE)
)

(deffunction wm-sync-get-index-of-slot (?fact ?slot)
  ; returns the index of a slot for a deftemplate.
  ; The index is later used as key in the key-value messages
  ; A multifield has multiple indices depending on if we want to add or
  ; remove the value from the multifield
  ; (+0: whole multifield as string, +1 add value, +2 remove value)
  (bind ?template (fact-relation ?fact))
  (bind ?index 1)
  (progn$ (?cur-slot (deftemplate-slot-names ?template))
    (if (eq ?cur-slot ?slot)
      then
      (return ?index)
      else
      (if (deftemplate-slot-multip ?template ?cur-slot)
        then
        ; increase more than one slot because it is a multifield
        (bind ?index (+ 3 ?index))
        else
        (bind ?index (+ 1 ?index))
      )
    )
  )
  (printout error "Fact " ?fact " of template " ?template
            " has no slot " ?slot "." crlf)
  (return 0)
)


(deffunction wm-sync-get-slot-of-index (?fact ?index)
  ; returns the slot corresponding to a key and deftemplate
  ; if the slot is a multifield it is returned as a tuple with slotname and relative index
  (bind ?template (fact-relation ?fact))
  (bind ?cur-index 1)
  (progn$ (?cur-slot (deftemplate-slot-names ?template))
    (if (eq ?cur-index ?index)
      then
      (if (deftemplate-slot-multip ?template ?cur-slot)
        then
        (return (create$ ?cur-slot 0))
        else
        (return ?cur-slot)
      )
      else
      (if (deftemplate-slot-multip ?template ?cur-slot)
        then
        (if (< ?index (+ 3 ?cur-index))
          then
          (return (create$ ?cur-slot (- ?index ?cur-index)))
          else
          (bind ?cur-index (+ 3 ?cur-index))
        )
        else
        (bind ?cur-index (+ 1 ?cur-index))
      )
    )
  )
  (printout error "Fact " ?fact " of template " ?template
            " has no index " ?index "." crlf)
  (return FALSE)
)

(deffunction worldmodel-sync-get-sync-id ()
  ; return determimistic sync-ids for initial facts
  ; and random ones for non-initial facts
  (if ?*USING-INITIAL-SYNC-IDS*
    then
    (bind ?*LAST-INITIAL-SYNC-ID* (+ 1 ?*LAST-INITIAL-SYNC-ID*))
    (return ?*LAST-INITIAL-SYNC-ID*)
    else
    (return (random (+ 1 ?*LAST-INITIAL-SYNC-ID*) 999999))
  )
)

(deffunction synced-modify (?fact $?args)
  ; modify a fact and synchronize the change
  ; returns the adress of the modified fact, ?fact is no longer usable
  (bind ?slots-to-change (create$))
  (bind ?values-to-set (create$))
  (progn$ (?field ?args)
    (if (eq (mod ?field-index 2) 1)
      then
      (bind ?slots-to-change (insert$ ?slots-to-change 1 ?field))
      else
      (bind ?values-to-set (insert$ ?values-to-set 1 ?field))
    )
  )
  (progn$ (?slot ?slots-to-change)
    (bind ?value (nth$ ?slot-index ?values-to-set))
    (if (not (wm-check-synchronizability ?fact ?slot ?value))
      then
      (return)
    )
    ; create worldmodel-change-fact for synchronization
    (bind ?key (+ (* 100 (fact-slot-value ?fact sync-id))
		  (wm-sync-get-index-of-slot ?fact ?slot)))
    ; remove previous worldmodel change facts for the same key
    (delayed-do-for-all-facts ((?wmcf worldmodel-change)) (eq ?wmcf:key ?key)
      (retract ?wmcf)
    )
    (assert (worldmodel-change (key ?key) (value ?value)))
  )
  ; modify the fact locally
  (return (dyn-mod ?fact ?args))
)

(deffunction synced-add-to-multifield (?fact ?slot ?value)
  ; modify a fact and synchronize the change
  ; returns the adress of the modified fact, ?fact is no longer usable
  (if (not (wm-check-synchronizability ?fact ?slot ?value))
    then
    (return)
  )
  ; create worldmodel-change-fact for synchronization
  (assert (worldmodel-change (key (+ (* 100
                                        (fact-slot-value ?fact sync-id))
                                     (+ 1 (wm-sync-get-index-of-slot ?fact ?slot))))
                             (value ?value))
  )
  ; modify the fact locally
  (return (dyn-add-to-multifield ?fact ?slot ?value))
)

(deffunction synced-remove-from-multifield (?fact ?slot ?value)
  ; modify a fact and synchronize the change
  ; returns the adress of the modified fact, ?fact is no longer usable
  (if (not (wm-check-synchronizability ?fact ?slot ?value))
    then
    (return)
  )
  ; create worldmodel-change-fact for synchronization
  (assert (worldmodel-change (key (+ (* 100
                                        (fact-slot-value ?fact sync-id))
                                     (+ 2 (wm-sync-get-index-of-slot ?fact ?slot))))
                             (value ?value))
  )
  ; modify the fact locally
  (return (dyn-remove-from-multifield ?fact ?slot ?value))
)

(deffunction synced-assert (?fact)
  ; asserts a fact locally and for the synchronized bots
  ; the given fact needs to be a string normally used in (assert)
  ; e.g. ?fact = "(pose (x 3) (y 2))"
  (bind ?fact (str-cat ?fact))
  (if (not (str-index "(sync-id" ?fact))
    then
    ;add random sync-id
    (bind ?fact (str-cat (sub-string 1 (- (str-length ?fact) 1) ?fact)
                         "(sync-id " (worldmodel-sync-get-sync-id)
                         "))"))
  )
  ; create worldmodel-change-fact for synchronization
  (assert (worldmodel-change (key ?*SYNC-ID-ASSERT*)
                             (value ?fact))
  )
  ; assert the fact locally
  (dyn-assert ?fact)
)

(deffunction synced-retract (?fact)
  ; retract a fact locally and also for synchronized bots
  ; create worldmodel-change-fact for synchronization
  (assert (worldmodel-change (key ?*SYNC-ID-RETRACT*)
                             (value (fact-slot-value ?fact sync-id)))
  )
  ; retract the fact locally
  (retract ?fact)
)

(defrule worldmodel-sync-set-agent-in-change
  "Set the agent field in a new worldmodel change. We know that the change is from this agent because otherwise the field would be set."
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (agent DEFAULT))
  =>
  (modify ?wmc (agent (sym-cat ?*ROBOT-NAME*)))
)

(deffunction worldmodel-sync-apply-key-value-msg (?pair)
  ; This function applies the modification of the worldmodel
  ; sent with a single key-value message
  
  (bind ?synced-templates (create$))
  ;get list of all templates that are synced
  (do-for-fact ((?wm-sync-info wm-sync-info)) TRUE
    (bind ?synced-templates ?wm-sync-info:synced-templates)
  )
  (bind ?key (pb-field-value ?pair "key"))
  ; get value
  (bind ?value nil)
  (if (pb-has-field ?pair "value_string")
    then
    (bind ?value (pb-field-value ?pair "value_string"))
    else
    (if (pb-has-field ?pair "value_symbol")
      then
      (bind ?value (sym-cat (pb-field-value ?pair "value_symbol")))
      else
      (if (pb-has-field ?pair "value_uint32")
        then
        (bind ?value (pb-field-value ?pair "value_uint32"))
        else
        (if (pb-has-field ?pair "value_float")
          then
          (bind ?value (pb-field-value ?pair "value_float"))
          else
          (printout error "No value set for sync-pair!" crlf)
        )
      )
    )
  )
  ; is it a change, assert, or retract
  (if (eq ?key ?*SYNC-ID-ASSERT*)
    then
    (dyn-assert ?value)
    else
    (if (eq ?key ?*SYNC-ID-RETRACT*)
      then
      ; assert fact that is used by dnyamically generated rule to retract the fact
      (assert (synced-fact-to-retract ?value))
      else
      ; it is a change to an existing fact
      (bind ?sync-id (integer (/ ?key 100)))
      ; find corresponding fact
      (bind ?applied FALSE)
      (progn$ (?template ?synced-templates)
        (do-for-fact ((?fact ?template)) (eq ?sync-id ?fact:sync-id)
          (if ?applied then
            (printout error "sync-id " ?sync-id " occurs twice!" crlf)
          )
          (bind ?applied TRUE)
          ; find corresponding slot
          (bind ?slot-id (- ?key (* 100 ?sync-id)))
          (bind ?slot (wm-sync-get-slot-of-index ?fact ?slot-id))
          (if (not ?slot) then
            (printout error "Can't find slot for key " ?key 
                      " of template " ?template "." crlf)
          )
          ; actually modify
          ;check if slot is a single- od multifield
          (if (eq MULTIFIELD (type ?slot))
            then
            ; is it an override, add or remove key?
            (bind ?slot-index-diff (nth$ 2 ?slot))
            (bind ?slot (nth$ 1 ?slot))
            (if (eq ?slot-index-diff 0)
              then
              ;override
              (dyn-override-multifield ?fact ?slot (explode$ ?value)) ;only when changed
              else
              (if (eq ?slot-index-diff 1)
                then
                (dyn-add-to-multifield ?fact ?slot ?value)
                else
                (dyn-remove-from-multifield ?fact ?slot ?value)
              )
            )
            else
            (dyn-mod ?fact ?slot ?value) ;modifies only when there is a change
          )
        )
      )
      (if (not ?applied) then
        (printout error "sync-id " ?sync-id " could not be found!" crlf)
      )
    )
  )
)

(defrule worldmodel-sync-publish-worldmodel
  ;the master publishes the worldmodel repeatedly
  ; the worldmodel is represented as set of key-value pairs
  ; these pairs are sent together in messages with up to ?*MAX-MESSAGE-SIZE* bytes
  (time $?now)
  ?s <- (timer (name send-worldmodel-sync) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (lock-role MASTER)
  (peer-id private ?peer)
  (wm-sync-info (synced-templates $?templates))
  =>
  ;construct worldmodel msg
  (bind ?worldmodel (pb-create "llsf_msgs.Worldmodel"))
  (bind ?msg-size ?*MESSAGE-OVERHEAD-SIZE*)
  (progn$ (?templ ?templates)
    (delayed-do-for-all-facts ((?fact ?templ)) TRUE
      (bind ?fact-msg (pb-create "llsf_msgs.WorldmodelFact"))
      (pb-set-field ?fact-msg "template" (str-cat ?templ))
      (pb-set-field ?fact-msg "sync_id" (fact-slot-value ?fact sync-id))
      (bind ?fact-msg-size (+ ?*MESSAGE-OVERHEAD-SIZE* 4 (str-length ?templ)))
      ;construct submsg for each field
      (progn$ (?slot (fact-slot-names ?fact))
        (if (neq ?slot sync-id) then
          (bind ?kv-pair (pb-create "llsf_msgs.KeyValuePair"))
          (bind ?key (+ (* 100 (fact-slot-value ?fact sync-id))
                        (wm-sync-get-index-of-slot ?fact ?slot)))
          (pb-set-field ?kv-pair "key" ?key)
          (bind ?kv-msg-size (+ ?*MESSAGE-OVERHEAD-SIZE* 4))
          (if (deftemplate-slot-multip ?templ ?slot)
            then
            ; use string to represent the multifield
            (bind ?multifield-as-string (implode$ (fact-slot-value ?fact ?slot)))
            (pb-set-field ?kv-pair "value_string" ?multifield-as-string)
            (bind ?kv-msg-size (+ ?kv-msg-size (str-length ?multifield-as-string)))
            else
            ; use value directly for singlefields
            (bind ?value (fact-slot-value ?fact ?slot))
            (switch (type ?value)
              (case INTEGER then
                    (pb-set-field ?kv-pair "value_uint32" ?value)
                    (bind ?kv-msg-size (+ ?kv-msg-size 4)))
              (case STRING then
                    (pb-set-field ?kv-pair "value_string" ?value)
                    (bind ?kv-msg-size (+ ?kv-msg-size (str-length ?value))))
              (case SYMBOL then
                    (pb-set-field ?kv-pair "value_symbol" ?value)
                    (bind ?kv-msg-size (+ ?kv-msg-size (str-length ?value))))
              (case FLOAT then
                    (pb-set-field ?kv-pair "value_float" ?value)
                    (bind ?kv-msg-size (+ ?kv-msg-size 4)))
              (default (printout error "WM-publish with key " ?key " has invalid type "
                                 (type ?value) "." crlf))
            )
          )
          (pb-add-list ?fact-msg "pairs" ?kv-pair)
          (bind ?fact-msg-size (+ ?fact-msg-size ?kv-msg-size))
        )
      )
      ; split worldmodel into multiple peaces if the msg size gets too big
      (if (> (+ ?msg-size ?fact-msg-size) ?*MAX-MESSAGE-SIZE*)
        then
        ; send full msg
        (pb-broadcast ?peer ?worldmodel)
        (pb-destroy ?worldmodel)
        ;create new msg
        (bind ?worldmodel (pb-create "llsf_msgs.Worldmodel"))
        (bind ?msg-size ?*MESSAGE-OVERHEAD-SIZE*)
      )
      (pb-add-list ?worldmodel "facts" ?fact-msg)
      (bind ?msg-size (+ ?msg-size ?fact-msg-size))
    )
  )
  (pb-broadcast ?peer ?worldmodel)
  (pb-destroy ?worldmodel)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
)

;receive worldmodel
(defrule worldmodel-sync-receive-worldmodel
  ?msg <- (protobuf-msg (type "llsf_msgs.Worldmodel") (ptr ?p))
  (not (lock-role MASTER))
  (state ?state)
  (wm-sync-info (synced-templates $?templs))
  =>
  ; (printout t "***** Received Worldmodel *****" crlf)
  ; unwatch synced templates to have less sync stuff in the debug log
  (progn$ (?templ ?templs)
    (unwatch facts ?templ)
  )
  ;update worldmodel for each received fact
  (foreach ?fact-msg (pb-field-list ?p "facts")
    (bind ?sync-id (pb-field-value ?fact-msg "sync_id"))
    (bind ?fact-exists FALSE)
    (progn$ (?template ?templs)
      (do-for-fact ((?fact ?template)) (eq ?sync-id ?fact:sync-id)
        (bind ?fact-exists TRUE)
      )
    )
    ; create fact if it does not exist jet
    (if (not ?fact-exists) then
      (dyn-assert (str-cat "(" (pb-field-value ?fact-msg "template")
                           " (sync-id " ?sync-id
                           "))"))
    )
    ;update worldmodel for each key-value pair
    (bind ?pairs (pb-field-list ?fact-msg "pairs"))
    (progn$ (?kv-pair ?pairs)
      (worldmodel-sync-apply-key-value-msg ?kv-pair)
    )
  )
  (retract ?msg)
  ;reenable watching
  (progn$ (?templ ?templs)
    (watch facts ?templ)
  )
)

;send worldmodel change
(defrule worldmodel-sync-send-change
  (time $?now)
  ?wmc <- (worldmodel-change (key ?key) (value ?value)
			     (id ?id) (agent ?agent)
			     (last-sent $?ls&:(timeout ?now ?ls ?*WORLDMODEL-CHANGE-SEND-PERIOD*)))
  (not (lock-role MASTER))
  (peer-id private ?peer)
  =>
  ;set random id (needed by the master to determine if a change was already appied)
  (if (eq ?id 0) then
    (bind ?id (random 1 99999999))
  )
  (bind ?change-msg (pb-create "llsf_msgs.WorldmodelChange"))
  (bind ?pair-msg (pb-create "llsf_msgs.KeyValuePair"))
  (pb-set-field ?pair-msg "key" ?key)
  (switch (type ?value)
    (case INTEGER then (pb-set-field ?pair-msg "value_uint32" ?value))
    (case STRING then (pb-set-field ?pair-msg "value_string" ?value))
    (case SYMBOL then (pb-set-field ?pair-msg "value_symbol" ?value))
    (case FLOAT then (pb-set-field ?pair-msg "value_float" ?value))
    (default (printout error "WM-change with key " ?key " has invalid type "
                       (type ?value) "." crlf))
  )
  (pb-set-field ?change-msg "pair" ?pair-msg)
  
  (pb-set-field ?change-msg "agent" (str-cat ?agent))
  (pb-set-field ?change-msg "id" ?id)
  
  (pb-broadcast ?peer ?change-msg)
  (pb-destroy ?change-msg)
  (modify ?wmc (last-sent ?now) (id ?id))
)

;the master does not have to send the change, because it sends the whole worldmodel
(defrule worldmodel-sync-retract-as-master
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?wmc <- (worldmodel-change)
  (lock-role MASTER)
  =>
  (retract ?wmc)
)

;receive worldmodel change
(defrule worldmodel-sync-receive-change
  ?pmsg <- (protobuf-msg (type "llsf_msgs.WorldmodelChange") (ptr ?p))
  (lock-role MASTER)
  ?arf <- (already-received-wm-changes $?arc)
  (peer-id private ?peer)
  (time $?now)
  (wm-sync-info (synced-templates $?synced-templates))
  =>
  ;ensure that this change was not already applied
  (bind ?id (pb-field-value ?p "id"))
  (if (not (member$ ?id ?arc)) then
    (retract ?arf)
    (assert (already-received-wm-changes (append$ ?arc ?id)))
    (bind ?pair (pb-field-value ?p "pair"))
    ; apply
    (worldmodel-sync-apply-key-value-msg ?pair)
  )
  (retract ?pmsg)
  ;send acknowledgment
  (bind ?msg-ack (pb-create "llsf_msgs.WorldmodelChangeAck"))
  (pb-set-field ?msg-ack "id" ?id)  
  (pb-broadcast ?peer ?msg-ack)
  (pb-destroy ?msg-ack)
)

(defrule worldmodel-sync-receive-change-ack
  ?pmsg <- (protobuf-msg (type "llsf_msgs.WorldmodelChangeAck") (ptr ?p))
  (not (lock-role MASTER))
  =>
  (do-for-fact ((?wmc worldmodel-change))
	       (eq ?wmc:id (pb-field-value ?p "id"))
    (retract ?wmc)
  )
  (retract ?pmsg)
)

(defrule worldmodel-sync-apply-delayed-worldmodel-change
  ?dwf <- (delayed-worldmodel-change REMOVE_INCOMING ?machine-name ?field-to-remove ?agent-removing $?)
  ?machine <- (machine (name ?machine-name)
		       (incoming $?incoming&:(member$ ?field-to-remove ?incoming))
		       (incoming-agent $?incoming-agent&:(member$ ?agent-removing ?incoming-agent)))
  =>
  (modify ?machine (incoming (delete-member$ ?incoming ?field-to-remove))
	  (incoming-agent (delete-member$ ?incoming-agent ?agent-removing)))
)

(defrule worldmodel-sync-delete-delayed-worldmodel-change-after-timeout
  (time $?now)
  ?dwf <- (delayed-worldmodel-change ? ? ? ? $?time-asserted&:(timeout ?now ?time-asserted ?*DELAYED-WORLDMODEL-CHANGE-TIMEOUT*))
  =>
  (retract ?dwf)
)

(defrule worldmodel-sync-stop-using-initial-sync-ids
  ; from now on use random sync-ids
  (phase EXPLORATION|PRODUCTION)
  =>
  (printout t "Stop using initial ids" crlf)
  (bind ?*USING-INITIAL-SYNC-IDS* FALSE)
)

(defrule worldmodel-sync-set-sync-ids
  ; this rule adds a rule for each template that have to be synced.
  ; the added rule sets the sync id for the template
  (wm-sync-info (synced-templates $?templates))
  =>
  (progn$ (?temp ?templates)
    (printout t "Synchronizing template " ?temp crlf)
    (bind ?new-rule (format nil "(defrule worldmodel-sync-set-sync-id-for-%s %n ?f <- (%s (sync-id 0)) %n => %n (modify ?f (sync-id (worldmodel-sync-get-sync-id))))" ?temp ?temp))
    (build ?new-rule)
  )
)

(defrule worldmodel-sync-retract-with-sync-id
  ; this rule adds a rule for each template that have to be synced.
  ; the added rule retracts a fact with given sync-id if requested
  (wm-sync-info (synced-templates $?templates))
  =>
  (progn$ (?temp ?templates)
    (bind ?new-rule (format nil "(defrule worldmodel-sync-retract-with-sync-id-%s %n ?r <- (synced-fact-to-retract ?sid) %n ?f <- (%s (sync-id ?sid)) %n => %n (retract ?r) %n (retract ?f))" ?temp ?temp))
    (build ?new-rule)
  )
)
