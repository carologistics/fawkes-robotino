(deftemplate lock
  (slot type (type SYMBOL) (allowed-values GET REFUSE ACCEPT RELEASE))
  (slot agent (type SYMBOL))
  (slot resource (type SYMBOL))
)

(deffacts locking
  ;TODO: extend to send more than one lock at a time
  (signal (type send-lock-msg) (time (create$ 0 0)) (seq 1))

  ;DEBUG:
  ;(lock (type GET) (agent ?*ROBOT-NAME*) (resource INS))
)

;;;;SENDING and RECEIVING;;;;

(defrule lock-send-massege
  (declare (salience ?*PRIORITY-LOCK-SEND*))
  (lock (type ?type) (agent ?a) (resource ?r))
  (time $?now)
  ?s <- (signal (type send-lock-msg) (time $?t&:(timeout ?now ?t ?*LOCK-PERIOD*)) (seq ?seq))
  =>
  (printout t "Sending lock-message with type " ?type " of " ?r " from agent " ?a crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
  (printout t "bla 1" crlf)
  (bind ?lock-msg (pb-create "llsf_msgs.LockMessage"))
  (printout t "bla 2" crlf)
  (pb-set-field ?lock-msg "type" ?type)
  (printout t "bla 3" crlf)
  (pb-set-field ?lock-msg "agent" (str-cat ?a))
  (printout t "bla 4" crlf)
  (pb-set-field ?lock-msg "resource" (str-cat ?r))
  (printout t "bla 5" crlf)
  (pb-broadcast ?lock-msg)
  (printout t "bla 6" crlf)
  (pb-destroy ?lock-msg)
  (printout t "bla 7" crlf)
)

(defrule lock-receive-message
  ?msg <- (protobuf-msg (type "llsf_msgs.LockMessage") (ptr ?p))
  (lock-role ?role)
  =>
  (bind ?type (sym-cat (pb-field-value ?p "type")))
  (bind ?a (sym-cat (pb-field-value ?p "agent")))
  (bind ?r (sym-cat (pb-field-value ?p "resource")))
  (printout t "Received lock message with type " ?type " of " ?r " from " ?a crlf)
  (retract ?msg)
  (if (eq ?role MASTER)
      then
      (if (or (eq ?type GET) (eq ?type RELEASE))
	  then
	  (assert (lock (type ?type) (agent ?a) (resource ?r)))
      )
      else
      (if (or (eq ?type ACCEPT) (eq ?type REFUSE))
	  then
	  (if (eq ?a ?*ROBOT-NAME*)
	      then
	      (assert (lock (type ?type) (agent ?a) (resource ?r)))
	  )
      )
  )
)
