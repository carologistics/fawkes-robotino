(deftemplate lock
  (slot type (type SYMBOL) (allowed-values GET REFUSE ACCEPT RELEASE))
  (slot agent (type SYMBOL))
  (slot resource (type SYMBOL))
)

(deftemplate locked-resource
	(slot resource (type SYMBOL))
	(slot agent (type SYMBOL))
)

(deffacts lock-facts
  (signal (type send-lock-msg) (time (create$ 0 0)) (seq 1))
  (signal (type send-master-announce) (time (create$ 0 0)) (seq 1))
  (init-locking)
  ;DEBUG:
  ;(lock (type GET) (agent ?*ROBOT-NAME*) (resource INS))
)

(defrule lock-init
  ?i <- (init-locking)
  (time $?now)
  =>
  (bind ?*MASTER-TIMEOUT* 2.0)
  (printout t "Initial lock-role is SLAVE" crlf)
  ;seed for random numbers (needed for problem solving if there are two masters)
  (seed (nth$ 2 ?now))
  (retract ?i)
  (assert (master-last-seen ?now)
          (lock-role SLAVE)
  )  
)

;;;;MASTER/SLAVE selection;;;;

(defrule lock-master-announce
  (lock-role MASTER)
  (time $?now)
  ?s <- (signal (type send-master-announce) (time $?t&:(timeout ?now ?t ?*MASTER-ANNOUNCE-PERIOD*)) (seq ?seq))
  =>
  (printout t "Announcing MASTER role" crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
  (bind ?lock-msg (pb-create "llsf_msgs.LockMasterAnnounce"))
  (pb-set-field ?lock-msg "agent" (str-cat ?*ROBOT-NAME*))
  (pb-broadcast ?lock-msg)
  (pb-destroy ?lock-msg)
)

(defrule lock-recognize-master
  ?msg <- (protobuf-msg (type "llsf_msgs.LockMasterAnnounce") (ptr ?p))
  ?r <- (lock-role ?role)
  ?mls <- (master-last-seen $?)
  (time $?now)
  =>
  (bind ?a (sym-cat (pb-field-value ?p "agent")))
  (retract ?mls ?msg)
  (assert (master-last-seen ?now))
  (bind ?*MASTER-TIMEOUT* 10.0)
  (if (and (eq ?role MASTER) (not (eq ?a ?*ROBOT-NAME*)))
      then
      (printout t "TWO MASTERS DETECTED, WAITING RANDOM TIME AS SLAVE!!!")
      (retract ?r)
      (assert (lock-role SLAVE))
      ;TODO: are there consequences of the change?
      (bind ?*MASTER-TIMEOUT* (float (random 0 10)))
  )
)

(defrule lock-getting-master
  ?r <- (lock-role SLAVE)
  (time $?now)
  ?mls <- (master-last-seen $?last&:(timeout ?now ?last ?*MASTER-TIMEOUT*))
  =>
  (printout t "MASTER timed out -> Getting MASTER" crlf)
  (retract ?r)
  (assert (lock-role MASTER))
)

(defrule lock-retract-get
	?l <- (lock (type GET) (agent ?a) (resource ?r))
	?la <- (lock (type ACCEPT) (agent ?a) (resource ?r))
	=>
	(retract ?l)
)

;;;;SENDING and RECEIVING;;;;

(defrule lock-send-message
  (declare (salience ?*PRIORITY-LOCK-SEND*))
  (time $?now)
  ?s <- (signal (type send-lock-msg) (time $?t&:(timeout ?now ?t ?*LOCK-PERIOD*)) (seq ?seq))
  =>
  (printout t "Sending all lock-messages:" crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
	(do-for-all-facts (?lock lock) TRUE
		(printout "   type " ?lock:type " of " ?lock:resource " from agent " ?lock:agent crlf)
		(bind ?lock-msg (pb-create "llsf_msgs.LockMessage"))
		(pb-set-field ?lock-msg "type" ?lock:type)
		(pb-set-field ?lock-msg "agent" (str-cat ?lock:agent))
		(pb-set-field ?lock-msg "resource" (str-cat ?lock:resource))
		(pb-broadcast ?lock-msg)
		(pb-destroy ?lock-msg)
	)
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
  (if (eq ?role MASTER) then
		(if (or (eq ?type GET) (eq ?type RELEASE)) then
			(assert (lock (type ?type) (agent ?a) (resource ?r)))
		)
	else
		(if (or (eq ?type ACCEPT) (eq ?type REFUSE)) then
			(if (eq ?a ?*ROBOT-NAME*) then
				(assert (lock (type ?type) (agent ?a) (resource ?r)))
			)
		)
  )
)

(defrule lock-accept-get
	(lock-role MASTER)
	?l <- (lock (type GET) (agent ?a) (resource ?r))
	(not (locked-resource (resource ?r) (agent ?)))
  =>
	(assert (locked-resource (resource ?r) (agent ?a))
					(lock (type ACCEPT) (agent ?a) (resource ?r)))
	(if (not (eq ?a ?*ROBOT-NAME*))
		(retract ?l)
	)
)

(defrule lock-refuse-get
	(lock-role MASTER)
	?l <- (lock (type GET) (agent ?a) (resource ?r))
	?lm <- (locked-resource (resource ?r) (agent ?))
  =>
	(assert (lock (type REFUSE) (agent ?a) (resource ?r)))
	(if (not (eq ?a ?*ROBOT-NAME*))
		(retract ?l)
	)
)

(defrule lock-release
	(lock-role MASTER)
	?l <- (lock (type RELEASE) (agent ?a) (resource ?r))
	?lm <- (locked-resource (resource ?r) (agent ?))
  =>
	(retract ?l ?lm)
)
