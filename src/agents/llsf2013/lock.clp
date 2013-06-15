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
)

(defrule lock-init
  ?i <- (init-locking)
  (time $?now)
  =>
  (bind ?*CURRENT-MASTER-TIMEOUT* ?*INITIAL-MASTER-TIMEOUT*)
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
  ;(printout t "Announcing MASTER role" crlf)
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
  (bind ?*CURRENT-MASTER-TIMEOUT* ?*ROBOT-TIMEOUT*)
  (if (and (eq ?role MASTER) (not (eq ?a ?*ROBOT-NAME*)))
      then
      (printout t "TWO MASTERS DETECTED, WAITING RANDOM TIME AS SLAVE!!!")
      (retract ?r)
      (assert (lock-role SLAVE))
      ;TODO: are there consequences of the change?
      (bind ?*CURRENT-MASTER-TIMEOUT* (float (random 0 10)))
  )
)

(defrule lock-getting-master
  ?r <- (lock-role SLAVE)
  (time $?now)
  ?mls <- (master-last-seen $?last&:(timeout ?now ?last ?*CURRENT-MASTER-TIMEOUT*))
  =>
  (printout t "MASTER timed out -> Getting MASTER" crlf)
  (retract ?r)
  (assert (lock-role MASTER))
)

(defrule lock-retract-get
  (declare (salience ?*PRIORITY-HIGH*))
	?l <- (lock (type GET) (agent ?a) (resource ?r))
	?la <- (lock (type ACCEPT) (agent ?a) (resource ?r))
	=>
	(retract ?l)
	;(printout t "----- Get retracted: " ?r crlf)
)

(defrule lock-retract-release
  (declare (salience ?*PRIORITY-HIGH*))
	?l <- (lock (type GET) (agent ?a) (resource ?r))
	?lr <- (lock (type RELEASE) (agent ?a) (resource ?r))
	=>
	(retract ?lr)
	;(printout t "----- Release retracted: " ?r crlf)
)

;;;;SENDING and RECEIVING;;;;

(defrule lock-send-message
  (declare (salience ?*PRIORITY-LOCK-SEND*))
  (time $?now)
  ?s <- (signal (type send-lock-msg) (time $?t&:(timeout ?now ?t ?*LOCK-PERIOD*)) (seq ?seq))
	(lock-role ?role)
  =>
  ;(printout t "Sending all lock-messages:" crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
	(do-for-all-facts ((?lock lock)) TRUE
		(if (or (and (eq ?role MASTER) (or (eq ?lock:type ACCEPT) (eq ?lock:type REFUSE)))
						(and (eq ?role SLAVE) (or (eq ?lock:type GET) (eq ?lock:type RELEASE)))) then
			;(printout t "   type " ?lock:type " of " ?lock:resource " from agent " ?lock:agent crlf)
			(bind ?lock-msg (pb-create "llsf_msgs.LockMessage"))
			(pb-set-field ?lock-msg "type" ?lock:type)
			(pb-set-field ?lock-msg "agent" (str-cat ?lock:agent))
			(pb-set-field ?lock-msg "resource" (str-cat ?lock:resource))
			(pb-broadcast ?lock-msg)
			(pb-destroy ?lock-msg)
		)
	)
)

(defrule lock-receive-message
  ?msg <- (protobuf-msg (type "llsf_msgs.LockMessage") (ptr ?p))
  (lock-role ?role)
  =>
  (bind ?type (sym-cat (pb-field-value ?p "type")))
  (bind ?a (str-cat (pb-field-value ?p "agent")))
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
  (if (not (eq ?a ?*ROBOT-NAME*)) then
    (retract ?l)
  )
)

(defrule lock-retract-already-accepted-get
  (lock-role MASTER)
  ?l <- (lock (type GET) (agent ?a) (resource ?r))
  (locked-resource (resource ?r) (agent ?a))
  =>
  (retract ?l)
  (printout t "retracting already accepted locking GET" crlf)
)

(defrule lock-refuse-get
  (lock-role MASTER)
  ?l <- (lock (type GET) (agent ?a) (resource ?r))
  ?lm <- (locked-resource (resource ?r) (agent ~?a))	
  =>
  (assert (lock (type REFUSE) (agent ?a) (resource ?r)))
  (if (not (eq ?a ?*ROBOT-NAME*)) then
    (retract ?l)
  )
)

(defrule lock-release
  (lock-role MASTER)
  ?l <- (lock (type RELEASE) (agent ?a) (resource ?r))
  ?lm <- (locked-resource (resource ?r) (agent ?a))
  =>
  (retract ?l ?lm)
)

(defrule lock-retract-accept-after-release
  (not (locked-resource (resource ?r) (agent ?a)))
  ?l <- (lock (type ACCEPT) (agent ?a) (resource ?r))
  =>
  (retract ?l)
)

(defrule lock-retract-refuse-after-accept
  (lock (type ACCEPT) (agent ?a) (resource ?r))
  ?lr <- (lock (type REFUSE) (agent ?a) (resource ?r))
  =>
  (retract ?lr)
)

(defrule lock-delete-all-locks-when-changing-the-phase
  (declare (salience ?*PRIORITY-HIGH*))
  (change-phase ?new-phase) 
  =>
  (do-for-all-facts ((?lock lock)) TRUE
    (retract ?lock)
  )
  (do-for-all-facts ((?lock locked-resource)) TRUE
    (retract ?lock)
  ) 
)
