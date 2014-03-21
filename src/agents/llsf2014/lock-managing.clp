;---------------------------------------------------------------------------
;  lock-managing.clp - managing of locking mechanisms
;       includes: deftemplates, lock-managing, Master/Slave selection,
;                  networking, lock-accepting
;
;  Created: Fri Feb 28 19:16:02 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate lock
  (slot type (type SYMBOL) (allowed-values GET REFUSE ACCEPT RELEASE RELEASE_RVCD))
  (slot agent (type STRING))
  (slot resource (type SYMBOL))
)

(deftemplate locked-resource
  (slot resource (type SYMBOL))
  (slot agent (type STRING))
)

(deffacts lock-facts
  (timer (name send-lock-msg) (time (create$ 0 0)) (seq 1))
  (timer (name send-master-announce) (time (create$ 0 0)) (seq 1))
  (timer (name send-status-of-all-locks) (time (create$ 0 0)) (seq 1))
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
  (declare (salience ?*PRIORITY-LOCK-SEND*))
  (lock-role MASTER)
  (time $?now)
  ?s <- (timer (name send-master-announce) (time $?t&:(timeout ?now ?t ?*MASTER-ANNOUNCE-PERIOD*)) (seq ?seq))
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
      (printout warn "TWO MASTERS DETECTED, WAITING RANDOM TIME AS SLAVE!!!" crlf)
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

;;;;SENDING and RECEIVING;;;;

(defrule lock-send-message
  (declare (salience ?*PRIORITY-LOCK-SEND*))
  (time $?now)
  ?s <- (timer (name send-lock-msg) (time $?t&:(timeout ?now ?t ?*LOCK-PERIOD*)) (seq ?seq))
  (lock-role ?role)
  =>
  ;(printout t "Sending all lock-messages:" crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
  (delayed-do-for-all-facts ((?lock lock)) TRUE
    (if (or (and (eq ?role MASTER) (or (eq ?lock:type ACCEPT) (eq ?lock:type REFUSE) (eq ?lock:type RELEASE_RVCD)))
	    (and (eq ?role SLAVE) (or (eq ?lock:type GET) (eq ?lock:type RELEASE)))) 
      then
      ;(printout t "   type " ?lock:type " of " ?lock:resource " from agent " ?lock:agent crlf)
      (bind ?lock-msg (pb-create "llsf_msgs.LockMessage"))
      (pb-set-field ?lock-msg "type" ?lock:type)
      (pb-set-field ?lock-msg "agent" (str-cat ?lock:agent))
      (pb-set-field ?lock-msg "resource" (str-cat ?lock:resource))
      (pb-broadcast ?lock-msg)
      (pb-destroy ?lock-msg)

      ;send RELEASE_RCVD only once
      (if (eq ?lock:type RELEASE_RVCD)
        then
	(retract ?lock)
      )
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
  ;(printout t "Received lock message with type " ?type " of " ?r " from " ?a crlf)
  (retract ?msg)
  (if (eq ?role MASTER)
    then
    (if (or (eq ?type GET) (eq ?type RELEASE)) then
      (assert (lock (type ?type) (agent ?a) (resource ?r)))
    )
    else
    (if (or (eq ?type ACCEPT) (eq ?type REFUSE) (eq ?type RELEASE_RVCD))
      then
      (if (eq ?a ?*ROBOT-NAME*)
        then
        (assert (lock (type ?type) (agent ?a) (resource ?r)))
      )
    )
  )
)

(defrule lock-send-status-of-all-locks-to-slaves
  (time $?now)
  ?s <- (timer (name send-status-of-all-locks) (time $?t&:(timeout ?now ?t ?*LOCK-STATUS-SEND-PERIOD*)) (seq ?seq))
  (lock-role MASTER)
  =>
  ;(printout t "Sending all lock-messages:" crlf)
  (modify ?s (time ?now) (seq (+ ?seq 1)))
  (bind ?complete-lock-msg (pb-create "llsf_msgs.CompleteLockStatus"))
  (do-for-all-facts ((?lock locked-resource)) TRUE
    (bind ?lock-msg (pb-create "llsf_msgs.LockMessage"))
    (pb-set-field ?lock-msg "type" ACCEPT)
    (pb-set-field ?lock-msg "agent" (str-cat ?lock:agent))
    (pb-set-field ?lock-msg "resource" (str-cat ?lock:resource))
    (pb-add-list ?complete-lock-msg "locks" ?lock-msg)
  )
  (pb-broadcast ?complete-lock-msg)
  (pb-destroy ?complete-lock-msg)
)


(defrule lock-receive-status-of-all-locks-from-master
  ?msg <- (protobuf-msg (type "llsf_msgs.CompleteLockStatus") (ptr ?p))
  (lock-role SLAVE)
  =>
  ;(printout t "Receiving all locks:" crlf)
  ;retract old locks
  (do-for-all-facts ((?lock locked-resource)) TRUE
    (retract ?lock)
  )
  ;read current locks
  (foreach ?lock (pb-field-list ?p "locks")
    (bind ?a (str-cat (pb-field-value ?lock "agent")))
    (bind ?r (sym-cat (pb-field-value ?lock "resource")))
    (assert (locked-resource (agent ?a) (resource ?r)))
  )
)

;;;;;; accepting, releasing and refusing locks ;;;;;;;


(defrule lock-accept-get
  (declare (salience ?*PRIORITY-LOCK-HIGH*))
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
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  (lock-role MASTER)
  ?l <- (lock (type GET) (agent ?a) (resource ?r))
  (locked-resource (resource ?r) (agent ?a))
  =>
  (retract ?l)
  (printout t "Accept already locked GET" crlf)
  (assert (lock (type ACCEPT) (agent ?a) (resource ?r)))
)

(defrule lock-refuse-get
  (declare (salience ?*PRIORITY-LOCK-HIGH*))
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
  (declare (salience ?*PRIORITY-LOCK-HIGH*))
  (lock-role MASTER)
  ?l <- (lock (type RELEASE) (agent ?a) (resource ?r))
  ?lm <- (locked-resource (resource ?r) (agent ?a))
  =>
  (retract ?l ?lm)
  (if (neq ?a ?*ROBOT-NAME*)
    then
    (assert (lock (type RELEASE_RVCD) (agent ?a) (resource ?r)))
  )
  (delayed-do-for-all-facts ((?accept lock)) (and (eq ?accept:type ACCEPT) (eq ?accept:agent ?a) (eq ?accept:resource ?r))
    (retract ?accept)
  )
)

(defrule lock-received-old-release
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  (lock-role MASTER)
  ?l <- (lock (type RELEASE) (agent ?a) (resource ?r))
  =>
  (retract ?l)
  (if (neq ?a ?*ROBOT-NAME*)
    then
    (assert (lock (type RELEASE_RVCD) (agent ?a) (resource ?r)))
  )
)

(defrule lock-retract-accept-after-release
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  (lock (type RELEASE) (agent ?a) (resource ?r))
  ?l <- (lock (type ACCEPT) (agent ?a) (resource ?r))
  =>
  (retract ?l)
)

(defrule lock-retract-refuse-after-accept
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  (lock (type ACCEPT) (agent ?a) (resource ?r))
  ?lr <- (lock (type REFUSE) (agent ?a) (resource ?r))
  =>
  (retract ?lr)
)

(defrule lock-retract-accepted-get
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  ?l <- (lock (type GET) (agent ?a) (resource ?r))
  ?la <- (lock (type ACCEPT) (agent ?a) (resource ?r))
  =>
  (retract ?l)
  ;(printout t "----- Get retracted: " ?r crlf)
)

(defrule lock-retract-release
  (declare (salience ?*PRIORITY-HIGH*))
  ?lrcvd <- (lock (type RELEASE_RVCD) (agent ?a) (resource ?r))
  ?lr <- (lock (type RELEASE) (agent ?a) (resource ?r))
  =>
  (retract ?lrcvd ?lr)
  ;(printout t "----- Release retracted: " ?r crlf)
)

(defrule lock-retract-old-release-received
  (declare (salience ?*PRIORITY-LOCK-CLEAN*))
  ?l <- (lock (type RELEASE_RVCD) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?r))
  (not (lock (type RELEASE) (agent ?a) (resource ?r)))
  =>
  (retract ?l)
)

(defrule lock-delete-all-locks-when-changing-the-phase
  (declare (salience ?*PRIORITY-HIGH*))
  (change-phase ?new-phase) 
  =>
  (delayed-do-for-all-facts ((?lock lock)) TRUE
    (retract ?lock)
  )
  (delayed-do-for-all-facts ((?lock locked-resource)) TRUE
    (retract ?lock)
  ) 
)

(defrule lock-delete-locks-of-lost-robot
  (time $?now)
  ?ar <- (active-robot (name ?name) (last-seen $?last-seen&:(timeout ?now ?last-seen ?*ROBOT-TIMEOUT*)))
  =>
  (printout warn "Lost connection to " ?name " Deliting all of its locks!" crlf)
  (delayed-do-for-all-facts ((?lock lock)) (eq (str-cat ?lock:agent) (str-cat ?name))
    (retract ?lock)
  )
  (delayed-do-for-all-facts ((?resource locked-resource)) (eq (str-cat ?resource:agent) (str-cat ?name))
    (retract ?resource)
  )
  (retract ?ar)
)

(defrule debug-check-inconsistency
  (lock (type REFUSE) (agent ?a) (resource ?res))
  (lock (type ACCEPT) (agent ?a) (resource ?res))
  =>
  (printout warn "Found inconsistency in locks (accept + refuse)!!!!!!!" crlf)
)
