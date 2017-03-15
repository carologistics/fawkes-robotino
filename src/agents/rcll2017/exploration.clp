
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Created: Fri Apr 26 18:38:18 2013 (Magdeburg)
;  Copyright  2013  Frederik Zwilling
;             2013  Alexander von Wirth 
;             2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;Read exploration rows from config
(defrule exp-cfg-get-row
  "Read configuration for exploration row order of machines"
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (confval (path "/clips-agent/rcll2016/exploration/row-high") (list-value $?row-high))
  (confval (path "/clips-agent/rcll2016/exploration/row-mid") (list-value $?row-mid))
  (confval (path "/clips-agent/rcll2016/exploration/row-low") (list-value $?row-low))
  (confval (path "/clips-agent/rcll2016/exploration/row") (value ?agent-row))
  =>
  (bind $?row-high-sym (create$))
  (progn$ (?m ?row-high)
    (bind $?row-high-sym (append$ ?row-high-sym (sym-cat ?m)))
  )
  (bind $?row-mid-sym (create$))
  (progn$ (?m ?row-mid)
    (bind $?row-mid-sym (append$ ?row-mid-sym (sym-cat ?m)))
  )
  (bind $?row-low-sym (create$))
  (progn$ (?m ?row-low)
    (bind $?row-low-sym (append$ ?row-low-sym (sym-cat ?m)))
  )
  (bind ?row (sym-cat ?agent-row))
  (switch ?row
    (case HIGH then (assert (exp-row (name HIGH) (row ?row-high-sym))))
    (case MID then (assert (exp-row (name MID) (row ?row-mid-sym))))
    (case LOW then (assert (exp-row (name LOW) (row ?row-low-sym))))
    (default (printout warn "NO EXPLORATION ROW DEFINED! UNABLE TO DO EXPLORATION!" crlf))
  )
  (printout t "At first, I am exploring the " ?row " row" crlf)
  
)

(defrule exp-turn-line-over
  "Turn over line if we start on the left field"
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (team-color MAGENTA)
  ?er <- (exp-row (row $?row))
  (not (exp-line-already-turned))
  =>
  (bind $?turned-row (create$))
  (progn$ (?m ?row)
    (bind ?turned-row (insert$ ?turned-row 1 ?m))
  )
  (modify ?er (row ?turned-row))
  (assert (exp-line-already-turned))
)

(defrule exp-determine-first-quadrant
  "Determine the first machine in dependency of the role"
  (phase EXPLORATION)
  (exp-row (row $?row))
  (team-color ?team-color&~nil)
  (have-exp-info)
  =>
  ;find first machine in row
  (bind ?first NONE)
  (bind ?q nil)
  (progn$ (?row-qn ?row)
    (do-for-fact ((?q exploration-quadrant)) (eq ?q:name ?row-qn)
      (progn$ (?m ?q:zones)
        (do-for-fact ((?me zone-exploration)) (and (eq ?m ?me:name) (eq ?me:team ?team-color))
          (bind ?first ?q:name)
        )
        (if (neq ?first NONE)
          then
          (break)
        )
      )
    )
  )
  (assert (exp-next-quadrant ?first)
    (exp-doing-first-quadrant)
  )
)

(defrule exp-handle-no-own-machine-in-row
  "If exploration row is empty(explored) change exploration tactic from LINE to NEAREST"
  (declare (salience ?*PRIORITY-WM*))
  (exp-next-quadrant NONE)
  ?s <- (state EXP_START)
  ?et <- (exp-tactic LINE)
  (exp-row (row $?row))
  =>
  ;go directly in the nearest-mode
  (printout t "No zone of my team is in my line" crlf)
  (retract ?s ?et)
  (assert (state EXP_IDLE)
          (exp-tactic NEAREST)
          (goalmachine (nth$ (length$ ?row) ?row)) ;getting nearest from last in row
  )
)

(defrule exp-start
  "Set up the state. There are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again."
  (phase EXPLORATION)
  ?st <- (exploration-start)
  (team-color ?team-color)
  =>
  (retract ?st)
  (assert (state EXP_START)
          (exp-tactic LINE)
          (timer (name send-machine-reports))
  )
  (if (eq ?team-color nil) then
    (printout error "Ouch, starting exploration but I don't know my team color" crlf)
  )
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

(defrule exp-lock-next
  "Request lock for the next exploration quadrant"
  (phase EXPLORATION)
  ?s <- (state EXP_START)
  (exp-tactic LINE)
  (exp-next-quadrant ?q)
  (not (driven-to-waiting-point))
  =>
  (printout t "First exploration quadrant: " ?q crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?q))
    (exp-next-quadrant ?q)
  )
)


(defrule exp-goto-first
  "Go to the first exploration quadrant at normal speed, the rest is done more slowly"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?f <- (exp-doing-first-quadrant)
  (exp-next-quadrant ?q)
  (exploration-quadrant (name ?qn) (zones $?zones))
  (zone-exploration (times-searched ?ts&:(= ?ts 0)) (name ?name&:(member$ ?name ?zones)))
  =>
  (retract ?s ?f)
  (assert (state EXP_GOTO_FIRST))
  (skill-call goto place ?name)
)

(defrule exp-goto-first-final
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_FIRST)
  ?skill-f <- (skill-done (name "goto") (status FINAL))
  =>
  (retract ?s)
  (assert (state EXP_SEARCHING))
)

(defrule exp-goto-next
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  (exp-next-quadrant ?qn)
  (exploration-quadrant (name ?qn) (zones $?zones))
  (zone-exploration (times-searched ?ts&:(= ?ts 0)) (name ?name&:(member$ ?name ?zones)))
  =>
  (retract ?s)
  (assert (state EXP_GOTO_NEXT))
  (skill-call goto place ?name)
)


(defrule exp-goto-next-final
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_NEXT)
  ?skill-f <- (skill-done (name "goto") (status FINAL))
  =>
  (retract ?s)
  (assert (state EXP_SEARCHING))
)


(defrule exp-stopped-in-quadrant
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx 0.05)) (vy ?vy&:(< ?vy 0.05)) (omega ?w&:(< ?w 0.05))
  )
  (
)


(defrule exp-found-line
  "Found a line that is within an unexplored zone."
  (phase EXPLORATION)
  (state EXP_SEARCHING)
  ?line <- (LaserLineInterface (id ?id&~:(str-index "moving_avg" ?id))
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
  )
  =>
  (printout t "Line in zone: " (laser-line-get-zone ?id ?timestamp))
)

(defrule exp-require-resource-locking
  "Request lock for next machine."
  (phase EXPLORATION)
  ?s <- (state EXP_FOUND_NEXT_MACHINE)
  (exp-next-quadrant ?nextMachine)
  =>
  (printout t "Require lock for " ?nextMachine crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
          (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?nextMachine)))
)

(defrule exp-check-resource-locking
  "Handle a lock that was accepted by the master"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_REQUIRED)
  (exp-next-quadrant ?nextMachine)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  =>
  (printout t "Lock accepted." crlf)
  (retract ?s ?l)
  (assert (state EXP_LOCK_ACCEPTED))
)

(defrule exp-receive-exploration-info
  "Receive Zone/Team mapping and update zone-exploration facts accordingly."
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p))
  (not (have-exp-info))
  (lock-role ?role)
  =>
  (retract ?pbm)
  (foreach ?m (pb-field-list ?p "zones")
    (bind ?name (sym-cat (pb-field-value ?m "zone")))
    (bind ?team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?me zone-exploration)) (eq ?me:name ?name)
      (printout t "Zone " ?name " is from team " ?team crlf)
      (modify ?me (team ?team))
    )
  )
  (assert (exp-machines-initialized)
    (have-exp-info)
  )
)


