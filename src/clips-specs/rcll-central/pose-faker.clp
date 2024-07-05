(deftemplate pose-faker-info
  (slot robot (type SYMBOL))
  (slot current-zone (type SYMBOL))
  (slot target-zone (type SYMBOL))
)

(defrule pose-faker-init-mps-targets
  (domain-fact (name mps-type) (param-values ?mps ?))
  (domain-fact (name zone-content) (param-values ?zone ?mps))
  (wm-fact (key game-found-tag ori args? m ?mps) (type UINT) (value ?ori))
  (not (domain-fact (name mps-poi)))
   =>
   (bind ?x (string-to-field (sub-string 4 4 ?zone)))
   (bind ?y (string-to-field (sub-string 5 5 ?zone)))
   (bind ?input (+ (* 10 ?x) ?y 10))
   (bind ?output (+ (* 10 ?x) ?y -10))
   (if (eq ?ori 45) then
     (bind ?input (+ ?input 1)) ; y+1 compared to 0
     (bind ?output (+ ?output -1)) ; y-1 compared to 0
   )
   (if (eq ?ori 90) then
     (bind ?input (+ ?input -10 1))
     (bind ?output (+ ?output 10 -1))
   )
   (if (eq ?ori 135) then
     (bind ?input (+ ?input -20 1))
     (bind ?output (+ ?output 20 -1))
   )
   (if (eq ?ori 180) then
     (bind ?input (+ ?input -20))
     (bind ?output (+ ?output 20))
   )
   (if (eq ?ori 225) then
     (bind ?input (+ ?input -20 -1))
     (bind ?output (+ ?output 20 1))
   )
   (if (eq ?ori 270) then
     (bind ?input (+ ?input -10 -1))
     (bind ?output (+ ?output 10 1))
   )
   (if (eq ?ori 315) then
     (bind ?input (+ ?input -1))
     (bind ?output (+ ?output 1))
   )
   (bind ?mirror-input FALSE)
   (bind ?mirror-output FALSE)
   ; if the input or output is in the other field-halve, we need to account for that
   (if (< ?input 10) then
     (bind ?input (+ ?input 10))
     (bind ?mirror-input TRUE)
   )
   (if (< ?output 10) then
     (bind ?output (+ ?output 10))
     (bind ?mirror-output TRUE)
   )
   (bind ?input-zone (sym-cat (str-cat (sub-string 1 3 ?zone) ?input)))
   (bind ?output-zone (sym-cat (str-cat (sub-string 1 3 ?zone) ?output)))
   (if ?mirror-input then (bind ?input-zone (mirror-name ?input-zone)))
   (if ?mirror-output then (bind ?output-zone (mirror-name ?output-zone)))
   (printout t "INPUT: " ?input-zone crlf)
   (printout t "OUTPUT: " ?output-zone crlf)
   (assert (domain-fact (name mps-poi) (param-values ?mps INPUT ?input-zone)))
   (assert (domain-fact (name mps-poi) (param-values ?mps OUTPUT ?output-zone)))
)

(defrule pose-faker-fake-pose-start
  (domain-fact (name at) (param-values ?r ?start-mps ?start-side))
  (skill (name go-wait|move) (skill-string ?sks) (start-time $?time)
    (skiller ?skiller&:(str-index ?r ?skiller))
  )
  (domain-fact (name mps-poi) (paramvalues ?mps ?side ?zone))
  (test (str-index (str-cat ?mps "-" ?side) ?sks))
    (not (pose-faker-info (robot ?r)
      (target-zone ?zone)
    ))
  =>
  (bind ?start-zone FALSE)
  ; Case 1: ?start-mps and ?start-side are normal mps + side
  (do-for-fact ((?d domain-fact))
    (and (eq ?d:name mps-poi) (member$ ?start-mps ?d:param-values)
      (member$ ?start-side ?d:param-values))
    (bind ?start-zone (nth$ 3 ?d:param-values))
  )
  ; Case 2: ?start-mps is a wait zone
  (if (str-index "WAIT-" ?start-mps) then
    (bind ?mps-side-str (sub-string 6 (length$ ?start-mps) ?start-mps))
    (bind ?actual-start-side OUTPUT)
    (bind ?actual-start-mps FALSE)
    (if (str-index INPUT ?mps-side-string) then
      (bind ?actual-start-side INPUT)
      (bind ?actual-start-mps (sym-cat
        (sub-string
          1
          (- (length$ ?mps-side-string 6))
          ?mps-side-str
        )
      ))
     else
      (bind ?actual-start-mps (sym-cat
        (sub-string
          1
          (- (length$ ?mps-side-string 7))
          ?mps-side-str
        )
      ))
    )
    (do-for-fact ((?d domain-fact))
      (and (eq ?d:name mps-poi) (member$ ?actual-start-mps ?d:param-values)
        (member$ ?acutal-start-side ?d:param-values))
      (bind ?start-zone (nth$ 3 ?d:param-values))
    )
  )
  ; Case 3: ?start-mps is a zone
  (do-for-fact ((?d domain-fact))
    (and (eq ?d:name zone-content) (eq (nth$ 1 ?d:param-values) ?start-mps))
    (bind ?start-zone ?start-mps)
  )
  (if (not ?start-zone) then
    (printout error "Cannot determine zone of current robot pose" crlf)
    else
    (assert (pose-faker-info (robot ?r)
      (current-zone ?start-zone)
      (target-zone ?zone)
    ))
  )
)

(defrule pose-faker-next-pose
  ?pfi <- (pose-faker-info
      (robot ?r)
      (target-zone ?target-zone)
      (current-zone ?current-zone)
  )
  =>
  (bind ?magenta-current-x (eq (sub-string 1 1 ?current-zone) "M"))
  (bind ?magenta-target-x (eq (sub-string 1 1 ?target-zone) "M"))
  (bind ?current-x (string-to-field (sub-string 4 4 ?current-zone)))
  (bind ?current-y (string-to-field (sub-string 5 5 ?current-zone)))
  (bind ?target-x (string-to-field (sub-string 4 4 ?target-zone)))
  (bind ?target-y (string-to-field (sub-string 5 5 ?target-zone)))
  (if ?magenta-current-x then (bind ?current-x (* -1 ?current-x)))
  (if ?magenta-target-x then (bind ?target-x (* -1 ?target-x)))
  (bind ?next-x ?current-x)
  (bind ?next-y ?current-y)
  (if (< ?current-x ?target-x) then (bind ?next-x (+ ?next-x 1)))
  (if (> ?current-x ?target-x) then (bind ?next-x (- ?next-x 1)))
  (if (< ?current-y ?target-y) then (bind ?next-y (+ ?next-y 1)))
  (if (> ?current-y ?target-y) then (bind ?next-y (- ?next-y 1)))
  ; (if (= ?next-x 0)
  ; (bind ?zone (sym-cat (str-cat
)
