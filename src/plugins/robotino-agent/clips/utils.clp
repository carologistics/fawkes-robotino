
;---------------------------------------------------------------------------
;  utils.clp - Robotino agent decision testing -- utility functions
;
;  Created: Sun Jun 17 12:19:34 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction append$ (?list $?items)
  (insert$ ?list (+ (length$ ?list) 1) ?items)
)

(deffunction enqueue-goto-target (?target_list ?name ?priority)
  (append$ ?target_list (implode$ (create$ ?name ?priority)))
)

(deffunction goto-target-prio (?m)
  (nth$ 2 (explode$ ?m))
)

(deffunction goto-target> (?a ?b)
  (> (goto-target-prio ?a) (goto-target-prio ?b))
)

(deffunction filter-goto-target ($?machines)
  (bind ?tm (sort goto-target> ?machines))
  (bind ?rv (create$))
  (bind ?prio 0)
  (foreach ?m ?tm
           (bind ?mprio (goto-target-prio ?m))
           (if (>= ?mprio ?prio) then
             (bind ?prio ?mprio)
             (append$ ?rv ?m)
           else
             (break)
           )
  )
)
