
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

(deffunction machine-prio (?machine)
  (if (eq ?machine DELIVER) then
    (return ?*GOTOPRIO_DELIVER*)
  else
    (if (eq ?machine M3) then
      (return ?*GOTOPRIO_M3*)
    else
      (if (eq ?machine M1_2) then
        (return ?*GOTOPRIO_M1_2*)
      else
        (if (eq ?machine M2_3) then
          (return ?*GOTOPRIO_M2_3*)
        else
          (if (eq ?machine M2) then
            (return ?*GOTOPRIO_M2*)
          else
            (if (eq ?machine M1) then
              (return ?*GOTOPRIO_M1*)
            else
              (return ?*GOTOPRIO_UNK*)
            )
          )
        )
      )
    )
  )
)

(deffunction merge-goto-machines (?cur-min-prio ?new-min-prio ?machines ?name)
  (if (< ?cur-min-prio ?new-min-prio) then
    (if (> (length$ ?machines) 0) then
      (printout t "#### OVERWRITING candidate machines ####" crlf)
    )
    (return (create$ ?name))
  else
    (return (append$ ?machines ?name))
  )
)
