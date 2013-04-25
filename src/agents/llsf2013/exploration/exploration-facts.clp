(deftemplate machine-exploration
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
  (slot next (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
)

(deftemplate machine-light
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot red (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot yellow (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot green (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
)

(deftemplate machine-type
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE) (default TEST))
)

(deftemplate matching-type-light
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE) (default TEST))
  (slot red (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot yellow (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot green (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
)

(deftemplate signal
  (slot name (type STRING))
  (multislot time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot seq (type INTEGER) (default 1))
)

;machine name, coordinates, next machine in exploration cycle
(assert 
  (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9))
  (machine-exploration (name M9) (x 1.38) (y 3.42) (next M5))
  (machine-exploration (name M8) (x 1.38) (y 2.18) (next M3))
  (machine-exploration (name M7) (x 2.5) (y 4.5) (next M10))
  (machine-exploration (name M6) (x 3.1) (y 4.42) (next M7))
  (machine-exploration (name M5) (x 2.3) (y 3.1) (next M4))
  (machine-exploration (name M4) (x 3.1) (y 2.13) (next M8))
  (machine-exploration (name M3) (x 3.1) (y 1.06) (next M1))
  (machine-exploration (name M2) (x 4.42) (y 3.62) (next M6))
  (machine-exploration (name M1) (x 3.62) (y 1.18) (next M2))
)
