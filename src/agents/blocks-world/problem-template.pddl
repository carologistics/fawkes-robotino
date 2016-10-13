; Standart problem description in blocks world
; Source: https://github.com/gerryai/PDDL4J/blob/master/pddl/blockworld

(define (problem blocks_world_from_robot_memory)
   (:domain blocksworld)
   (:objects
     <<#OBJECTS|{relation:'object'}>><<name>> <</OBJECTS>>
   )
   (:init <<#ONTABLE|{relation:'on-table'}>>(on-table <<object>>) <</ONTABLE>>
          <<#CLEAR|{relation:'clear'}>>(clear <<object>>) <</CLEAR>>
          <<#HOLDING|{relation:{$in:['holding','arm-empty']}}>>(<<relation>> <<object>>) <</HOLDING>>
   )
(:goal (and (on a b) (on b c) (on c d) (on d e))))