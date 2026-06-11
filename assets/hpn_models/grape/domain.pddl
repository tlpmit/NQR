; Notes
; - We need a convention for handling robots

(define
  (domain panda_grape_domain)

  (:object-types
    (panda  "package://qr_assets/franka_description/urdf/panda_arm_hand.urdf")
    (table-type  "package://qr_assets/grape_quest/table.sdf")
    (grape-type  "package://qr_assets/grape_quest/grape.sdf")
    (small_cap-type  "package://qr_assets/grape_quest/small_cap.sdf")
    (large_cap-type  "package://qr_assets/grape_quest/large_cap.sdf")
  )

  (:predicates
    ; Useful for some planners but not required for world sim
    (controllable ?x)  ; chain ?x is controllable
    (surface ?x)       ; lpk---not sure type of ?x
    (graspable ?x)     ; ?x is a body that can be picked up
    (permanent ?x)     ; ?x is a body that can't be moved
    (wall ?x)          ; ?x is a body (could use class instead)
    (floor ?x)         ; ?x is a body (could use class instead)
    (stackable ?x ?y)  ; okay to put ?x on top of ?y
    (color ?x ?y)      ; color of object ?x is ?y
    (class ?x ?y)      ; object category (not the same as type above)
                       ; types are associated with specific meshes, etc
                       ; categories are more general (e.g. "bowl")

    ; Useful for specifying goals, but too vague for initial conds
    (on ?x ?y)         ; body ?x is resting on body ?y
  )
)