; Notes
; - Handling robots
; - Handling subparts, including robot::gripper
; - Specifying confs of robot (chains)
; - object in the hand grasp convention

(define (problem grape_1)

  (:domain panda_grape_domain)

  (:objects
     ; Use PDDL type notation to associate instances with sdf types
      ; panda_1 - panda
      table - table-type
      grape_1 - grape-type
      small_cap_1 - small_cap-type
      small_cap_2 - small_cap-type      
      large_cap_1 - large_cap-type

      ; purely symbolic entities
      vessel
      fruit
      right
   )

  (:init
      ; (chain-conf panda_1::right (0, 0, 0, 0, 0, 0, 0))
      (body-pose table (0, 0, -0.005, 0, 0, 0))
      (body-pose grape_1 (0.3, -0.1, 0.01, 0, 0, 0))
      (body-pose small_cap_1 (0.3, -0.1, 0.03, 0, 3.14159, 0))
      (body-pose small_cap_2 (0.3, 0.1, 0.03, 0, 3.14159, 0))      
      (body-pose large_cap_1 (0.3, -0.1, 0.055, 0, 3.14159, 0))

      (controllable right)

      ; (holding grape_3 panda_1::gripper (0, 0, 0, 0, 0, 0) ) ; bogus numbers

      (surface table)
      (graspable grape_1)
      (graspable small_cap_1)
      (graspable small_cap_2)
      (graspable large_cap_1)

      (class grape_1 fruit)
      (class large_cap_1 vessel)
      (class small_cap_1 vessel)
      (class small_cap_2 vessel)      

  )

  (:goal
	(and (on grape_1 grape_2)))
)

