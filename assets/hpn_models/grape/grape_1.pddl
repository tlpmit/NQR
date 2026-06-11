; Notes
; - Handling robots
; - Handling subparts, including robot::gripper
; - Specifying confs of robot (chains)
; - object in the hand grasp convention

(define (problem grape_1)

  (:domain panda_grape_domain)

  (:objects
     ; Use PDDL type notation to associate instances with sdf types
      world - qr::world-type

      panda_1 - panda
      table - table-type
      grape_1 - grape-type
      grape_2 - grape-type
      grape_3 - grape-type
      small_cap_1 - small_cap-type
      large_cap_1 - large_cap-type

      ; purely symbolic entities
      vessel
      fruit
      right
   )

  (:init
      (weld world::world panda_1::panda_link0 (0, 0, 0, 0, 0, 0))
      ; (chain-conf panda_1::right (0, 0, 0, 0, 0, 0, 0))
      (body-pose table (0, 0, -0.005, 0, 0, 0))
      (body-pose grape_1 (.1, .2, .02, 0, 0, 3.14))
      (body-pose grape_2 (.2, .3, .02, 0, 0, 3.14))
      (body-pose small_cap_1 (0.42, 0.0, 0.03, 0, 3.14159, 0))
      (body-pose large_cap_1 (0.42, 0.2, 0.055, 0, 3.14159, 0))

      (controllable right)

      ; (holding grape_3 panda_1::gripper (0, 0, 0, 0, 0, 0) ) ; bogus numbers

      (surface table)
      (graspable grape_1)
      (graspable grape_2)
      (graspable grape_3)
      (graspable small_cap_1)
      (graspable large_cap_1)

      (class grape_1 fruit)
      (class grape_2 fruit)
      (class grape_3 fruit)
      (class large_cap_1 vessel)
      (class small_cap_1 vessel)

  )

  (:goal
	(and (on grape_1 grape_2)))
)

