(define
  (problem base-problem) 
  (:objects
    spot
    right
  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))    

    (robot spot)
    (use-right)
    ;(use-base)

    (shadow-extents (1, 2, 1.5))
    (shadow-pose (0.7, -1, 0., 0, 0, 0.0))
    (chain-conf right (0, -2.3,  1.62,  0, 1.5, 0))

      )
  )
