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
    (use-base)
    

    (shadow-extents (4.6, 6.6, 1.6))
    (shadow-pose (0.6, -3.3, 0., 0, 0, 0.0))
    (chain-conf right (0, -2.3,  1.62,  0, 1.5, 0))

      )
  )
