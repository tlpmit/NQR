(define
  (problem base-problem) 
  (:objects
    spot
    right
    world - qr::world-type
  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))    

    (robot spot)
    (use-right)
    (use-base)
    


    (shadow-extents (2, 4, 1.6))
    (shadow-pose (0.625, -2, 0., 0, 0, 0.0))
    (chain-conf right (0, -2.3,  1.62,  0, 1.5, 0))

      )
  )
