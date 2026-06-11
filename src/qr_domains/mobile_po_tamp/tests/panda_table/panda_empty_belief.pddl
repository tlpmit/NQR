(define
  (problem base-problem) 
  (:objects
    panda
    right
  )
  (:init
    (workspace ((-1.0, -1., 0.0), (1.0, 1., 2.0)))

    (robot panda)
    (use-right)
    
    (shadow-extents (1, 2, 0.5))
    (shadow-pose (0.25, -1, 0., 0, 0, 0.0))    

      )
  )
