(define
  (problem base-problem) 
  (:objects
    rainbow
    right
  )
  (:init
    (workspace ((-0.75, -2, -2), (2, 2, 2)))    

    (robot rainbow)
    (use-right)
    ; (use-base)
    
    (shadow-extents (2, 4, 1.))
    (shadow-pose (0.4, -2, 0., 0, 0, 0.0))    

      )
  )
