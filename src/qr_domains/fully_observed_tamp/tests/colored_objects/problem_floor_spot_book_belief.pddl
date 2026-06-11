(define
  (problem base-problem) 
  (:objects
    spot
    green
    red
    magenta
  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))    

    (robot spot)
    (use-right)
    ;(use-base)
    
      )
    (:goal (and (exists ?x (exists ?y (and (color ?x magenta) 
                                           (color ?y green) (on ?y ?x))))))
  )