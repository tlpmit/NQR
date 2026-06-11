(define
  (problem empty)
  (:objects
      spot
      right
  )
  (:init
    (robot spot)
    (workspace ((-2, -2, 0), (2, 2, 2)))
    (use-right)
    ;(use-base)
    ; Get a good view of the table
    (chain-conf right (2.26974487e-04, -2.30147457e+00,  1.62356675e+00,  
                    7.04407692e-03, 1.55041754e+00, -4.28676605e-03))
  )
 (:goal
    (and (exists ?x (exists ?y (and (color ?x red)
                                    (color ?y green) (on ?x ?y)))))
 )
)
