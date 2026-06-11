
(define
  (problem foo)
  (:domain foo) 
  (:objects
    table - table-type
    grape-class
    red-grape-class
    green-grape-class
    blue-mat - qrgeom::box-type
    movo
    red
    blue
    green
  )
  (:init
    (robot movo)
    (use-right)    
    ;(use-base)

    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (4, 3, 2)))
    (support-surface table)

    ; mat
    (qrgeom::box-shape blue-mat (0.3, 0.3, 0.02))
    (qrgeom::box-color blue-mat (0, 0, 1, 1.0))
    (body-pose blue-mat::box (0.72, -0.5, 0.75, 0, 0, 0))
    (support-surface blue-mat)
    
  )
  (:goal (and (exists ?g1 (exists ?g2 (and ;(color ?g1 red) 
                                           ;(color ?g2 green)
                                           (class ?g1 red-grape-class)
                                           (class ?g2 green-grape-class)
                                           (on ?g1 blue-mat)
                                           (on ?g2 blue-mat)                                           
                                         ))))
    )
)