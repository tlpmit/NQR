(define
  (problem foo)
  (:domain foo) 
  (:objects
    movo
    table - table-type ;qrgeom::box-type
    grape - big-grape-type
    large-cap - large-cap-type
    ;small-cap - small-cap-type
    other-small-cap - small-cap-type
    grape-class
  )
  (:init
    (robot movo)
    (use-right)

    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))
    (support-surface table)
    ; table height is 0.72

    (body-pose grape (0.8, 0.0, 0.755, 0, 0, 0))
    (graspable grape)
    (class grape grape-class)

    (body-pose large-cap (0.8,  0.2,  0.775,  0,  3.14159,  0))
    
    ;(body-pose small-cap (0.8, 0.0, 0.75, 0, 3.14159, 0))

    (body-pose other-small-cap (0.8, 0.1, 0.75, 0, 3.14159, 0))
    (graspable other-small-cap)
    
  )
  (:goal (and (on grape large-cap))
  ; (:goal (and (on other-small-cap large-cap))
    )
)