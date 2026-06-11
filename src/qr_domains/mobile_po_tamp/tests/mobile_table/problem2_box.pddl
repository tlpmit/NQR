(define
  (problem foo)
  (:domain foo) 
  (:objects
    movo
    table - table-type
    grape - big-grape-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
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

    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))
    (qrgeom::box-color shpam2 (0, 1, 0, 1.0))    
    (body-pose shpam1 (0.8,  0.2,  0.775,  0,  0,  0))
    (body-pose shpam2 (0.8, 0.1, 0.775, 0, 0, 0))
    (graspable shpam2)
    
  )
  (:goal (and (on grape shpam1))
  )
)