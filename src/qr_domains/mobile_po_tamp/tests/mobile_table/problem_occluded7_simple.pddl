(define
  (problem foo)
  (:domain foo) 
  (:objects
    spot
    table - table-type
    grape2 - big-grape-type
    shpam2 - qrgeom::box-type
    grape-class
  )
  (:init
    (robot spot)
    (use-right)    
    ;(use-base)

    ; table
    (body-pose table (0.75, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (5, 3, 2)))
    (support-surface table)

    ; grapes
    (body-pose grape2 (0.75, 0.0, 0.755, 0, 0, 0))
    (graspable grape2)
    (class grape2 grape-class)
    
    (qrgeom::box-shape shpam2 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam2 (0, 0, 0.8, 1.0))    
    (body-pose shpam2 (0.65, 0.0, 0.85, 0, 0, 0.0))
    (graspable shpam2)
  )
   (:goal (and (exists ?g (and (class ?g grape-class) (holding ?g))))
   )
)