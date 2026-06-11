(define
  (problem foo)
  (:domain foo) 
  (:objects
    spot
    table - table-type
    ;grape2 - big-grape-type
    grape2 - qrgeom::box-type
    ;shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    ;shpam3 - qrgeom::box-type
    grape-class
  )
  (:init
    (robot spot)
    (use-right)    
    (use-base)

    ; table
    (body-pose table (0.85, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (5, 3, 2)))
    (support-surface table)

    ; grapes
    (graspable grape2)
    (qrgeom::box-shape grape2 (0.05, 0.05, 0.05))
    (qrgeom::box-color grape2 (1, 0, 1, 1.0))    
    (body-pose grape2 (0.85, 0.0, 0.765, 0, 0, 0))

    ;(qrgeom::box-shape shpam1 (0.05, 0.2, 0.2))
    ;(qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    ;(body-pose shpam1 (0.75, -0.3, 0.85, 0, 0, 0.0))
    ;(graspable shpam1)
    
    (qrgeom::box-shape shpam2 (0.075, 0.2, 0.2))
    (qrgeom::box-color shpam2 (0, 0, 0.8, 1.0))    
    (body-pose shpam2 (0.75, 0.0, 0.85, 0, 0, 0.0))
    (graspable shpam2)
    
    ;(qrgeom::box-shape shpam3 (0.05, 0.2, 0.2))
    ;(qrgeom::box-color shpam3 (0, 0, 0.8, 1.0))    
    ;(body-pose shpam3 (0.75, 0.3, 0.85, 0, 0, 0.0))
    ;(graspable shpam3)
    
  )
   (:goal (and (exists ?g (and (class ?g grape-class) (holding ?g))))
   )
)