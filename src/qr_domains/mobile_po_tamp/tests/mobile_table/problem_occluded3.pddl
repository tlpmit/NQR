(define
  (problem foo)
  (:domain foo) 
  (:objects
    movo
    table - table-type
    grape1 - big-grape-type
    grape2 - red-grape-type
    grape3 - green-grape-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    shpam3 - qrgeom::box-type
    ;small-cap - small-cap-type
    grape-class
    red-grape-class
    green-grape-class
    ;small-cap-class
    blue-mat - qrgeom::box-type
  )
  (:init
    (robot movo)
    (use-right)  
    ;(use-base)  

    ; table
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (4, 3, 2)))
    (support-surface table)

    ; mat
    (qrgeom::box-shape blue-mat (0.3, 0.3, 0.02))
    (qrgeom::box-color blue-mat (0, 0, 1, 1.0))
    (body-pose blue-mat::box (0.72, -0.5, 0.75, 0, 0, 0))
    
    ; cap
    ;(body-pose small-cap (0.42, 0.0, 0.03, 0, 3.14159, 0))

    ; grapes
    (body-pose grape1 (1.0, -0.3, 0.755, 0, 0, 0))
    (body-pose grape2 (1.0, 0.0, 0.755, 0, 0, 0))
    (body-pose grape3 (1.0, 0.1, 0.755, 0, 0, 0))
    (graspable grape1)
    (graspable grape2)
    (graspable grape3)
    (class grape1 grape-class)
    (class grape2 red-grape-class)
    (class grape3 green-grape-class)
    
    (qrgeom::box-shape shpam1 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam1 (0.2, 0.2, 0.2, 1.0))    
    (body-pose shpam1 (0.9, -0.3, 0.85, 0, 0, 0.0))
    (graspable shpam1)
    
    (qrgeom::box-shape shpam2 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam2 (0.2, 0.2, 0.2, 1.0))    
    (body-pose shpam2 (0.9, 0.0, 0.85, 0, 0, 0.0))
    (graspable shpam2)
    
    (qrgeom::box-shape shpam3 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam3 (0.2, 0.2, 0.2, 1.0))    
    (body-pose shpam3 (0.9, 0.3, 0.85, 0, 0, 0.0))
    (graspable shpam3)
    
  )
  (:goal (and (on grape1 blue-mat)))
)