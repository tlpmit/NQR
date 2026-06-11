(define
  (problem base-problem) 
  (:objects
    base
    right
    table - table-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type

  )
  (:init
    (workspace ((-0.5, -0.5, 0.0), (1.0, 0.5, 2.0)))
    (body-pose table (0.5, 0.0, -0.75, 0, 0, 0))    

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))

    (qrgeom::box-color shpam1 (1, 0, 0, 1.0))    
    (qrgeom::box-color shpam2 (0, 0, 1, 1.0))
    
    (body-pose shpam1 (0.5, 0.1, 0.02, 0, 0, 0))
    (body-pose shpam2 (0.5,  0.25, 0.02, 0, 0, 0))     

    (use-right)

    (graspable shpam1)
    (graspable shpam2)
    (support-surface table)
  )
)