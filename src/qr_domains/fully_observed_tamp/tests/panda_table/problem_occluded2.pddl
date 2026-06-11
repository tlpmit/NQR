(define
  (problem foo)
  (:domain foo) 
  (:objects
    panda
    table - qrgeom::box-type
    grape1 - big-grape-type
    grape2 - big-grape-type
    grape3 - big-grape-type
    ;large-cap - large-cap-type
    ;small-cap - small-cap-type
    ;other-small-cap - small-cap-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    shpam3 - qrgeom::box-type
    grape-class
  )
  (:init
    (workspace ((-2.0, -3., 0.0), (1.0, 3., 2.0)))

    (robot panda)

    ; table
    (qrgeom::box-shape table (2, 2, 0.001))
    (qrgeom::box-color table (0.8, 0.8, 0.8))
    (body-pose table (0, 0, -0.005, 0, 0, 0))
    (support-surface table)

    ; grapes
    (body-pose grape1 (0.5, -0.3, 0.025, 0, 0, 0))
    (body-pose grape2 (0.5, 0.0, 0.025, 0, 0, 0))
    (body-pose grape3 (0.5, 0.1, 0.025, 0, 0, 0))
    (graspable grape1)
    (graspable grape2)
    (graspable grape3)
    (class grape1 grape-class)
    (class grape2 grape-class)
    (class grape3 grape-class)

    ; caps
    ;(body-pose large-cap (0.42,  0.2,  0.055,  0,  3.14159,  0))
    ;(body-pose small-cap (0.42, 0.0, 0.03, 0, 3.14159, 0))
    ;(body-pose other-small-cap (0.42, 0.1, 0.03, 0, 3.14159, 0))
    ;(graspable other-small-cap)

    (qrgeom::box-shape shpam1 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    (body-pose shpam1 (0.4, -0.3, 0.1, 0, 0, 0.0))
    (graspable shpam1)
    (qrgeom::box-shape shpam2 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam2 (0, 0, 1, 1.0))    
    (body-pose shpam2 (0.4, 0.0, 0.1, 0, 0, 0.0))
    (graspable shpam2)
    (qrgeom::box-shape shpam3 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam3 (0, 0, 1, 1.0))    
    (body-pose shpam3 (0.4, 0.3, 0.1, 0, 0, 0.0))
    (graspable shpam3)
    
  )
)