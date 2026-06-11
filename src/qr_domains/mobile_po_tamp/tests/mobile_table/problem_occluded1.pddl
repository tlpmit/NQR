(define
  (problem foo)
  (:domain foo) 
  (:objects
    panda
    table - qrgeom::box-type
    grape - big-grape-type
    ;large-cap - large-cap-type
    ;small-cap - small-cap-type
    ;other-small-cap - small-cap-type
    shpam1 - qrgeom::box-type
  )
  (:init
    (workspace ((-2, -3, -2), (4, 3, 2)))

    (robot panda)
    (use-right)

    ; table
    (qrgeom::box-shape table (2, 2, 0.001))
    (qrgeom::box-color table (0.8, 0.8, 0.8))
    (body-pose table (0, 0, -0.005, 0, 0, 0))
    (support-surface table)

    ; grape
    (body-pose grape (0.5, -0.3, 0.025, 0, 0, 0))
    (graspable grape)

    ; caps
    ;(body-pose large-cap (0.42,  0.2,  0.055,  0,  3.14159,  0))
    ;(body-pose small-cap (0.42, 0.0, 0.03, 0, 3.14159, 0))
    ;(body-pose other-small-cap (0.42, 0.1, 0.03, 0, 3.14159, 0))
    ;(graspable other-small-cap)

    (qrgeom::box-shape shpam1 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    (body-pose shpam1 (0.4, -0.3, 0.1, 0, 0, 0.0))
    (graspable shpam1)
    
  )
)