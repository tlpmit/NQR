(define
  (problem foo)
  (:domain foo) 
  (:objects
    panda
    table - qrgeom::box-type
    grape - grape-type
    ;large-cap - large-cap-type
    ;small-cap - small-cap-type
    other-small-cap - small-cap-type
  )
  (:init
    (workspace ((-2.0, -3., 0.0), (1.0, 3., 2.0)))

    (robot panda)
    ;(body-pose panda (0, 0.0, 0.071, 0.0, -0.0, 0.0))

    (qrgeom::box-shape table (2, 2, 0.001))
    (qrgeom::box-color table (0.8, 0.8, 0.8))
    (body-pose table (0, 0, -0.005, 0, 0, 0))
    (support-surface table)

    (body-pose grape (0.2, -0.3, 0.01, 0, 0, 0))
    (graspable grape)

    ;(body-pose large-cap (0.42,  0.2,  0.055,  0 3.14159,  0))
    ;(body-pose small-cap (0.42, 0.0, 0.03, 0, 3.14159, 0))

    (body-pose other-small-cap (0.42, 0.1, 0.03, 0, 3.14159, 0))
    (graspable other-small-cap)
    
  )
  (:goal (and (on grape other-small-cap))
    )
)