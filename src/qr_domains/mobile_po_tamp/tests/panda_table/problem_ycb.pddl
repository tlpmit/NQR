(define
  (problem foo)
  (:domain foo) 
  (:objects
    panda
    banana - qrgeom::box-type
    table - qrgeom::box-type
    ; banana - banana-type
    ; bowl - bowl-type
  )
  (:init
    (workspace ((-2.0, -3., 0.0), (1.0, 3., 2.0)))

    (robot panda)

    ; table
    (qrgeom::box-shape table (2, 2, 0.001))
    (qrgeom::box-color table (0.8, 0.8, 0.8))
    (body-pose table (0, 0, -0.002, 0, 0, 0))
    (support-surface table)

    ; YCB
    ;(body-pose banana (0.52,  0.2,  0.2,  0,  3.14159,  0))
    ;(qrgeom::box-color banana (1.0, 1.0, 0.0))
    ;(graspable banana)

    (qrgeom::box-shape banana (0.1, 0.05, 0.05))
    (qrgeom::box-color banana (1.0, 1.0, 0.0))
    (body-pose banana (0.52,  0.2,  0.1,  0,  0,  0))
    (graspable banana)

    ;(body-pose bowl (0.42, -0.2,  0.025,  0,  0,  0))
    ;(qrgeom::box-color bowl (0.65, 0.15, 0.15))
    ;(graspable bowl)
    
  )
)