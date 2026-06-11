(define
  (problem base-problem) 
  (:objects
    spot
    base
    ;table - qrgeom::box-type
    table - table-type
    green-mat - qrgeom::box-type
    red-mat - qrgeom::box-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    shpam3 - qrgeom::box-type
    blue
    green
    red
  )
  (:init
    ; move spot back from the table a bit
    ; (chain-conf base (-0.2, 0, 0))
    ; (chain-conf base (0.340, 0.220, -0.628))

    (workspace ((-2, -2, -2), (2, 2, 2)))

    ;(qrgeom::box-shape table (0.3, 0.5, 0.55))
    ;(qrgeom::box-color table (1, 0.75, 0.75))
    ;(body-pose table (1.0, 0.0, .275, 0, 0, 0))
    (body-pose table (1.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    ; colored mats on table
    (qrgeom::box-shape green-mat (0.3, 0.3, 0.02))
    (qrgeom::box-shape red-mat (0.3, 0.3, 0.02))
    (qrgeom::box-color green-mat (0, 1, 0, 1.0))
    (qrgeom::box-color red-mat (1, 0, 0, 1.0))
    (body-pose green-mat::box (1.1, -0.3, 0.73, 0, 0, 0))
    (body-pose red-mat::box (1.1, 0.0, 0.73, 0, 0, 0))

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam3 (0.1, 0.05, 0.05))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    (qrgeom::box-color shpam2 (0, 0, 1, 1.0))    
    (qrgeom::box-color shpam3 (0, 0, 1, 1.0))    
    (body-pose shpam1 (0.8, 0.0, 0.78, 0, 0, 0))
    (body-pose shpam2 (0.8,  0.25, 0.78, 0, 0, 0))
    (body-pose shpam3 (0.8, -0.25, 0.78, 0, 0, 0))

    (use-base)

    (robot spot)
    (use-right)
    ;(use-base)
    (graspable shpam1)
    (graspable shpam2)
    (graspable shpam3)
    (support-surface table)
    (support-surface red-mat)
    (support-surface green-mat)
  )
  ; (:goal (and (color ?x blue) (color ?y red) (on ?x ?y)
  (:goal (and (holding shpam2)))
)