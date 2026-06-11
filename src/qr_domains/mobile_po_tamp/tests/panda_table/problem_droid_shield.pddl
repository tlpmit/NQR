(define
  (problem base-problem) 
  (:objects
    base
    right
    table - table-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    ;shpam3 - qrgeom::box-type
    shield - qrgeom::box-type
    wicket1 - qrgeom::box-type
    wicket2 - qrgeom::box-type
    wicket3 - qrgeom::box-type

  )
  (:init
    (workspace ((-0.5, -0.5, 0.0), (1.0, 0.5, 2.0)))
    (body-pose table (0.5, 0.0, -0.75, 0, 0, 0))    

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (graspable shpam1)
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))
    (graspable shpam2)
    ;(qrgeom::box-shape shpam3 (0.1, 0.05, 0.05))
    ;(graspable shpam3)
    (qrgeom::box-shape shield (0.2, 0.025, 0.3))
    (graspable shield)
    
    (qrgeom::box-shape wicket1 (0.2, 0.025, 0.2))
    (qrgeom::box-shape wicket2 (0.2, 0.025, 0.2))
    (qrgeom::box-shape wicket3 (0.2, 0.2, 0.025))

    (qrgeom::box-color shpam2 (1, 0, 0, 1.0))    
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))
    ;(qrgeom::box-color shpam3 (0, 1, 0, 1.0))
    (qrgeom::box-color shield (0, 1, 1, 1.0))
    (qrgeom::box-color wicket1 (.2, .2, .2, 1.0))
    (qrgeom::box-color wicket2 (.2, .2, .2, 1.0))
    (qrgeom::box-color wicket3 (.2, .2, .2, 1.0))

    (body-pose shpam1 (0.6, 0.1, 0.02, 0, 0, 0))
    (body-pose shpam2 (0.6,  0.25, 0.02, 0, 0, 0))     
    ;(body-pose shpam3 (0.6,  0.25, 0.08, 0, 0, 1.5))     
    (body-pose shield (0.48,  0.25, 0.1525, 0, 0, 1.5))     
    (body-pose wicket1 (0.605, 0.35, 0.1025, 0, 0, 0))
    (body-pose wicket2 (0.605, 0.15, 0.1025, 0, 0, 0))
    (body-pose wicket3 (0.605, 0.25, 0.18, 0, 0, 0))


    (use-right)

    (support-surface table)
  )
)