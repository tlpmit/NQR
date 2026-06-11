(define
  (problem foo)
  (:domain foo) 
  (:objects
    spot
    world - qr::world-type
    table - table-type
    floor - floor-type
    grail - qrgeom::box-type
    ;shelf-top - qrgeom::box-type
    shelf-left - qrgeom::box-type
    shelf-right - qrgeom::box-type
  )
  (:init
    (robot spot)
    (use-right)    
    (use-base)

    ; table
    (workspace ((-2, -3, -2), (5, 3, 2)))

    (weld world::world floor::base (0, 0, -0.025, 0, 0, 0))  ; avoid contact of floor with spot
    (weld world::world table (1.5, 0, 0, 0, 0, 0))

    (qrgeom::box-color table (0.82, 0.7, 0.55, 1.0))  
    (qrgeom::box-color floor (0.5, 0.5, 0.5, 1.0))  

    ; grapes
    (graspable grail)
    (qrgeom::box-shape grail (0.05, 0.05, 0.15))
    (qrgeom::box-color grail (1, 0, 1, 1.0))    
    (body-pose grail (1.5, 0.0, 0.815, 0, 0, 0))
    (qrgeom::box-mass grail 0.020)  ; 20 grams
    (qrgeom::box-inertia grail (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model grail "compliant-hydroelastic")

    ; shelf pieces
    ;(qrgeom::box-shape shelf-top (0.45, 0.40, 0.025))
    ;(qrgeom::box-color shelf-top (0, 0.5, 0, 1.0))  
    ; height of 0.975 contacts supports
    ; body-pose shelf-top (0.8, 0.0, 1.0, 0, 0, 0))
    ;(weld world::world shelf-top (1.50, 0.0, 1.0, 0, 0, 0))

    (qrgeom::box-shape shelf-left (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-left (0, 0.5, 0, 1.0))       
    ;(body-pose shelf-left (0.8, -0.2, 0.85, 0, 0, 0))
    (weld world::world shelf-left (1.45, -0.2, 0.85, 0, 0, 0))

    (qrgeom::box-shape shelf-right (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-right (0, 0.5, 0, 1.0))       
    ;(body-pose shelf-right (0.8, 0.2, 0.85, 0, 0, 0))
    (weld world::world shelf-right (1.45, 0.2, 0.85, 0, 0, 0))

    (support-surface table)
    ;(support-surface shelf-top)
  )
   (:goal (and (holding grail))
   )
)