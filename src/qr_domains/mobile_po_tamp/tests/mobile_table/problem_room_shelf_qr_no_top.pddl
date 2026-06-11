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
    wall-right - qrgeom::box-type
    wall-left - qrgeom::box-type
    wall-front - qrgeom::box-type
    ;wall-back - qrgeom::box-type    
  )
  (:init
    (robot spot)
    (use-right)    
    (use-base)

    ; table
    (workspace ((-1, -3, -2), (5, 3, 2)))

    (weld world::world floor::base (0, 0, -0.025, 0, 0, 0))  ; avoid contact of floor with spot
    (weld world::world table (3.9, 2.5, 0, 0, 0, 1.57))

    (qrgeom::box-color table (0.82, 0.7, 0.55, 1.0))  
    (qrgeom::box-color floor (0.5, 0.5, 0.5, 1.0))

    (qrgeom::box-shape wall-left (4.5, 0.1, 2))
    (qrgeom::box-color wall-left (0.5, 0.5, 0.5, 1.0))    
    (weld world::world wall-left (2.75, -3, 1., 0, 0, 0.0))

    (qrgeom::box-shape wall-right (4.5, 0.1, 2))
    (qrgeom::box-color wall-right (0.5, 0.5, 0.5, 1.0)) 
    (weld world::world wall-right (2.75, 3, 1., 0, 0, 0.0))

    ;(qrgeom::box-shape wall-back (0.1, 6.0, 2))
    ;(qrgeom::box-color wall-back (0.5, 0.5, 0.5, 1.0))    
    ;(weld world::world wall-back (-1, 0, 1., 0, 0, 0.0))

    (qrgeom::box-shape wall-front (0.1, 6, 2))
    (qrgeom::box-color wall-front (0.5, 0.5, 0.5, 1.0)) 
    (weld world::world wall-front (5, 0, 1., 0, 0, 0.0))

    ; grail
    (graspable grail)
    (qrgeom::box-shape grail (0.075, 0.075, 0.15))
    (qrgeom::box-color grail (1, 0.0, 1, 1.0))    
    (qrgeom::box-mass grail 0.020)  ; 20 grams
    (qrgeom::box-inertia grail (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model grail "compliant-hydroelastic")
    (body-pose grail (3.5, 2.5, 0.815, 0, 0, 0))

    (qrgeom::box-shape shelf-left (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-left (0, 0.5, 0, 1.0))       
    (weld world::world shelf-left (3.55, 2.3, 0.85, 0, 0, 0))

    (qrgeom::box-shape shelf-right (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-right (0, 0.5, 0, 1.0))       
    (weld world::world shelf-right (3.55, 2.7, 0.85, 0, 0, 0))

    (support-surface table)
    ;(support-surface shelf-top)
  )
   (:goal (and (holding grail))
   )
)