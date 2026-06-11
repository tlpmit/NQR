(define
  (problem foo)
  (:domain foo) 
  (:objects
    spot
    world - qr::world-type
    table1 - table-type
    table2 - table-type
    floor - floor-type
    grail - qrgeom::box-type
    ;shelf-top - qrgeom::box-type
    shelf-left - qrgeom::box-type
    shelf-right - qrgeom::box-type
    mat - qrgeom::box-type
  )
  (:init
    (robot spot)
    (use-right)    
    (use-base)

    (workspace ((-2, -3, -2), (5, 3, 2)))

    (weld world::world floor::base (0, 0, -0.025, 0, 0, 0))  ; avoid contact of floor with spot
    (weld world::world table1 (1.5, 1, 0, 0, 0, 0))
    (weld world::world table2 (1.5, -1, 0, 0, 0, 0))

    (qrgeom::box-color table1 (0.82, 0.7, 0.55, 1.0))  
    (qrgeom::box-color table2 (0.82, 0.7, 0.55, 1.0))  
    (qrgeom::box-color floor (0.5, 0.5, 0.5, 1.0))  
    
    (graspable grail)
    (qrgeom::box-shape grail (0.05, 0.05, 0.15))
    (qrgeom::box-color grail (1, 0, 1, 1.0))    
    (body-pose grail (1.5, 1.0, 0.815, 0, 0, 0))
    (qrgeom::box-mass grail 0.020)  ; 20 grams
    (qrgeom::box-inertia grail (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model grail "compliant-hydroelastic")

    ; mat
    (qrgeom::box-shape mat (0.2, 0.2, 0.02))
    (qrgeom::box-color mat (0, 0, 0.9, 1.0))
    (weld world::world mat (1.5, -1, .76, 0, 0, 0))

    ; shelf pieces
    (qrgeom::box-shape shelf-left (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-left (0, 0.5, 0, 1.0))       
    (weld world::world shelf-left (1.45, 0.8, 0.85, 0, 0, 0))

    (qrgeom::box-shape shelf-right (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-right (0, 0.5, 0, 1.0))       
    (weld world::world shelf-right (1.45, 1.2, 0.85, 0, 0, 0))

    (support-surface table1)
    (support-surface table2)
    
  )
   (:goal (and (holding grail))
   )
)