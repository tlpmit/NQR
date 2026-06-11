(define
  (problem foo)
  (:domain foo) 
  (:objects
    world - qr::world-type
    floor - floor-type
    spot
    table - table-type
    grape2 - qrgeom::box-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    shpam3 - qrgeom::box-type
    grape-class
  )
  (:init
    (robot spot)
    (use-right)    
    (use-base)

    ; table
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (5, 3, 2)))
    (support-surface table)

    (weld world::world floor::base (0, 0, -0.025, 0, 0, 0))  ; avoid contact of floor with spot
    (weld world::world table (0.9, 0, 0, 0, 0, 0))
    
    (qrgeom::box-color table (0.82, 0.7, 0.55, 1.0))  
    (qrgeom::box-color floor (0.5, 0.5, 0.5, 1.0))  

    ; grapes
    (qrgeom::box-shape grape2 (0.05, 0.05, 0.05))
    (qrgeom::box-color grape2 (0, 1, 0, 1.0))  
    ;(body-pose grape2 (0.975, 0.0, 0.77, 0, 0, 0))
    (body-pose grape2 (1.0, 0.0, 0.77, 0, 0, 0))
    (graspable grape2)
    (qrgeom::box-mass grape2 0.020)  ; 20 grams
    (qrgeom::box-inertia grape2 (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model grape2 "compliant-hydroelastic")

    (qrgeom::box-shape shpam1 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    (body-pose shpam1 (0.9, -0.3, 0.85, 0, 0, 0.0))
    (graspable shpam1)    
    (qrgeom::box-mass shpam1 0.020)  ; 20 grams
    (qrgeom::box-inertia shpam1 (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model shpam1 "compliant-hydroelastic")
    
    (qrgeom::box-shape shpam2 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam2 (0, 0, 0.8, 1.0))    
    (body-pose shpam2 (0.9, 0.0, 0.85, 0, 0, 0.0))
    (graspable shpam2)
    (qrgeom::box-mass shpam2 0.020)  ; 20 grams
    (qrgeom::box-inertia shpam2 (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model shpam2 "compliant-hydroelastic")
    
    (qrgeom::box-shape shpam3 (0.05, 0.2, 0.2))
    (qrgeom::box-color shpam3 (0, 0, 0.8, 1.0))    
    (body-pose shpam3 (0.9, 0.3, 0.85, 0, 0, 0.0))
    (graspable shpam3)
    (qrgeom::box-mass shpam3 0.020)  ; 20 grams
    (qrgeom::box-inertia shpam3 (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model shpam3 "compliant-hydroelastic")
    
  )
   (:goal (and (exists ?g (and (class ?g grape-class) (holding ?g))))
   )
)