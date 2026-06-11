(define
  (problem foo)
  (:domain foo) 
  (:objects
    spot
    table - table-type
    grail - qrgeom::box-type
    shelf-top - qrgeom::box-type
    shelf-left - qrgeom::box-type
    shelf-right - qrgeom::box-type
  )
  (:init
    (robot spot)
    (use-right)    
    ;(use-base)

    ; table
    (body-pose table (0.85, 0, 0, 0, 0, 0))
    (workspace ((-2, -3, -2), (5, 3, 2)))
    (support-surface table)

    ; grapes
    (graspable grail)
    (qrgeom::box-shape grail (0.075, 0.075, 0.15))
    (qrgeom::box-color grail (1, 0, 1, 1.0))    
    (body-pose grail (0.9, 0.0, 0.815, 0, 0, 0))

    ; shelf pieces
    (qrgeom::box-shape shelf-top (0.45, 0.40, 0.025))
    (qrgeom::box-color shelf-top (0, 0.5, 0, 1.0))
    ; height of 0.975 contacts supports
    (body-pose shelf-top (0.8, 0.0, 1.0, 0, 0, 0))

    (qrgeom::box-shape shelf-left (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-left (0, 0.5, 0, 1.0))    
    (body-pose shelf-left (0.8, -0.2, 0.85, 0, 0, 0))

    (qrgeom::box-shape shelf-right (0.45, 0.025, 0.25))
    (qrgeom::box-color shelf-right (0, 0.5, 0, 1.0))    
    (body-pose shelf-right (0.8, 0.2, 0.85, 0, 0, 0))

  )
   (:goal (and (exists ?g (and (class ?g grape-class) (holding ?g))))
   )
)