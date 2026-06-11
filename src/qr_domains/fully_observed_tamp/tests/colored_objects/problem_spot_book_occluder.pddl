(define
  (problem base-problem) 
  (:objects
    spot
    base
    right
    table - table-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    book - qrgeom::box-type

  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))
    (body-pose table (1, 0, 0, 0, 0, 0))    

    ; start in a different conf from the default, more like stowed
    (chain-conf right (0.0, -3.0, 3.0, 0.0, 0.0, 0.0))
    ;(chain-conf right (0, -2.3,  1.62,  0, 1., 0))
        
    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))

    (qrgeom::box-color shpam1 (1, 0, 0, 1.0))    
    (qrgeom::box-color shpam2 (0, 1, 0, 1.0))
    
    (body-pose shpam1 (0.8,  0.05, 0.815, 0, 0, 1.5))
    (body-pose shpam2 (0.8,  0.05, 0.76, 0, 0, 0))

    (qrgeom::box-shape book (0.2, 0.2, 0.025))
    (body-pose book (0.8, -0.3, 0.75, 0, 0, 0))
    (qrgeom::box-color book (1, 0, 1, 1.0))        

    (robot spot)
    (use-right)
    ; (use-base)

    (graspable shpam1)
    (graspable shpam2)
    (graspable book)    
    (support-surface table)
    ; (support-surface book)    
  )
  (:goal (and (on shpam1 book) (on shpam2 book)))
  ; (:goal (and (on shpam1 book)))
  ; (:goal (and (on shpam2 shpam1)))
)