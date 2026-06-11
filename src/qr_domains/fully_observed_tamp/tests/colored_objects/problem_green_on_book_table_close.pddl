(define
  (problem base-problem) 
  (:objects
    base
    right
    table - table-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    book - qrgeom::box-type

  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))
    (body-pose table (0.5, 0, 0, 0, 0, 0))    ; x was 1.0

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))

    (qrgeom::box-color shpam1 (0, 1, 0, 1.0))    
    (qrgeom::box-color shpam2 (1, 0, 0, 1.0))
    
    (body-pose shpam1 (0.3, 0.0, 0.76, 0, 0, 0))
    (body-pose shpam2 (0.3,  0.15, 0.76, 0, 0, 0))

    (qrgeom::box-shape book (0.2, 0.2, 0.025))
    (body-pose book (0.3, -0.2, 0.75, 0, 0, 0))
    (qrgeom::box-color book (1, 0, 1, 1.0))        

    (use-right)

    (graspable shpam1)
    (graspable shpam2)
    (graspable book)    
    (support-surface table)
  )
  (:goal (and (on shpam1 book) (on shpam2 book)))
  ; (:goal (and (on shpam1 book)))
  ; (:goal (and (on shpam2 shpam1)))
)