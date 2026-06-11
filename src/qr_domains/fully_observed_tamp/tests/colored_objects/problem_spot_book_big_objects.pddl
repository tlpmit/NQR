(define
  (problem base-problem) 
  (:objects
    spot
    base
    table - qrgeom::box-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    book - qrgeom::box-type
    green
    red
    magenta
  )
  (:init
    (workspace ((-2, -2, -2), (2, 2, 2)))

    ; table
    (qrgeom::box-shape table (1.0, 2.0, 0.05))
    (body-pose table (1.2, 0.0, 0.725, 0.0, 0.0, 0.0))

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.05, 0.1, 0.1))
    (qrgeom::box-shape shpam2 (0.05, 0.1, 0.1))

    (qrgeom::box-color shpam1 (1, 0, 0, 1.0))    
    (qrgeom::box-color shpam2 (0, 1, 0, 1.0))
    
    (body-pose shpam1 (0.8, 0.0, 0.8025, 0, 0, 0))
    (body-pose shpam2 (0.8,  0.15, 0.8025, 0, 0, 0))
    ;(body-pose shpam2 (0.8,  0.3, 0.8025, 0, 0, 0))

    (qrgeom::box-shape book (0.4, 0.4, 0.04))
    (body-pose book (0.9, -0.3, 0.7875, 0, 0, 0))
    (qrgeom::box-color book (1, 0, 1, 1.0))        

    (robot spot)
    (use-right)
    ;(use-base)
    
    (graspable shpam1)
    (graspable shpam2)
    (graspable book)    
    (support-surface table)
      )
      (:goal (and (exists ?x (exists ?y (and (color ?x red) (on ?x book)
                                           (color ?y green) (on ?y book))))))
  )