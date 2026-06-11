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
    (qrgeom::box-shape table (1.0, 2.0, 0.2))
    (body-pose table (1.2, 0.0, 0.4, 0.0, 0.0, 0.0))

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.05, 0.1, 0.05))
    (qrgeom::box-shape shpam2 (0.05, 0.1, 0.05))

    (qrgeom::box-color shpam1 (1, 0, 0, 1.0))    
    (qrgeom::box-color shpam2 (0, 1, 0, 1.0))
    
    (body-pose shpam1 (0.9, 0.0, 0.53, 0, 0, 0))
    (body-pose shpam2 (0.9,  0.15, 0.53, 0, 0, 0))

    (qrgeom::box-shape book (0.4, 0.4, 0.025))
    (body-pose book (1.0, -0.3, 0.5125, 0, 0, 0))
    (qrgeom::box-color book (1, 0, 1, 1.0))        

    (robot spot)
    (use-right)
    ;(use-base)
    
    (graspable shpam1)
    (graspable shpam2)
    (graspable book)    
    (support-surface table)
      )
  ; (:goal (and (on shpam1 book) (on shpam2 book)))
   ;(:goal (and (on shpam1 book)))
    (:goal (and (exists ?x (exists ?y (and (color ?x red) (on ?x book)
                                           (color ?y green) (on ?y book))))))
  ; (:goal (and (on shpam2 shpam1)))
)