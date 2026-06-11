(define
  (problem base-problem) 
  (:objects
    pr2
    table - table-type
    a - qrgeom::box-type
    b - qrgeom::box-type
    c - qrgeom::box-type
  )
  (:init
    ; robot and table
    (body-pose pr2 (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))

    ; manipulanda
    (qrgeom::box-shape a (0.05, 0.05, 0.05))
    (qrgeom::box-shape b (0.05, 0.05, 0.05))
    (qrgeom::box-shape c (0.05, 0.05, 0.05))
    (qrgeom::box-color a (1, 0, 0, 1.0))
    (qrgeom::box-color b (0, 1, 0, 1.0))
    (qrgeom::box-color c (0, 0, 1, 1.0))
    
    (body-pose b (0.8, -0.1, 0.76, 0, 0, 0))
    ; (body-pose b (0.8,  0.1, 0.88, 0, 0, 0))
    (body-pose a (0.8,  0.1, 0.76, 0, 0, 0))
    ; c on a
    (body-pose c (0.8, 0.1, 0.82, 0, 0, 0))

    ; additional declarations
    (robot pr2)
    (use-right)
    (use-base)
    (graspable a)
    (graspable b)
    (graspable c)
    (support-surface table)
  )
  (:goal (and (on a b) (on b c)
  ;(:goal (and (on a c)
  ))
)