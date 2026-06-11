(define
  (problem base-problem) 
  (:objects
    pr2
    table - table-type
    cubby-top - qrgeom::box-type
    cubby-bottom - qrgeom::box-type
    cubby-left - qrgeom::box-type
    cubby-middle - qrgeom::box-type
    cubby-right - qrgeom::box-type
    front-left - qrgeom::box-type
    front-right - qrgeom::box-type
    a - qrgeom::box-type
  )
  (:init
    ; robot and table
    (body-pose pr2 (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))

    ; cubby holes
    (qrgeom::box-shape cubby-left (0.2, 0.02, 0.3))
    (qrgeom::box-shape cubby-right (0.35, 0.02, 0.3))
    (qrgeom::box-shape cubby-middle (0.35, 0.02, 0.3))
    (qrgeom::box-shape cubby-top (0.3, 0.1, 0.02))
    (qrgeom::box-shape cubby-bottom (0.3, 0.1, 0.02))

    (body-pose cubby-left (0.9, -0.5, 0.9, 0, 0, 0))
    (body-pose cubby-middle (0.9, 0.0, 0.9, 0, 0, 0))
    (body-pose cubby-right (0.9, 0.1, 0.9, 0, 0, 0))
    (body-pose cubby-bottom (0.9, 0.05, 0.76, 0, 0, 0))
    (body-pose cubby-top (0.9, 0.05, 1.1, 0, 0, 0))

    ; manipulanda
    (qrgeom::box-shape a (0.05, 0.35, 0.15))
    (qrgeom::box-color a (1, 0, 0, 1.0))
    (body-pose a (0.9, -0.25, 0.87, 0, 0, 0))

    ; obstacles
    (qrgeom::box-shape front-left (0.05, 0.1, 0.15))
    (body-pose front-left (0.825, -0.4, 0.87, 0, 0, 0))

    (qrgeom::box-shape front-right (0.05, 0.1, 0.15))
    (body-pose front-right (0.825, -0.15, 0.87, 0, 0, 0))
 
    ; additional declarations
    (robot pr2)
    (use-right)
    (use-base)
    (graspable a)
    (support-surface table)
    (support-surface cubby-bottom)
  )
  (:goal (and (on a cubby-bottom)
  ))
)