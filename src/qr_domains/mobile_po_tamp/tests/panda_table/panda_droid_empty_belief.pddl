(define
  (problem base-problem) 
  (:objects
    panda
    right
    floor - qrgeom::box-type
    camera - qrgeom::box-type
  )
  (:init
    (workspace ((-0.5, -0.5, 0.0), (0.75, 0.5, 2.0)))

    (robot panda)
    (use-right)
    
    (shadow-extents (1, 1, 0.25))
    (shadow-pose (0.2, -0.5, 0., 0, 0, 0.0))    

    (qrgeom::box-shape floor (0.86, 1.24, 0.725))
    (qrgeom::box-color floor (0.3, 0.5, 0.3))
    (body-pose floor (-0.24, 0.525, -0.375, 0, 0, 0))
    (support-surface floor)

    (qrgeom::box-shape camera (0.36, 0.2, 0.76))
    (qrgeom::box-color camera (0.3, 0.5, 0.3))
    (body-pose camera (0.0, 0.525, 0.38, 0, 0, 0))

      )
  )
