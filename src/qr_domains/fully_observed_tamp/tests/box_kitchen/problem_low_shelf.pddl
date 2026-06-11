(define
  (problem foo)
  (:domain foo) 
  (:objects
    movo
    base
    stove - qrgeom::box-type
    sink - qrgeom::box-type
    table - qrgeom::box-type
    shelf - qrgeom::box-type
    wall - qrgeom::box-type
    cabbage - qrgeom::box-type
    steak - qrgeom::box-type
    salt - qrgeom::box-type
    pepper - qrgeom::box-type
  )

  (:init
    (joint-conf base (-1.0, 0.0, 0.0))

    (qrgeom::box-shape stove (0.5, 0.5, 0.7))
    (qrgeom::box-color stove (1, 0.75, 0.75))
    (body-pose stove (-0.75, -1, .35, 0, 0, 0))

    (qrgeom::box-shape sink (0.5, 0.5, 0.7))
    (qrgeom::box-color sink (0.75, 0.75, 1.0))
    (body-pose sink (-0.75, 1, .35, 0, 0, 0))

    (qrgeom::box-shape table (0.5, 0.5, 0.7))
    (qrgeom::box-color table (65, 35, 15))
    (body-pose table (.75, 0, .35, 0, 0, 0))

    (qrgeom::box-shape wall (0.02, 5.0, 2.0))
    (qrgeom::box-color wall (0.5, 0.5, 0.5, 0.5))
    (body-pose wall (1, 0, 1, 0, 0, 0))

    (qrgeom::box-shape shelf (0.3, 0.5, 0.02))
    (qrgeom::box-color shelf (0.5, 0.8, 0.5, 0.65))
    (body-pose shelf (0.8, 0., 0.86, 0, 0, 0))

    (qrgeom::box-shape steak (0.05, 0.05, 0.1))
    (qrgeom::box-color steak (.9, .1, .1))
    (body-pose steak (0.65, 0.0, 0.755, 0, 0, 0))

    (qrgeom::box-shape cabbage (0.05, 0.05, 0.1))
    (qrgeom::box-color cabbage (.1, .9, .1))
    (body-pose cabbage (0.8, 0.0, 0.755, 0, 0, 0))

    (qrgeom::box-shape salt (0.05, 0.05, 0.1))
    (qrgeom::box-color salt (.9, .9, .9))
    (body-pose salt (.8, 0.15, 0.755, 0, 0, 0))
    
    (qrgeom::box-shape pepper (0.05, 0.05, 0.1))
    (qrgeom::box-color pepper (.1, .1, .1))
    (body-pose pepper (.8, -0.15, 0.755, 0, 0, 0))
    
    (workspace ((-2, -2, 0), (2, 2, 3)))    ; corners

    ; some helpful static facts
    (robot movo)
    (use-right)
    ;(use-left)
    (use-base)

    (graspable cabbage)
    (graspable steak)
    (graspable salt)
    (graspable pepper)

    (support-surface table)
    (support-surface sink)
    (support-surface stove)
    (support-surface shelf)

  )
  (:goal (and 
      ;(on steak cabbage)
      ;(on cabbage stove)
      (on salt stove)
  )
  )
)