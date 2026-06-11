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
    shelf-bottom - qrgeom::box-type
    left-side - qrgeom::box-type
    right-side - qrgeom::box-type
    wall - qrgeom::box-type
    cabbage - qrgeom::box-type
    steak - qrgeom::box-type
    salt - qrgeom::box-type
    pepper - qrgeom::box-type
    extra1 - qrgeom::box-type
    extra2 - qrgeom::box-type
    extra3 - qrgeom::box-type
    extra4 - qrgeom::box-type
    extra5 - qrgeom::box-type
    thing1 - qrgeom::box-type
    thing2 - qrgeom::box-type
    thing3 - qrgeom::box-type
    thing4 - qrgeom::box-type
  )

  (:init
    (joint-conf base (-1.0, 0.0, 0.0))

    (qrgeom::box-shape stove (0.5, 0.5, 0.7))
    (qrgeom::box-color stove (1, 0.75, 0.75))
    (body-pose stove (.25, -1, .35, 0, 0, 0))

    (qrgeom::box-shape sink (0.5, 0.5, 0.7))
    (qrgeom::box-color sink (0.75, 0.75, 1.0))
    (body-pose sink (.25, 1, .35, 0, 0, 0))

    (qrgeom::box-shape table (0.5, 0.5, 0.7))
    (qrgeom::box-color table (65, 35, 15))
    (body-pose table (.75, 0, .35, 0, 0, 0))

    (qrgeom::box-shape wall (0.02, 5.0, 2.0))
    (qrgeom::box-color wall (0.5, 0.5, 0.5, 0.5))
    (body-pose wall (1, 0, 1, 0, 0, 0))

    (qrgeom::box-shape shelf (0.3, 0.5, 0.02))
    (qrgeom::box-color shelf (0.5, 0.8, 0.5, 0.65))
    (body-pose shelf (0.8, 0., 0.9, 0, 0, 0))

    (qrgeom::box-shape shelf-bottom (0.3, 0.5, 0.005))
    (qrgeom::box-color shelf-bottom (0.5, 0.8, 0.5, 0.65))
    (body-pose shelf-bottom (0.8, 0., 0.7025, 0, 0, 0))

    (qrgeom::box-shape left-side (0.3, 0.02, 0.3))
    (qrgeom::box-color left-side (0.5, 0.8, 0.5, 0.65))
    (body-pose left-side (0.8, 0.25, 0.8, 0, 0, 0))

    (qrgeom::box-shape right-side (0.3, 0.02, 0.3))
    (qrgeom::box-color right-side (0.5, 0.8, 0.5, 0.65))
    (body-pose right-side (0.8, -0.25, 0.8, 0, 0, 0))

    (qrgeom::box-shape steak (0.05, 0.05, 0.1))
    (qrgeom::box-color steak (.9, .1, .1))
    (body-pose steak (0.65, 0.0, 0.76, 0, 0, 0))

    (qrgeom::box-shape cabbage (0.05, 0.05, 0.1))
    (qrgeom::box-color cabbage (.1, .9, .1))
    (body-pose cabbage (0.8, 0.0, 0.76, 0, 0, 0))

    (qrgeom::box-shape salt (0.05, 0.05, 0.1))
    (qrgeom::box-color salt (.9, .9, .9))
    (body-pose salt (.8, 0.15, 0.76, 0, 0, 0))
    
    (qrgeom::box-shape pepper (0.05, 0.05, 0.1))
    (qrgeom::box-color pepper (.1, .1, .1))
    (body-pose pepper (.8, -0.15, 0.76, 0, 0, 0))

    (qrgeom::box-shape extra1 (0.05, 0.05, 0.1))
    (qrgeom::box-color extra1 (.2, .2, .9))
    (body-pose extra1 (.65, -0.15, 0.7575, 0, 0, 0))

    (qrgeom::box-shape extra2 (0.05, 0.05, 0.1))
    (qrgeom::box-color extra2 (.2, .2, .9))
    (body-pose extra2 (.65, 0.15, 0.7575, 0, 0, 0))

    (qrgeom::box-shape extra3 (0.05, 0.05, 0.1))
    (qrgeom::box-color extra3 (.2, .2, .9))
    (body-pose extra3 (.55, -0.15, 0.7575, 0, 0, 0))

    (qrgeom::box-shape extra4 (0.05, 0.05, 0.1))
    (qrgeom::box-color extra4 (.2, .2, .9))
    (body-pose extra4 (.55, 0.0, 0.7575, 0, 0, 0))

    (qrgeom::box-shape extra5 (0.05, 0.05, 0.1))
    (qrgeom::box-color extra5 (.2, .2, .9))
    (body-pose extra5 (.55, 0.15, 0.7575, 0, 0, 0))

    (qrgeom::box-shape thing1 (0.05, 0.05, 0.1))
    (qrgeom::box-color thing1 (.8, .6, .2))
    (body-pose thing1 (.2, -1.0, 0.7575, 0, 0, 0))

    (qrgeom::box-shape thing2 (0.05, 0.05, 0.1))
    (qrgeom::box-color thing2 (.8, .6, .2))
    (body-pose thing2 (.2, -1.1, 0.7575, 0, 0, 0))

    (qrgeom::box-shape thing3 (0.05, 0.05, 0.1))
    (qrgeom::box-color thing3 (.8, .6, .2))
    (body-pose thing3 (.3, -1.0, 0.7575, 0, 0, 0))

    (qrgeom::box-shape thing4 (0.05, 0.05, 0.1))
    (qrgeom::box-color thing4 (.8, .6, .2))
    (body-pose thing4 (.1, -1.0, 0.7575, 0, 0, 0))


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
    (graspable extra1)
    (graspable extra2)
    (graspable extra3)
    (graspable extra4)
    (graspable extra5)
    (graspable thing1)
    (graspable thing2)
    (graspable thing3)
    (graspable thing4)

    (support-surface table)
    (support-surface sink)
    (support-surface stove)
    (support-surface shelf)
    (support-surface shelf-bottom)

  )
  (:goal (and 
      ;(on cabbage stove)
      (on thing1 shelf-bottom)
      ;(on thing2 table)
      ;(on thing3 table)
      ;(on thing4 table)
  )
  )
)