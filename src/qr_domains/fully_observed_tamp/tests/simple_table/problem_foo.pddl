(define
  (problem kitchen-problem-0) ; lpk: should this be problem0_wall or something?
  ;(:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
    movo
    table - table-type

    pillar - qrgeom::box-type
    sugar-box - qrgeom::box-type
  )
  (:init
    (body-pose movo (0, 0.0, 0.0, 0.0, -0.0, 0.0))
    (body-pose table (0.75, 0, 0, 0, 0, 0))
    (body-pose pillar::box (0.75, -0.45, 0.7305, 0, 0, 0.0))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.02))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))

    (qrgeom::box-shape sugar-box (0.170300, 0.039100, 0.086700))
    (qrgeom::box-color sugar-box (1, 1, 0, 1.0))

    (body-pose sugar-box (0.75, -0.2, 0.78, 0, 0, -0.7853982393431673))


    (workspace ((-2, -2, 0), (2, 2, 2)))

    ; some helpful static facts
    (robot movo)
    (use-right)
    (use-base)
    (graspable sugar-box)
    (support-surface table)
    (support-surface pillar)

  )
  (:goal (and (on sugar-box pillar) ))
)