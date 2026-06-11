(define
  (problem kitchen-problem-0) ; lpk: should this be problem0_wall or something?
  (:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
    world - qr::world-type
    floor - floor-type
    walls - wall-type
    movo - movo-type
    table - table-type

    pillar - qrgeom::box-type
    sugar-box - sugar-box-type
  )
  (:init
    (weld world::world floor::base (0, 0, 0, 0, 0, 0))
    (body-pose movo (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (weld world::world table (1, 0, 0, 0, 0, 0))
    ; (weld world::world pillar::box (1, 0.35, 0.7305, 0, 0, 0.7853982393431673))
    (weld world::world pillar::box (1, -0.5, 0.7305, 0, 0, 0.7853982393431673))
    (body-pose sugar-box (1.0, -0.1, 0.78, 0, 1.5714, -0.7853982393431673))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.02))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))

    (workspace ((-2, -2, 0), (2, 2, 2)))

    ; some helpful static facts
    (robot movo)
    (use-right)
    ;(use-base)
    (graspable sugar-box)
    (support-surface table)
    (support-surface pillar)

  )
  (:goal (and (on sugar-box pillar) ))
)