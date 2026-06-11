(define
  (problem kitchen-problem-0)
  (:domain kitchen-domain-0)
  (:objects
    world - qr::world-type
    floor - floor-type
    movo - movo-type
    table - table-type

    pillar - qrgeom::box-type
    potted-meat-can - potted-meat-can-type
  )
  (:init
    (weld world::world floor::base (0, 0, 0, 0, 0, 0))
    (body-pose movo (-1, 0.0, 0.071, 0.0, -0.0, 0.0))
    (weld world::world table (0, 0, 0, 0, 0, 0))
    (weld world::world pillar::box (0, 0.35, 0.7305, 0, 0, 0.7853982393431673))
    (body-pose potted-meat-can (0.03284035, 0.02635271, 0.735188, 0, 0, 0.7853982393431673))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.001))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))
  )
  (:goal (and
  ))
)