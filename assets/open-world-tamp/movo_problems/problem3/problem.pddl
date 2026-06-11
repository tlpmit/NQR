(define
  (problem kitchen-problem-0)
  (:domain kitchen-domain-0)
  (:objects
    world - qr::world-type
    floor - floor-type
    movo - movo-type
    table - table-type

    blue-box - qrgeom::box-type
    red-box - qrgeom::box-type

    potted-meat-can - potted-meat-can-type
    tomato-soup-can - tomato-soup-can-type
  )
  (:init
    (weld world::world floor (0, 0, 0, 0, 0, 0))
    (body-pose movo (-1, -1, 0.071, 0.0, -0.0, 0.0))
    (weld world::world table (0, 0, 0, 0, 0, 0))
    (body-pose potted-meat-can (0.03284035, 0.02635271, 0.735188, 0, 0, 0.7853982393431673))
    (body-pose tomato-soup-can (0.0093421 , -0.28422424,  0.731921, 0, 0, 0.3926991550385985))
    (body-pose blue-box (0.   , 0.2  , 0.755, 0, 0, 0))
    (body-pose red-box  (0.   , 0.4  , 0.755, 0, 0, 0))

    (qrgeom::box-shape blue-box (0.15, 0.15, 0.05))
    (qrgeom::box-color blue-box (0, 0, 1, 1))
    (qrgeom::box-shape red-box  (0.15, 0.15, 0.05))
    (qrgeom::box-color red-box  (1, 0, 0, 1))
  )
  (:goal (and
  ))
)