(define
  (problem spam-on-spam-on-spam) 
  (:objects
    world - qr::world-type
    floor - floor-type
    walls - wall-type
    movo - movo-type
    table - table-type

    pillar - qrgeom::box-type
    spam1 - potted-meat-can-type
    spam2 - potted-meat-can-type
    spam3 - potted-meat-can-type
  )
  (:init
    (weld world::world floor::base (0, 0, -0.01, 0, 0, 0)) 
    (body-pose movo (0, 0.0, 0.0, 0.0, -0.0, 0.0))
    (weld world::world table (1, 0, 0, 0, 0, 0))
    (weld world::world pillar::box (1, -0.5, 0.7305, 0, 0, 0.7853982393431673))

    (body-pose spam1 (1.0, -0.1, 0.78, 0, 1.5714, -0.7853982393431673))
    (body-pose spam2 (1.0,  0.05, 0.78, 0, 1.5714, -0.7853982393431673))

    ; already on spam3
    ;(body-pose spam2 (1.0,  -0.25, 0.8726, 0, 1.5714, -0.7853982393431673))

    (body-pose spam3 (1.0, -0.25, 0.78, 0, 1.5714, -0.7853982393431673))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.02))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))

    (workspace ((-2, -2, 0), (2, 2, 2)))

    ; some helpful static facts
    (robot movo)
    (use-right)
    (graspable spam1)
    (graspable spam2)
    ;(graspable spam3)
    (support-surface spam2)
    (support-surface spam3)
    (support-surface table)
    (support-surface pillar)

  )
  (:goal (and (on spam1 spam2) 
              (on spam2 spam3)
  ))
)