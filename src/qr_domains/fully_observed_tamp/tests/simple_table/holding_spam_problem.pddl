
(define
  (problem kitchen-problem-0) ; lpk: should this be problem0_wall or something?
  (:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
    world - qr::world-type
    ;floor - floor-type
    ;walls - wall-type
    movo 
    table - table-type

    pillar - qrgeom::box-type
    ;potted-meat-can - potted-meat-can-type
    sugar-box - sugar-box-type
  )
  (:init
    ;(weld world::world floor::base (0, 0, 0, 0, 0, 0))
    ;(body-pose movo (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (weld world::world table (1, 0, 0, 0, 0, 0))
    ;(weld world::world pillar::box (1, 0.35, 0.7305, 0, 0, 0.7853982393431673))
    (weld world::world pillar (1, 0.35, 0.7305, 0, 0, 0.7853982393431673))
    ; (body-pose potted-meat-can (1.03284035, 0.02635271, 0.79, 0., 1.5714, -0.7853982393431673))
    ;(body-pose potted-meat-can (0.85, 0.2, 0.78, 0., 1.5714, -0.7853982393431673))
    (body-pose sugar-box (1.0, -0.1, 0.85, 0, 1.5714, -0.7853982393431673))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.02))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))

    (workspace ((-3, -3, 0), (3, 3, 3)))

    ; some helpful static facts
    (robot movo)
    (use-right)
    ;(use-base)
    ;(graspable potted-meat-can)
    (graspable sugar-box)
    (support-surface table)
    (support-surface pillar)

  )
  (:goal (and (holding sugar-box) ))
  ;(:goal (and (on sugar-box potted-meat-can) ))
)