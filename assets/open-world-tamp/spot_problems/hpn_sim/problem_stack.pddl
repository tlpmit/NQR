(define
  (problem kitchen-problem-0) ; lpk: should this be problem0 or something?
  (:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
    world - qr::world-type
    floor - floor-type
    table - table-type

    pillar - qrgeom::box-type
    red-box - qrgeom::box-type
    blue-box - qrgeom::box-type
  )
  (:init
    (weld world::world floor::base (0, 0, -0.025, 0, 0, 0))  ; avoid contact of floor with spot
    (weld world::world table (1, 0, 0, 0, 0, 0))
    ;(weld world::world pillar::box (1, 0.15, 0.7305, 0, 0, 0.7853982393431673))
    (weld world::world pillar::box (1, 0.15, 0.74, 0, 0, 0.7853982393431673))

    (qrgeom::box-shape pillar (0.3, 0.3, 0.02))
    (qrgeom::box-color pillar (0, 1, 0, 1.0))
    (qrgeom::box-contact-model pillar "compliant-hydroelastic")

    (body-pose red-box (0.9, -0.2, 0.84, -1.57, 0, 1.57))
    (qrgeom::box-shape red-box (0.10, 0.10, 0.05))
    (qrgeom::box-color red-box (1, 0, 0, 1.0))  ; red
    (qrgeom::box-mass red-box 0.020)  ; 20 grams
    (qrgeom::box-inertia red-box (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model red-box "compliant-hydroelastic")

    ;(body-pose blue-box (0.9, 0.2, 0.945, -1.57, 0, 1.57))
    (body-pose blue-box (0.9, -0.2, 0.945, -1.57, 0, 1.57))
    (qrgeom::box-shape blue-box (0.10, 0.10, 0.05))
    (qrgeom::box-color blue-box (0, 0, 1, 1.0))  ; blue
    (qrgeom::box-mass blue-box 0.020)  ; 20 grams
    (qrgeom::box-inertia blue-box (4e-5, 4e-5, 4e-5))
    (qrgeom::box-contact-model blue-box "compliant-hydroelastic")

    (workspace ((-2, 2), (-2, 2)))

    ; some helpful static facts
    (use-right)
    (use-base)
    (graspable red-box)
    (support-surface table)
    (support-surface pillar)

  )
  (:goal (and (on red-box pillar)
  ))
)