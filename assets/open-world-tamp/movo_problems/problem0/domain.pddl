(define
  (domain pr2-problem0)

  (:object-types
    (movo-type             "package://qr_assets/movo_description_drake/movo_robotiq_collision.urdf")
    (floor-type            "package://OpenWorldTAMP/models/floor.urdf")
    (table-type            "package://OpenWorldTAMP/models/table.urdf")
    (potted-meat-can-type  "package://qr_assets/ycb_hydro/010_potted_meat_can.sdf")
  )

  (:predicates
    (on ?obj ?region)
  )
)
