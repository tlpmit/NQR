(define
  (domain pr2-problem0) ; lpk : different name

  (:object-types
    (movo-type             "package://qr_assets/movo_description_drake/movo.urdf")
    (floor-type            "package://OpenWorldTAMP/models/floor.urdf")
    (wall-type             "package://OpenWorldTAMP/models/aidan_world.sdf")
    (table-type            "package://OpenWorldTAMP/models/table.urdf")
    (potted-meat-can-type  "package://qr_assets/ycb_hydro/010_potted_meat_can.sdf")
    (sugar-box-type  	   "package://drake/manipulation/models/ycb/sdf/004_sugar_box.sdf")
  )

  (:predicates
    (on ?obj ?region)
    (holding ?obj)
    (graspable ?obj)
    (support-surface ?obj)
    (robot ?robot)
    (workspace ?bounds)
    (use-right)
    (use-left)
    (use-base)
  )
)
