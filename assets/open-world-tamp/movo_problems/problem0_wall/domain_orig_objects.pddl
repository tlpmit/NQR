(define
  (domain pr2-problem0) ; lpk : different name

  (:object-types
    (movo-type             "package://qr_assets/movo_description_drake/movo.urdf")
    (floor-type            "package://OpenWorldTAMP/models/floor.urdf")
    (wall-type             "package://OpenWorldTAMP/models/aidan_world.sdf")
    (table-type            "package://OpenWorldTAMP/models/table.urdf")
    (potted-meat-can-type  "package://qr_asserts/010_potted_meat_can_hydro.sdf")
    (old-potted-meat-can-type  "package://qr_assets/ycb_hydro/010_potted_meat_can.sdf")
    (old-sugar-box-type  "package://YCB/004_sugar_box/textured.urdf")
    (old-banana-type  "package://YCB/011_banana/textured.urdf")    
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
