(define
  (domain simple-movo-table) 

  (:object-types
    (spam-type     "package://HPNModels/ycb_hydro/010_potted_meat_can/textured.urdf")
    (strawberry-type  "package://HPNModels/ycb/012_strawberry/textured.urdf")
    (table-type    "package://HPNModels/misc/table.sdf")
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
