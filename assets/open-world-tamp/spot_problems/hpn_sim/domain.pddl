(define
  (domain pr2-problem0) ; lpk : different name

  (:object-types
    (floor-type        "package://OpenWorldTAMP/models/floor.urdf")
    (table-type        "package://OpenWorldTAMP/models/table.urdf")
    (sugar-box-type  	 "package://drake_models/ycb/004_sugar_box.sdf")
  )

  (:predicates
    (on ?obj ?region)
    (color ?obj ?color)
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
