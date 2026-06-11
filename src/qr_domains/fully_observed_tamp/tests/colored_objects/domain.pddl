(define
  (domain colored-boxes)

  (:object-types
    ;(table-type    "package://HPNModels/misc/table.sdf")
    (table-type            "package://OpenWorldTAMP/models/table.urdf")
  )

  (:predicates
    (on ?obj ?region)
    (color ?obj ?color)
    (color-most ?obj ?color)
    (holding ?obj)
    (init-holding ?obj ?hand ?grasp)
    (graspable ?obj)
    (support-surface ?obj)
    (robot ?robot)
    (workspace ?bounds)
    (use-right)
    (use-left)
    (use-base)
    (equal ?a ?b)
    (not-equal ?a ?b)
    (shadow-extents ?ext)
    (shadow-pose ?pose)
    (chain-conf ?chain ?conf)
  )
)
