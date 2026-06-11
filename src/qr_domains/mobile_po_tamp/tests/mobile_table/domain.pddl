(define
  (domain simple-mobile-robot-table) 

  (:object-types
    (large-cap-type    "package://HPNModels/grape/large_cap.sdf")
    (small-cap-type    "package://HPNModels/grape/small_cap.sdf")
    (grape-type    "package://HPNModels/grape/grape.sdf")
    (big-grape-type "package://HPNModels/grape/big_grape.sdf")
    (red-grape-type "package://HPNModels/grape/red_grape.sdf")
    (green-grape-type "package://HPNModels/grape/green_grape.sdf")
    (table-type    "package://HPNModels/misc/table.sdf")

  )

  (:predicates
    (on ?obj ?region)
    (holding ?obj)
    (graspable ?obj)
    (support-surface ?obj)
    (color ?obj ?color)
    (color-most ?obj ?color)
    (class ?x ?y)      ; object category (not the same as type above)
    (not-equal ?x ?y)
    (robot ?robot)
    (workspace ?bounds)
    (use-right)
    (use-left)
    (use-base)
    (shadow-extents ?x)
    (description ?x ?string)
    (shadow-pose ?p)
  )
)
