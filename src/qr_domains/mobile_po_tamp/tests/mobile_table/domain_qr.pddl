(define
  (domain simple-mobile-robot-table) 

  (:object-types

    (table-type    "package://OpenWorldTAMP/models/table.urdf")
    (floor-type    "package://OpenWorldTAMP/models/floor.urdf")

  )

  (:predicates
    (on ?obj ?region)
    (holding ?obj)
    (graspable ?obj)
    (support-surface ?obj)
    (color ?obj ?color)
    (color-most ?obj ?color)
    (description ?obj ?descr)
    (class ?x ?y)      ; object category (not the same as type above)
    (not-equal ?x ?y)
    (robot ?robot)
    (workspace ?bounds)
    (use-right)
    (use-left)
    (use-base)
    (shadow-extents ?x)
    (shadow-pose ?p)
  )
)
