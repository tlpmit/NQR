(define
  (domain simple-panda-table) 

  
  (:object-types
    (large-cap-type    "package://HPNModels/grape/large_cap.sdf")
    (small-cap-type    "package://HPNModels/grape/small_cap.sdf")
    (grape-type    "package://HPNModels/grape/grape.sdf")
    (big-grape-type "package://HPNModels/grape/big_grape.sdf")
    (strawberry-type  "package://HPNModels/ycb/012_strawberry/textured.urdf")
    (spam-type     "package://HPNModels/ycb/010_potted_meat_can/textured.urdf")
    (banana-type     "package://HPNModels/ycb/011_banana/textured.urdf")
    (mustard-bottle-type     "package://HPNModels/ycb/006_mustard_bottle/textured.urdf")
    (bowl-type  "package://HPNModels/ycb/024_bowl/textured.urdf")
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
