(define
  (domain movo_free_domain)

  (:object-types
    (movo-type "package://qr_assets/movo_description_drake/movo.urdf")
    (wall-type "package://OpenWorldTAMP/models/aidan_world.sdf")
  )

  (:predicates
    ; Useful for specifying goals, but too vague for initial conds
    ; body ?x is resting on body ?y   
    (on ?x ?y)
  )
)
