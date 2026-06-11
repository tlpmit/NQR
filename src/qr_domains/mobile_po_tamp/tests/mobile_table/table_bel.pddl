
(define
  (problem foo)
  (:domain foo) 
  (:objects
    ;table - table-type
    grape-class
    spot
    shadow_world - qrgeom::box-type
    shadow_source - qrgeom::box-type
  )
  (:init
    (robot spot)
    (body-pose spot (0.5, 0.0, 0.0, 0.0, 0.0, 0.0))

    ; Note that use-base and use-right need to be set in the world file, not the bel file

    ;(body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))
    ;(support-surface table)

    (qrgeom::box-shape shadow_world (2, 2, 1))
    (qrgeom::box-color shadow_world (0.5, 0.5, 0.5, 0.5))    
    (body-pose shadow_world (1.55, 0, 0.5, 0, 0, 0.0))

    ; shadow source is used by polyhedral shadow code, which needs an object responsible for shadow.
    (qrgeom::box-shape shadow_source (0.01, 0.01, 0.01))
    (qrgeom::box-color shadow_source (0.5, 0.5, 0.5, 0.5))    
    (body-pose shadow_source (1.75, 0, 2., 0, 0, 0.0))    
    
  )
  (:goal (and (exists ?g (and (class ?g grape-class) (holding ?g))))
    )
)