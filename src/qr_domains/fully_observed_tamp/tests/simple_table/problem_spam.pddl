(define
  (problem foo)
  (:domain foo) 
  (:objects
    movo 
    table - table-type
    spam - spam-type
  )
  (:init
    (body-pose movo (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (body-pose table (1, 0, 0, 0, 0, 0))
    (body-pose spam (0.85, -0.2, 0.745, 0, 0, 0))
    
    (workspace ((-2, -2, 0), (2, 2, 3)))    ; corners

    ; some helpful static facts
    (robot movo)
    (use-right)
    (graspable spam)
    (support-surface table)

  )
  (:goal (and (holding spam))
    )
)