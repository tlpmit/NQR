(define
  (problem foo)
  (:domain foo) 
  (:objects
    fetch
    spam - spam-type
  )
  (:init
    (body-pose fetch (0, 0.0, 0.1, 0.0, 0.0, 0.0))  ; check z
    (body-pose spam (1.5, 0.0, 0.745, 0, 0, 0))
    
    (workspace ((-2, -2, 0), (2, 2, 3)))    ; corners

    ; some helpful static facts
    (robot fetch)
    (use-right)
    (use-base)
    (graspable spam)

  )
  (:goal (and (holding spam)
    )    
  )
)