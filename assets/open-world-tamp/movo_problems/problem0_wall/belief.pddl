(define
  (problem kitchen-belief-0) ; lpk: should this be problem0_wall or something?
  (:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
      movo - movo-type
  )
  (:init
    (robot movo)
    (workspace ((-2, -2, 0), (2, 2, 2)))
    (use-right)
    ;(use-base)

  )
 (:goal (and (holding ?thing)))  
)