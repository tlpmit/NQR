(define
  (problem kitchen-belief-0) ; lpk: should this be problem0_wall or something?
  (:domain kitchen-domain-0)  ; lpk: ditto
  (:objects
  )
  (:init
    (robot movo)
    (workspace ((-2, 2), (-2, 2)))
    (use-right)
    (use-base)

  )
  (:goal (and (on ?x ?y)
              (on ?y ?z)
  ))
)