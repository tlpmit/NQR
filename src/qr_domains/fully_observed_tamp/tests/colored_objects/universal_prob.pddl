(define
  (problem univeral-quantifier-problem) 
  (:objects
    pr2
    table - table-type
    bigbox - qrgeom::box-type
    a - qrgeom::box-type
    b - qrgeom::box-type
    c - qrgeom::box-type
  )
  (:init
    ; robot and table
    (body-pose pr2 (0, 0.0, 0.071, 0.0, -0.0, 0.0))
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))

    ; manipulanda
    (qrgeom::box-shape bigbox (.3, .4, .01))
    (qrgeom::box-color bigbox (1, 1, 0, 1.0)) 

    (qrgeom::box-shape a (0.1, 0.05, 0.05))
    (qrgeom::box-shape b (0.1, 0.05, 0.05))
    (qrgeom::box-shape c (0.1, 0.05, 0.05))
    (qrgeom::box-color a (1, 0, 0, 1.0))
    (qrgeom::box-color b (1, 0, 0, 1.0))
    (qrgeom::box-color c (0, 0, 1, 1.0))
    
    (body-pose bigbox (1.0, -0.4, .73, 0, 0, 0))
    (body-pose b (0.8, 0.0, 0.76, 0, 0, 0))
    (body-pose a (0.8,  0.2, 0.76, 0, 0, 0))
    (body-pose c (0.8, 0.4, 0.76, 0, 0, 0))

    ; additional declarations
    (robot pr2)
    (use-right)
    (use-base)
    (graspable a)
    (graspable b)
    (graspable c)
    (support-surface table)
  )

  ; Goals of more general forms
  ; - or, implies, forall, not

  ; these work
  ;(:goal (and (on a bigbox) (on b bigbox)))
  ;(:goal (and (or (on a bigbox) (on b bigbox))))
  ;(:goal (and (exists ?x (on ?x bigbox))))
  ;(:goal (and (exists ?x (and (on ?x bigbox) (not (color ?x red))))))  ; a not-red thing on bigbox
  ;(:goal (and (forall ?x (or (not (color ?x red)) (on ?x bigbox))))) ; all red things on bigbox
  ;(:goal (and (exists ?x (exists ?y (and (on ?x ?y) (on ?y bigbox) (color ?x red) (color ?y red))))))

  ;(:goal (and (exists ?x (exists ?y (and (color ?x red) (color ?y red) (on ?x bigbox) (on ?y bigbox)
  ;              (not-equal ?x ?y))))))

  (:goal (and (forall ?x (or (and (on ?x bigbox) (not-equal ?x table) (not-equal ?x bigbox))
                             (equal ?x table) 
                             (equal ?x bigbox)))
    ))

  
  )