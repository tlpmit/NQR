(define
  (problem base-problem) 
  (:objects
    pr2
    table - table-type
    green-mat - qrgeom::box-type
    red-mat - qrgeom::box-type
    shpam1 - qrgeom::box-type
    shpam2 - qrgeom::box-type
    shpam3 - qrgeom::box-type
    blue
    green
    red
    base head left left-gripper right right-gripper
  )
  (:init
    ; robot and table
    (body-pose table (1, 0, 0, 0, 0, 0))
    (workspace ((-2, -2, -2), (2, 2, 2)))

    (chain-conf base (0.2, -0.4,  0.  ))
    (chain-conf head (0.00000000, 0.87266463))
    (chain-conf left (1.30899694, 0.87266463, 1.91986218, -1.91986218, -0.34906585, -0.17453293, -0.17453293))
    (chain-conf left-gripper 0.08000007)
    (chain-conf right (-0.26860425,  0.33310085, -1.55      , -1.19483991, -2.48690097,
       -1.99631823, -0.94273091))
    (chain-conf right-gripper 0.08 )

    ; colored mats on table
    (qrgeom::box-shape green-mat (0.3, 0.3, 0.02))
    (qrgeom::box-shape red-mat (0.3, 0.3, 0.02))
    (qrgeom::box-color green-mat (0, 1, 0, 1.0))
    (qrgeom::box-color red-mat (1, 0, 0, 1.0))
    (body-pose green-mat::box (1.2, -0.3, 0.73, 0, 0, 0))
    (body-pose red-mat::box (0.8, -0.5, 0.73, 0, 0, 0))

    ; manipulanda
    (qrgeom::box-shape shpam1 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam2 (0.1, 0.05, 0.05))
    (qrgeom::box-shape shpam3 (0.1, 0.05, 0.05))
    (qrgeom::box-color shpam1 (0, 0, 1, 1.0))    
    (qrgeom::box-color shpam2 (0, 0, 1, 1.0))    
    (qrgeom::box-color shpam3 (0, 0, 1, 1.0))    
    (body-pose shpam1 (0.8, 0.0, 0.76, 0, 0, 0))
    (body-pose shpam2 (0.8,  0.25, 0.76, 0, 0, 0))
    (body-pose shpam3 (0.8, -0.25, 0.76, 0, 0, 0))

    (init-holding shpam1 right 0)

    ; additional declarations
    (robot pr2)
    (use-right)
    (use-base)
    (graspable shpam1)
    (graspable shpam2)
    (graspable shpam3)
    (support-surface table)
    (support-surface red-mat)
    (support-surface green-mat)
  )
  (:goal (and (holding shpam2)))
)