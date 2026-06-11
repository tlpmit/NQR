(define (problem movo_1)
  (:domain movo_free_domain)
  (:objects
      world - qr::world-type
      movo - movo-type
      walls - wall-type
   )
   (:init
      (body-pose movo (0, 0.0, 0.0, 0.0, 0.0, 0.0))
   )
   (:goal (and
      (body-pose-close-2d movo (1.0, 0.0) 0.1)
   ))
)
