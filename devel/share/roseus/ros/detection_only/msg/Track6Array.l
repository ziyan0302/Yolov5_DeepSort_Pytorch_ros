;; Auto-generated. Do not edit!


(when (boundp 'detection_only::Track6Array)
  (if (not (find-package "DETECTION_ONLY"))
    (make-package "DETECTION_ONLY"))
  (shadow 'Track6Array (find-package "DETECTION_ONLY")))
(unless (find-package "DETECTION_ONLY::TRACK6ARRAY")
  (make-package "DETECTION_ONLY::TRACK6ARRAY"))

(in-package "ROS")
;;//! \htmlinclude Track6Array.msg.html


(defclass detection_only::Track6Array
  :super ros::object
  :slots (_tracks ))

(defmethod detection_only::Track6Array
  (:init
   (&key
    ((:tracks __tracks) ())
    )
   (send-super :init)
   (setq _tracks __tracks)
   self)
  (:tracks
   (&rest __tracks)
   (if (keywordp (car __tracks))
       (send* _tracks __tracks)
     (progn
       (if __tracks (setq _tracks (car __tracks)))
       _tracks)))
  (:serialization-length
   ()
   (+
    ;; detection_only/Track_6[] _tracks
    (apply #'+ (send-all _tracks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; detection_only/Track_6[] _tracks
     (write-long (length _tracks) s)
     (dolist (elem _tracks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; detection_only/Track_6[] _tracks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracks (let (r) (dotimes (i n) (push (instance detection_only::Track_6 :init) r)) r))
     (dolist (elem- _tracks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get detection_only::Track6Array :md5sum-) "bcae8edf8c2ae4b9b6e115eb41e70bec")
(setf (get detection_only::Track6Array :datatype-) "detection_only/Track6Array")
(setf (get detection_only::Track6Array :definition-)
      "# bboxes 
# [x1,y1,x2,y2,idx,class]
Track_6[] tracks 
================================================================================
MSG: detection_only/Track_6
# 1 bbox 
# [x1,y1,x2,y2,conf,class]
# float32[] bbox_info
float32 x1
float32 y1
float32 x2
float32 y2
float32 id
float32 cls
")



(provide :detection_only/Track6Array "bcae8edf8c2ae4b9b6e115eb41e70bec")


