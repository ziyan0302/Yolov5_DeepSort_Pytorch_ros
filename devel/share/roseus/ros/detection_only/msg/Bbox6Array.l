;; Auto-generated. Do not edit!


(when (boundp 'detection_only::Bbox6Array)
  (if (not (find-package "DETECTION_ONLY"))
    (make-package "DETECTION_ONLY"))
  (shadow 'Bbox6Array (find-package "DETECTION_ONLY")))
(unless (find-package "DETECTION_ONLY::BBOX6ARRAY")
  (make-package "DETECTION_ONLY::BBOX6ARRAY"))

(in-package "ROS")
;;//! \htmlinclude Bbox6Array.msg.html


(defclass detection_only::Bbox6Array
  :super ros::object
  :slots (_bboxes ))

(defmethod detection_only::Bbox6Array
  (:init
   (&key
    ((:bboxes __bboxes) ())
    )
   (send-super :init)
   (setq _bboxes __bboxes)
   self)
  (:bboxes
   (&rest __bboxes)
   (if (keywordp (car __bboxes))
       (send* _bboxes __bboxes)
     (progn
       (if __bboxes (setq _bboxes (car __bboxes)))
       _bboxes)))
  (:serialization-length
   ()
   (+
    ;; detection_only/Bbox_6[] _bboxes
    (apply #'+ (send-all _bboxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; detection_only/Bbox_6[] _bboxes
     (write-long (length _bboxes) s)
     (dolist (elem _bboxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; detection_only/Bbox_6[] _bboxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bboxes (let (r) (dotimes (i n) (push (instance detection_only::Bbox_6 :init) r)) r))
     (dolist (elem- _bboxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get detection_only::Bbox6Array :md5sum-) "61b8f2a52aff12e25f2957c1314c78d3")
(setf (get detection_only::Bbox6Array :datatype-) "detection_only/Bbox6Array")
(setf (get detection_only::Bbox6Array :definition-)
      "# bboxes 
# [x1,y1,x2,y2,conf,class]
Bbox_6[] bboxes 
================================================================================
MSG: detection_only/Bbox_6
# 1 bbox 
# [x1,y1,x2,y2,conf,class]
# float32[] bbox_info
float32 x1
float32 y1
float32 x2
float32 y2
float32 conf
float32 cls
")



(provide :detection_only/Bbox6Array "61b8f2a52aff12e25f2957c1314c78d3")


