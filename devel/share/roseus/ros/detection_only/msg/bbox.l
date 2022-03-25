;; Auto-generated. Do not edit!


(when (boundp 'detection_only::bbox)
  (if (not (find-package "DETECTION_ONLY"))
    (make-package "DETECTION_ONLY"))
  (shadow 'bbox (find-package "DETECTION_ONLY")))
(unless (find-package "DETECTION_ONLY::BBOX")
  (make-package "DETECTION_ONLY::BBOX"))

(in-package "ROS")
;;//! \htmlinclude bbox.msg.html


(defclass detection_only::bbox
  :super ros::object
  :slots (_bbox_info ))

(defmethod detection_only::bbox
  (:init
   (&key
    ((:bbox_info __bbox_info) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _bbox_info __bbox_info)
   self)
  (:bbox_info
   (&optional __bbox_info)
   (if __bbox_info (setq _bbox_info __bbox_info)) _bbox_info)
  (:serialization-length
   ()
   (+
    ;; float32[] _bbox_info
    (* 4    (length _bbox_info)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _bbox_info
     (write-long (length _bbox_info) s)
     (dotimes (i (length _bbox_info))
       (sys::poke (elt _bbox_info i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _bbox_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bbox_info (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _bbox_info i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get detection_only::bbox :md5sum-) "913b440d53de141b21b184494ddea913")
(setf (get detection_only::bbox :datatype-) "detection_only/bbox")
(setf (get detection_only::bbox :definition-)
      "# 1 bbox 
# [x1,y1,x2,y2,conf,class]
float32[] bbox_info 
")



(provide :detection_only/bbox "913b440d53de141b21b184494ddea913")


