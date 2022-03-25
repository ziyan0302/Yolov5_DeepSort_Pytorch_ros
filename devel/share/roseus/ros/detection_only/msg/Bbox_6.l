;; Auto-generated. Do not edit!


(when (boundp 'detection_only::Bbox_6)
  (if (not (find-package "DETECTION_ONLY"))
    (make-package "DETECTION_ONLY"))
  (shadow 'Bbox_6 (find-package "DETECTION_ONLY")))
(unless (find-package "DETECTION_ONLY::BBOX_6")
  (make-package "DETECTION_ONLY::BBOX_6"))

(in-package "ROS")
;;//! \htmlinclude Bbox_6.msg.html


(defclass detection_only::Bbox_6
  :super ros::object
  :slots (_x1 _y1 _x2 _y2 _conf _cls ))

(defmethod detection_only::Bbox_6
  (:init
   (&key
    ((:x1 __x1) 0.0)
    ((:y1 __y1) 0.0)
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    ((:conf __conf) 0.0)
    ((:cls __cls) 0.0)
    )
   (send-super :init)
   (setq _x1 (float __x1))
   (setq _y1 (float __y1))
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   (setq _conf (float __conf))
   (setq _cls (float __cls))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:conf
   (&optional __conf)
   (if __conf (setq _conf __conf)) _conf)
  (:cls
   (&optional __cls)
   (if __cls (setq _cls __cls)) _cls)
  (:serialization-length
   ()
   (+
    ;; float32 _x1
    4
    ;; float32 _y1
    4
    ;; float32 _x2
    4
    ;; float32 _y2
    4
    ;; float32 _conf
    4
    ;; float32 _cls
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x1
       (sys::poke _x1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y1
       (sys::poke _y1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _conf
       (sys::poke _conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cls
       (sys::poke _cls (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x1
     (setq _x1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y1
     (setq _y1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x2
     (setq _x2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y2
     (setq _y2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _conf
     (setq _conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cls
     (setq _cls (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get detection_only::Bbox_6 :md5sum-) "5a76d49beb9ad80ed19e0ba292e46abc")
(setf (get detection_only::Bbox_6 :datatype-) "detection_only/Bbox_6")
(setf (get detection_only::Bbox_6 :definition-)
      "# 1 bbox 
# [x1,y1,x2,y2,conf,class]
# float32[] bbox_info
float32 x1
float32 y1
float32 x2
float32 y2
float32 conf
float32 cls
")



(provide :detection_only/Bbox_6 "5a76d49beb9ad80ed19e0ba292e46abc")


