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
  :slots (_image _bboxes ))

(defmethod detection_only::Bbox6Array
  (:init
   (&key
    ((:image __image) (instance detection_only::Image :init))
    ((:bboxes __bboxes) ())
    )
   (send-super :init)
   (setq _image __image)
   (setq _bboxes __bboxes)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
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
    ;; detection_only/Image _image
    (send _image :serialization-length)
    ;; detection_only/Bbox_6[] _bboxes
    (apply #'+ (send-all _bboxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; detection_only/Image _image
       (send _image :serialize s)
     ;; detection_only/Bbox_6[] _bboxes
     (write-long (length _bboxes) s)
     (dolist (elem _bboxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; detection_only/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
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

(setf (get detection_only::Bbox6Array :md5sum-) "d023923a68c57000f2065e0f664c2a6d")
(setf (get detection_only::Bbox6Array :datatype-) "detection_only/Bbox6Array")
(setf (get detection_only::Bbox6Array :definition-)
      "# bboxes 
# [x1,y1,x2,y2,conf,class]
Image image
Bbox_6[] bboxes 


================================================================================
MSG: detection_only/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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



(provide :detection_only/Bbox6Array "d023923a68c57000f2065e0f664c2a6d")


