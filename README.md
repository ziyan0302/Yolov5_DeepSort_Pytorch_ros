# Yolov5_DeepSort_Pytorch_ros
Connect Yolov5 detection module and DeepSort tracking module via ROS 
# Prepare the environment
- Add a .pth in /usr/lib/python3/dist-packages/
  - User can take environment.sh as reference to vim a deepsort.pth
- Build catkin workspace first time
In Yolov5_DeepSort_Pytorch_ros root folder:
```
catkin_make
```



# Commands
```
roscore
source devel/setup.bash
```

roslaunch realseanse camera:
```
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=30
```

To rosrun Yolov5 detection node:
```
rosrun detection_only det_only_v3.py 
```

To rosrun DeepSort tracking node:
```
rosrun detection_only track_only_final.py 
rosrun detection_only track_only_final.py --save-img
```

# Topics
 Topics                  | Publisher      | Subscriber 
-------------------------| ---------------|------------------------
 /camera/color/image_raw | realsense_node | detection_node 
 /det_result             | detection_node | tracking_node
 /track_result           | tracking_node  |