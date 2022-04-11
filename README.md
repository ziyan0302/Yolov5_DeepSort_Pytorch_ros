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
To rosrun Yolov5 detection node:
```
rosrun detection_only det_only_v3.py 
```

To rosrun DeepSort tracking node:
```
rosrun detection_only track_only_v2.py --source /home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/test_imgs_less/ --save-img
```
