# Install script for directory: /home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/detection_only/msg" TYPE FILE FILES
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/msg/Bbox_6.msg"
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/msg/Bbox6Array.msg"
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/msg/Image.msg"
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/msg/Track_6.msg"
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/msg/Track6Array.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/detection_only/cmake" TYPE FILE FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/build/detection_only/catkin_generated/installspace/detection_only-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/include/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/share/roseus/ros/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/share/common-lisp/ros/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/share/gennodejs/ros/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/lib/python3/dist-packages/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/devel/lib/python3/dist-packages/detection_only")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/build/detection_only/catkin_generated/installspace/detection_only.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/detection_only/cmake" TYPE FILE FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/build/detection_only/catkin_generated/installspace/detection_only-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/detection_only/cmake" TYPE FILE FILES
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/build/detection_only/catkin_generated/installspace/detection_onlyConfig.cmake"
    "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/build/detection_only/catkin_generated/installspace/detection_onlyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/detection_only" TYPE FILE FILES "/home/ziyan/Yolov5_DeepSort_Pytorch_ros/src/detection_only/package.xml")
endif()

