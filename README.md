# rrt_lib
A library for ROS implementing simple RRT algorithms for holonomic systems

If you want to use the package in another ROS Package for instance, the tello_driver package
There is a need to modify the tello_driver's Cmake, by adding
      include_directories(
              # include
                ${catkin_INCLUDE_DIRS}
                ${rrt_lib_INDLUDE_DIRS}
              #  ${OpenCV_INCLUDE_DIRS}
      )
     
     
And a need to modify the package.xml of the tello_driver package by addding :

<build_depend>rrt_lib</build_depend>


