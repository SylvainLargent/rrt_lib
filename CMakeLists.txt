cmake_minimum_required(VERSION 2.8.3)
project(rrt_lib)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/Env.cpp
  src/Node.cpp
  src/Point3.cpp
  src/RRT.cpp
  src/RRT_STAR.cpp
  src/Segment.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)