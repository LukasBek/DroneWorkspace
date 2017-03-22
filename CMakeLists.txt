cmake_minimum_required(VERSION 2.8.3)
project(ardronepkg)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  geographic_msgs
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package( OpenCV 2 REQUIRED )



 catkin_package(
  
  CATKIN_DEPENDS ardrone_autonomy cv_bridge geographic_msgs geometry_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
 )

 

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
##add_executable(testclass src/testclass.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBRARIES})
add_executable(Image_converter src/Image_converter.cpp)
target_link_libraries(Image_converter ${catkin_LIBRARIES})
target_link_libraries(Image_converter ${OpenCV_LIBRARIES})
add_executable(FindCirclesWebcam src/FindCirclesWebcam.cpp)
target_link_libraries(FindCirclesWebcam ${catkin_LIBRARIES})
target_link_libraries(FindCirclesWebcam ${OpenCV_LIBRARIES})
