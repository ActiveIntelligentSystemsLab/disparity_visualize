cmake_minimum_required(VERSION 2.8)
project(disparity_visualize)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  rosconsole
  roscpp
  stereo_msgs
)
catkin_package()

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(disparity_view src/nodelets/disparity_visualize_nodelet.cpp)
target_link_libraries(disparity_view
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS disparity_view
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
