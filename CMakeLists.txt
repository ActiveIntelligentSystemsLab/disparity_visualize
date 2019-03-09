cmake_minimum_required(VERSION 2.8)
project(disparity_view)

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

# Nodelet library
add_library(disparity_view src/nodelets/disparity_nodelet.cpp)
target_link_libraries(disparity_view
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS disparity_view
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(disparity_view_node src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS disparity_view_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
