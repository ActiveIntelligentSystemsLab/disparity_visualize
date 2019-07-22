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

add_library(${PROJECT_NAME} src/disparity_visualize_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(${PROJECT_NAME}_node src/disparity_visualize_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})