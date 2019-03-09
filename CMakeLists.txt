cmake_minimum_required(VERSION 2.8)
project(image_view)

find_package(catkin REQUIRED COMPONENTS camera_calibration_parsers cv_bridge image_transport message_filters message_generation nodelet rosconsole roscpp std_srvs stereo_msgs)
catkin_package()

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

# Nodelet library
add_library(image_view src/nodelets/disparity_nodelet.cpp src/nodelets/window_thread.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
)
install(TARGETS image_view
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(disparity_view src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

install(TARGETS disparity_view
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
