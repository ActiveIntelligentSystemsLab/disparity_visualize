cmake_minimum_required(VERSION 2.8)
project(image_view)

find_package(catkin REQUIRED COMPONENTS camera_calibration_parsers cv_bridge image_transport message_filters message_generation nodelet rosconsole roscpp std_srvs stereo_msgs)
catkin_package()

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

# Nodelet library
add_library(image_view src/nodelets/disparity_nodelet.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
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
