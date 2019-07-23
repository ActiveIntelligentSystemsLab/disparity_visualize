/*****************************************
* Software License Agreement (BSD License)
* Please see: LICENSE
*
* Copyright 2019 Hironori Fujimoto
******************************************/

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disparity_visualize");

  nodelet::Loader nodelet_loader(false);

  nodelet::M_string remappings;
  nodelet::V_string args(argv + 1, argv + argc);

  nodelet_loader.load(ros::this_node::getName(), "disparity_visualize/disparity_visualize", remappings, args);

  ros::spin();
  return 0;
}