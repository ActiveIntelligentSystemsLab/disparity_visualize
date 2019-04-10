/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/opencv.hpp>

namespace disparity_view {

class DisparityNodelet : public nodelet::Nodelet
{
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher pub_;
  ros::Subscriber sub_;

  bool initialized;
  
  virtual void onInit();
  
  void imageCb(const stereo_msgs::DisparityImageConstPtr& msg);

public:
  ~DisparityNodelet();
};

DisparityNodelet::~DisparityNodelet()
{
}

void DisparityNodelet::onInit()
{
  initialized = false;
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();
  image_transport_.reset(new image_transport::ImageTransport(local_nh));
  const std::vector<std::string>& argv = getMyArgv();

  // Internal option, should be used only by image_view nodes
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  bool autosize;
  local_nh.param("autosize", autosize, false);

  sub_ = nh.subscribe<stereo_msgs::DisparityImage>("disparity", 1, &DisparityNodelet::imageCb, this);
  pub_ = image_transport_->advertise("image", 1);
}

void DisparityNodelet::imageCb(const stereo_msgs::DisparityImageConstPtr& msg)
{
  // Check for common errors in input
  if (msg->min_disparity == 0.0 && msg->max_disparity == 0.0)
  {
    NODELET_ERROR_THROTTLE(30, "Disparity image fields min_disparity and "
                           "max_disparity are not set");
    return;
  }
  if (msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    NODELET_ERROR_THROTTLE(30, "Disparity image must be 32-bit floating point "
                           "(encoding '32FC1'), but has encoding '%s'",
                           msg->image.encoding.c_str());
    return;
  }
  
  if(!initialized) {
    initialized = true;
  }
  // Colormap and display the disparity image
  float min_disparity = msg->min_disparity;
  float max_disparity = msg->max_disparity;

  const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
                             (float*)&msg->image.data[0], msg->image.step);
  
  cv::Mat normalized_disparity(msg->image.height, msg->image.width, CV_8UC1);

  for (int i = 0; i < dmat.rows * dmat.cols; ++i) {
    float disparity_at = dmat.at<float>(i);
    if (disparity_at > max_disparity)
      disparity_at = max_disparity;
    if (disparity_at < min_disparity)
      disparity_at = min_disparity;
    normalized_disparity.at<unsigned char>(i) = (disparity_at - min_disparity) / (max_disparity - min_disparity) * 255;
  }

  cv::Mat colored_disparity;
  cv::applyColorMap(normalized_disparity, colored_disparity, cv::COLORMAP_HSV);

  cv_bridge::CvImage cv_image(msg->header, sensor_msgs::image_encodings::BGR8, colored_disparity);
  pub_.publish(cv_image.toImageMsg());
}

} // namespace disparity_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( disparity_view::DisparityNodelet, nodelet::Nodelet)
