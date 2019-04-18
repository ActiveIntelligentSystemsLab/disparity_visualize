/*****************************************
* Software License Agreement (BSD License)
* Please see: LICENSE
*
* Copyright 2019 Hironori Fujimoto
******************************************/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/opencv.hpp>

namespace disparity_visualize {

class DisparityVisualizeNodelet : public nodelet::Nodelet
{
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher pub_;
  ros::Subscriber sub_;
  
  virtual void onInit();
  
  void imageCb(const stereo_msgs::DisparityImageConstPtr& msg);

public:
  ~DisparityVisualizeNodelet();
};

DisparityVisualizeNodelet::~DisparityVisualizeNodelet()
{
}

void DisparityVisualizeNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();
  image_transport_.reset(new image_transport::ImageTransport(local_nh));

  sub_ = nh.subscribe<stereo_msgs::DisparityImage>("disparity", 1, &DisparityVisualizeNodelet::imageCb, this);
  pub_ = image_transport_->advertise("image", 1);
}

void DisparityVisualizeNodelet::imageCb(const stereo_msgs::DisparityImageConstPtr& msg)
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
  
  // Colormap and display the disparity image
  float min_disparity = msg->min_disparity;
  float max_disparity = msg->max_disparity;

  const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
                             (float*)&msg->image.data[0], msg->image.step);

  // To save index of invalid pixels
  std::vector<int> invalid_indices;
  invalid_indices.reserve(dmat.rows * dmat.cols);
  
  // disparity normalized to [0, 255]
  cv::Mat normalized_disparity(msg->image.height, msg->image.width, CV_8UC1);

  for (int i = 0; i < dmat.rows * dmat.cols; ++i) {
    float disparity_at = dmat.at<float>(i);

    if (disparity_at > max_disparity || disparity_at < min_disparity)
      invalid_indices.push_back(i);

    normalized_disparity.at<unsigned char>(i) = (disparity_at - min_disparity) / (max_disparity - min_disparity) * 255;
  }

  // Colored by HSV color map
  cv::Mat colored_disparity;
  cv::applyColorMap(normalized_disparity, colored_disparity, cv::COLORMAP_HSV);

  for (int invalid_index : invalid_indices)
  {
    cv::Vec3b &invalid_pixel = colored_disparity.at<cv::Vec3b>(invalid_index);
    invalid_pixel = cv::Vec3b(0, 0, 0);
  }

  cv_bridge::CvImage cv_image(msg->header, sensor_msgs::image_encodings::BGR8, colored_disparity);
  pub_.publish(cv_image.toImageMsg());
}

} // namespace disparity_visualize

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( disparity_visualize::DisparityVisualizeNodelet, nodelet::Nodelet)
