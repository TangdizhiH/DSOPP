#include "sensors/camera_providers/image_ros_provider.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <glog/logging.h>

#include "common/file_tools/camera_frame_times.hpp"

namespace dsopp {
namespace sensors {
namespace providers {

namespace {

void ImageRosProvider::TopicCallback(const sensor_msgs::Image& img_msg) {}

ImageRosProvider::ImageRosProvider(std::string topic) : topic_(topic) { 
  ros::NodeHandle nh; 

  subscriber_ = nh.subscribe(topic_, 1, &ImageRosProvider::TopicCallback, this);
}

}  // namespace

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
