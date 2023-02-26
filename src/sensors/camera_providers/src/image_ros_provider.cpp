#include "sensors/camera_providers/image_ros_provider.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <glog/logging.h>

#include "common/file_tools/camera_frame_times.hpp"
#include "sensors/camera_providers/camera_provider.hpp"

namespace dsopp {
namespace sensors {
namespace providers {

namespace {}  // namespace

// write a callback function that will be called when a new image is received

void ImageRosProvider::ImageReadCallback(const sensor_msgs::Image& img_msg) {
  ROS_INFO("I heard: frame_id [%s]", img_msg->header.frame_id);
  ROS_INFO("I heard: timestamp [%s]", img_msg->header.stamp);
  // use cv_bridge to read data from sensor_msgs:Image& img_msg to cv::Mat
  // Assuming img_msg is the ROS image message to be converted
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;  // Or handle the error in some other way
  }

  // Access the converted OpenCV image
  cv::Mat img = cv_ptr->image;
  Precision exposure_time = 1;

  uint64_t timestamp = img_msg->header.stamp.toNSec();

  // feed the cv::Mat to the CameraDataFrame
  auto frame = std::make_unique<CameraDataFrame>(img_msg->header.frame_id, img, exposure_time, timestamp);

  frame_batch_.push_back(frame);

  ROS_INFO("current frame_batch_ size is " << frame_batch_.size());
}

// void ImageRosProvider::CameraInfoCallback(const sensor_msgs::Image& img_msg) {}

void ImageRosProvider::CameraInfoCallack(const sensor_msgs::CameraInfo info_msg) {
  ROS_INFO("I heard: frame_id from cam info [%s]", info_msg->header.frame_id);
  ROS_INFO("I heard: distortion_model_ [%s]", info_msg->distortion_model);

  distortion_model_ = info_msg->distortion_model;
}

ImageRosProvider::ImageRosProvider(std::string image_topic, std::string camera_info_topic,
                                   bool convert_to_grayscale = false)
    : CameraProvider(), image_topic_(image_topic), camera_info_topic_(camera_info_topic) {

  ros::NodeHandle nh;
  subscriber_ = nh.subscribe(image_topic_, 1, &ImageRosProvider::ImageReadCallback, this);

  camera_info_subscriber_ = nh.subscribe(camera_info_topic_, 1, &ImageRosProvider::CameraInfoCallback, this);

  Precision height = 240;
  Precision width = 320;

  image_size_ << height, width;
}

std::unique_ptr<CameraDataFrame> ImageRosProvider::nextFrame() {
  
  // terminates when there are no frames available
  if frame_batch_.size() == 0) {
    return nullptr;
  }
  // We assume that the frame_batch_ is not empty
  auto frame = std::move(frame_batch_.front());
  frame_batch_.pop_front();
  return frame;
}

size_t ImageRosProvider::queueSize() { return std::numeric_limits<size_t>::max(); }

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
