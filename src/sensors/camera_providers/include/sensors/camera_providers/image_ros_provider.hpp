#ifndef DSOPP_IMAGE_ROS_CAMERA_DATA_PROVIDER_HPP
#define DSOPP_IMAGE_ROS_CAMERA_DATA_PROVIDER_HPP

#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/camera_provider.hpp"

namespace dsopp {

// todo: got timestamp without using the following struct

namespace sensors {
namespace providers {

class ImageRosProvider final : public CameraProvider {
 public:
  explicit ImageRosProvider(std::string image_topic, std::string camera_info_topic, bool convert_to_grayscale);
  // The parmaters should be passed. The ros topic of image to subscribe, whether to convert to grayscale

  std::unique_ptr<CameraDataFrame> nextFrame() override;

  /**
   * @brief The required method from CameraProvider. But not known on Ros provider. So I'll just set it as 42
   *
   *
   * @return size_t 42 so long as the topic starts to send stuffs
   */
  size_t queueSize() override;

  ~ImageFolderProvider() override = default;

 private:
  /**
   * @brief
   * Used to fill the batch of frames when it is empty
   *
   */
  void ImageReadCallback(const sensor_msgs::Image& img_msg);
  void CameraInfoCallack(const sensor_msgs::CameraInfo info_msg);

  bool convert_to_grayscale_;
  std::deque<std::unique_ptr<CameraDataFrame>> frame_batch_;
  /** Container containing times */
  std::map<uint64_t, common::file_tools::CameraFrameTimes> times_;
  ros::Subscriber subscriber_;
  ros::Subscriber camera_info_subscriber_;

  std::string image_topic_;
  std::string camera_info_topic_;

  std::string distortion_model_;
  /** Container containing camera frames */
  std::deque<std::unique_ptr<CameraDataFrame>> frame_batch_;
};

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_IMAGE_ROS_CAMERA_DATA_PROVIDER_HPP
