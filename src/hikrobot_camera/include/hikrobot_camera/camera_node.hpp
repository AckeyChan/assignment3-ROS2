#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"
#include "hikrobot_camera/camera_node.hpp"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
  std::string type;
  std::string parameter;
  int retry_count;
  int latency;
  int Rnet;
  int counter;
  void* handle;
  MV_CC_DEVICE_INFO* detect_camera(MV_CC_DEVICE_INFO_LIST device_list, std::string _type, std::string _parameter);

  // TODO(student): Design the interfaces and resource ownership required by
  // your implementation. No SDK handles or camera operations are provided.
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
