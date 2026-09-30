#include <exception>
#include <memory>

#include "hikrobot_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<hikrobot_camera::CameraNode>());
  }
  catch (const std::exception & exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("hikrobot_camera"), "节点因未处理异常退出: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
