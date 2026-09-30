#include "hikrobot_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"


namespace hikrobot_camera
{

MV_CC_DEVICE_INFO* CameraNode::detect_camera(MV_CC_DEVICE_INFO_LIST device_list, std::string _type, std::string _parameter)
{
  if (_type == "NULL" || _parameter == "NULL")
  {
    RCLCPP_ERROR(this->get_logger(), "Type or parameter is NULL!");
    rclcpp::shutdown();
  }
  if (_type == "IP")
  {
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++)
    {
      if (device_list.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
      {
        unsigned int ip = device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp;
        std::string ip_str = std::to_string((ip & 0xFF000000) >> 24) + "." +
                             std::to_string((ip & 0x00FF0000) >> 16) + "." +
                             std::to_string((ip & 0x0000FF00) >> 8) + "." +
                             std::to_string((ip & 0x000000FF));
        if (ip_str == _parameter)
        {
          return device_list.pDeviceInfo[i];
        }
      }
    }
  }
  else if (_type == "ID")
  {
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++)
    {
      if ((device_list.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE))
      {
        unsigned int id = device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.nDeviceNumber;
        std::string id_str = std::to_string(id);
        if (id_str == _parameter)
        {
          return device_list.pDeviceInfo[i];
        }
      } 
    }
  }
  
  return nullptr;
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  this->declare_parameter("type", "NULL");
  this->declare_parameter("parameter", "NULL");
  this->declare_parameter("retry",-1);
  this->declare_parameter("latency",1);

  this->get_parameter("type", this->type);
  this->get_parameter("parameter", this->parameter);
  MV_CC_DEVICE_INFO_LIST device_list;

  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (MV_OK != nRet)
  {
    RCLCPP_ERROR(this->get_logger(), "Enum Devices fail! nRet [0x%x]", nRet);
    rclcpp::shutdown();
  }

  MV_CC_DEVICE_INFO* selected_camera = this->detect_camera(device_list, this->type, this->parameter);
}

}// namespace hikrobot_camera
