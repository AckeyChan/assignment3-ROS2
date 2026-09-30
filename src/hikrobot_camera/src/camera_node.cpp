#include "hikrobot_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"


namespace hikrobot_camera
{
  bool CheckError(int nRet, const std::string& function_name)
{
  if (nRet != MV_OK)
  {
    char error_msg[256];
    RCLCPP_ERROR(rclcpp::get_logger("hikrobot_camera"), "%s failed! nRet [0x%x]", function_name.c_str(), nRet);
    MV_CC_GetErrorMsg(nRet, error_msg, sizeof(error_msg));
    RCLCPP_ERROR(rclcpp::get_logger("hikrobot_camera"), "Error message: %s", error_msg);
    return false;
  }
  return true;
}
  void* WorkThread(void* handle)
{
    int nRet = MV_OK;
    MV_FRAME_OUT stOutFrame = {0} ;
    while(true)
    {
        nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
        if (nRet == MV_OK)
        {
            printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d]\n",
            stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);
            nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
            CheckError(nRet, "MV_CC_FreeImageBuffer");
        }
        if(_exit)
        {
            break;
        }
    }
    return NULL;
}

//抓取相机并且返回指针
MV_CC_DEVICE_INFO* CameraNode::detect_camera(MV_CC_DEVICE_INFO_LIST device_list, std::string _type, std::string _parameter)
{
  if (_type == "NULL" || _parameter == "NULL")
  {
    RCLCPP_ERROR(this->get_logger(), "Type or parameter is NULL!");
    return 0;
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
  RCLCPP_ERROR(this->get_logger(), "No camera found with the specified type and parameter.");
  return 0;
}

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{//参数声明和获取
  this->declare_parameter("type", "NULL");
  this->declare_parameter("parameter", "NULL");
  this->declare_parameter("retry_count", 0);
  this->declare_parameter("latency", 1);

  this->get_parameter("type", this->type);
  this->get_parameter("parameter", this->parameter);
  this->get_parameter("retry_count", this->retry_count);
  this->get_parameter("latency", this->latency);
  MV_CC_DEVICE_INFO_LIST device_list;
//枚举设备
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (MV_OK != nRet)
  {
    RCLCPP_ERROR(this->get_logger(), "Enum Devices fail! nRet [0x%x]", nRet);
    rclcpp::shutdown();
    return ;
  }
//重试机制
  MV_CC_DEVICE_INFO* selected_camera = nullptr;
  for (int i = 0; i <= this->retry_count; i++){
    selected_camera = this->detect_camera(device_list, this->type, this->parameter);
    if (selected_camera == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Camera not found. Retrying... (%d/%d)", i + 1, this->retry_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(this->latency));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Camera found and selected successfully.");
        break;
      }
  }
  //重试结果
  if (selected_camera == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to find the camera after %d retries.", this->retry_count);
      rclcpp::shutdown();
      return ;
    }

//处理相机
//对于每一处需要与相机交互的地方，都使用重试机制来确保操作成功



int Rnet;
int counter = 0;//重试计数器
void* handle = nullptr;
do{
  if (counter > 0  and counter < this->retry_count)
  {
    RCLCPP_WARN(this->get_logger(), "Retrying to create handle... (%d/%d)", counter, this->retry_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->latency));
  }
  Rnet = MV_CC_CreateHandle(&this->handle, selected_camera);
  counter++;
}while(!CheckError(Rnet, "MV_CC_CreateHandle") && counter < this->retry_count);
if (counter >= this->retry_count && Rnet != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create handle after %d retries.", this->retry_count);
    rclcpp::shutdown();
    return ;
  }



counter = 0;//重试计数器
do{
  if (counter > 0  and counter < this->retry_count)
  {
    RCLCPP_WARN(this->get_logger(), "Retrying to open device... (%d/%d)", counter, this->retry_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->latency));
  }
  Rnet = MV_CC_OpenDevice(this->handle);
  counter++;
}while(!CheckError(Rnet, "MV_CC_OpenDevice") && counter < this->retry_count);
if (counter >= this->retry_count && Rnet != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device after %d retries.", this->retry_count);
    rclcpp::shutdown();
    return ;
  }

  counter = 0;//重试计数器
do{
  if (counter > 0  and counter < this->retry_count)
  {
    RCLCPP_WARN(this->get_logger(), "Retrying to set image node number... (%d/%d)", counter, this->retry_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->latency));
  }
  Rnet = MV_CC_SetImageNodeNum(this->handle, 3);
  counter++;
}while(!CheckError(Rnet, "MV_CC_SetImageNodeNum") && counter < this->retry_count);
if (counter >= this->retry_count && Rnet != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set image node number after %d retries.", this->retry_count);
    MV_CC_CloseDevice(this->handle);
    MV_CC_DestroyHandle(this->handle);
    rclcpp::shutdown();
    return ;
  }

counter = 0;//重试计数器
do{
  if (counter > 0  and counter < this->retry_count)
  {
    RCLCPP_WARN(this->get_logger(), "Retrying to start grabbing... (%d/%d)", counter, this->retry_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->latency));
  }
  Rnet = MV_CC_StartGrabbing(this->handle);
  counter++;
}while(!CheckError(Rnet, "MV_CC_StartGrabbing") && counter < this->retry_count);
if (counter >= this->retry_count && Rnet != MV_OK)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing after %d retries.", this->retry_count);
    MV_CC_CloseDevice(this->handle);
    MV_CC_DestroyHandle(this->handle);
    rclcpp::shutdown();
    return ;
  }




pthread_t hThreadHandle = 0;
int arg = 42;
int ret = pthread_create(&hThreadHandle, NULL, WorkThread, arg);
if (0 == hThreadHandle)
{
    RCLCPP_ERROR(this->get_logger(), "Create work thread failed!");
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    rclcpp::shutdown();
    return ;
}
pthread_join(hThreadHandle, NULL);



}
}
