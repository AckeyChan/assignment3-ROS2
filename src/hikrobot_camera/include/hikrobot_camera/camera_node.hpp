#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hikrobot_camera
{

/// 相机的静态信息
struct DeviceIdentity
{
  std::string transport;      ///< 传输层类型："GigE" / "USB" / "Other"
  std::string model;          ///< 相机型号
  std::string serial_number;  ///< 序列号（GigE 与 USB 相机均可用）
  std::string ip;             ///< 点分十进制 IP 地址，仅 GigE 相机有效
};

/// 海康机器人 MVS 相机的 ROS 2 封装：
///  - 发现并按 IP / 序列号选择相机；
///  - 以 sensor_msgs/msg/Image 发布图像（话题可配置）；
///  - 通过 ROS 2 参数动态读写曝光、增益、帧率、像素格式；
///  - 断线自动重连并恢复配置；退出时释放全部设备资源。
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  // ------------------------------------------------------------------
  // 连接与生命周期
  // 说明：名称以 _locked 结尾的方法要求调用方已持有 device_mutex_。
  // ------------------------------------------------------------------
  void declare_parameters();
  void load_parameters();

  bool initialize_sdk();
  bool enumerate_devices(MV_CC_DEVICE_INFO_LIST & device_list, std::string & reason) const;
  const MV_CC_DEVICE_INFO * select_device(
    const MV_CC_DEVICE_INFO_LIST & device_list, DeviceIdentity & identity, std::string & reason) const;

  bool try_connect();
  void release_device_locked();
  void configure_device_locked();
  void handle_lost_connection(const std::string & why, int error_code);

  void grab_thread_main();
  void reconnect_until_connected();
  void interruptible_sleep(int milliseconds) const;

  // ------------------------------------------------------------------
  // 图像获取与发布
  // ------------------------------------------------------------------
  void process_and_publish(MV_FRAME_OUT & frame);
  void fill_stamp(builtin_interfaces::msg::Time & stamp, const MV_FRAME_OUT_INFO_EX & info);
  void log_stats();

  // ------------------------------------------------------------------
  // 参数校验与下发（_locked 后缀同上）
  // ------------------------------------------------------------------
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool apply_auto_mode_locked(
    const char * node_name, const std::string & mode, std::string & canonical, std::string & reason);
  bool ensure_manual_mode_locked(const char * auto_node_name, bool & changed, std::string & reason);
  bool apply_exposure_locked(double microseconds, bool & auto_changed, std::string & reason);
  bool apply_gain_locked(double db, bool & auto_changed, std::string & reason);
  bool apply_frame_rate_locked(double fps, std::string & reason);
  bool apply_pixel_format_locked(const std::string & format, std::string & reason);

  bool set_float_property_locked(
    const char * node_name, float value, float & actual, std::string & reason) const;
  bool set_enum_property_locked(
    const char * node_name, const std::string & value, std::string & reason) const;
  bool get_float_range_locked(
    const char * node_name, MVCC_FLOATVALUE & range, std::string & reason) const;
  bool get_enum_symbols_locked(
    const char * node_name, std::vector<std::string> & symbols, std::string & reason) const;
  bool get_enum_symbol_locked(
    const char * node_name, unsigned int value, std::string & symbol, std::string & reason) const;

  /// 在参数回调中无法直接调用 set_parameter（会递归），改为登记到待同步表，
  /// 由 parameter_sync_timer_ 在回调之外把 ROS 参数值修正为设备实际状态。
  void schedule_parameter_sync(const std::string & name, const std::string & value);
  void flush_parameter_sync();

  // ------------------------------------------------------------------
  // 日志辅助
  // ------------------------------------------------------------------
  void log_device_list(const MV_CC_DEVICE_INFO_LIST & device_list) const;

  static void __stdcall exception_callback(unsigned int message, void * user_data);
  void on_device_exception(unsigned int message);

  // ------------------------------------------------------------------
  // 相机"期望配置"（device_mutex_ 保护）
  // ------------------------------------------------------------------
  std::string type_;        ///< "IP" / "SN" / "ID"（同 SN）/ ""（自动）
  std::string identifier_;  ///< IP 地址或序列号；为空时自动选择第一个设备
  int64_t retry_count_{3};  ///< 单次连接各步骤的重试次数
  int64_t latency_ms_{100};  ///< 重试间隔 [ms]
  int64_t reconnect_interval_ms_{1000};  ///< 断线重连间隔 [ms]

  std::string pixel_format_;           ///< 目标像素格式；为空表示保持相机当前设置
  std::string exposure_auto_{"Off"};   ///< ExposureAuto："Off" / "Once" / "Continuous"
  std::string gain_auto_{"Off"};       ///< GainAuto
  double exposure_time_us_{-1.0};      ///< 曝光时间 [us]；< 0 表示不修改
  double gain_db_{-1.0};               ///< 增益 [dB]；< 0 表示不修改
  double frame_rate_fps_{-1.0};        ///< 目标帧率 [fps]；< 0 表示不修改

  // ------------------------------------------------------------------
  // 发布相关配置（config_mutex_ 保护，部分支持运行时修改）
  // ------------------------------------------------------------------
  mutable std::mutex config_mutex_;
  std::string image_topic_;
  std::string frame_id_;
  bool use_device_timestamp_{false};
  bool reliable_qos_{true};
  int64_t qos_depth_{10};
  double stats_interval_s_{5.0};

  // ------------------------------------------------------------------
  // 运行时状态
  // ------------------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  rclcpp::TimerBase::SharedPtr parameter_sync_timer_;

  mutable std::mutex device_mutex_;  ///< 保护 handle_ 生命周期与上面的"期望配置"
  void * handle_{nullptr};
  bool grabbing_{false};
  bool connected_{false};
  bool sdk_ready_{false};
  DeviceIdentity identity_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> disconnect_detected_{false};
  std::thread grab_thread_;

  // 采集线程私有状态（仅在 grab_thread_main / process_and_publish 中访问）
  std::vector<uint8_t> convert_buffer_;
  int last_warned_pixel_type_{0};
  bool has_warned_pixel_type_{false};
  uint64_t stats_frames_{0};
  uint64_t stats_published_{0};
  uint64_t stats_skipped_{0};
  uint64_t stats_timeouts_{0};
  uint64_t stats_lost_packets_{0};
  std::chrono::steady_clock::time_point stats_window_start_;
  std::string last_logged_encoding_;

  // 参数待同步表（sync_mutex_ 保护）
  std::mutex sync_mutex_;
  std::map<std::string, std::string> pending_sync_;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
