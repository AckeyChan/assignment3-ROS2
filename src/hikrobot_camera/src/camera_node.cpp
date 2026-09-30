
//枚举设备 → 按 IP / 序列号选择目标相机 → 创建句柄 → 打开设备 → 应用配置 → 开始取流；
//采集线程循环取帧，转换为 sensor_msgs/msg::Image 发布；
// 取流错误或 SDK 异常回调检测到断线后，释放设备资源并自动重连，重连成功后恢复配置；
//参数回调负责校验并下发曝光、增益、帧率、像素格式，失败时返回明确原因。

#include "hikrobot_camera/camera_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hikrobot_camera
{
namespace
{

/// 取流超时 [ms]。缩短该值可以让节点更快响应退出与断线，同时不影响正常出流。
constexpr unsigned int kGrabTimeoutMs = 200;

/// 连接/重连过程中的日志节流间隔 [ms]，避免频繁重试刷屏。
constexpr int kRetryLogThrottleMs = 5000;

/// 设备列表日志的节流间隔 [ms]。
constexpr int kDeviceListLogThrottleMs = 30000;

/// 属性回读判定：相对误差 5% 或绝对误差 1.0 以内视为设置成功。
constexpr double kReadBackToleranceRatio = 0.05;
constexpr double kReadBackToleranceAbsolute = 1.0;

/// 把任意 int64 毫秒值转换为可安全传入 sleep 的 int。
int to_sleep_ms(int64_t milliseconds)
{
  if (milliseconds <= 0)
  {
    return 0;
  }
  constexpr int64_t kMaxSleepMs = 600000;  // 最长 10 分钟
  if (milliseconds > kMaxSleepMs)
  {
    return static_cast<int>(kMaxSleepMs);
  }
  return static_cast<int>(milliseconds);
}

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
  {
    return std::string();
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string to_upper(const std::string & text)
{
  std::string result = text;
  std::transform(result.begin(), result.end(), result.begin(),
    [](unsigned char character) { return static_cast<char>(std::toupper(character)); });
  return result;
}

bool equals_ignore_case(const std::string & left, const std::string & right)
{
  return to_upper(left) == to_upper(right);
}

std::string find_symbol_ignore_case(const std::vector<std::string> & symbols, const std::string & value)
{
  for (const auto & symbol : symbols)
  {
    if (equals_ignore_case(symbol, value))
    {
      return symbol;
    }
  }
  return std::string();
}

std::string join_symbols(const std::vector<std::string> & symbols)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < symbols.size(); ++index)
  {
    if (index > 0)
    {
      stream << ", ";
    }
    stream << symbols[index];
  }
  return stream.str();
}

std::string format_number(double value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

std::string ip_to_string(unsigned int ip)
{
  std::ostringstream stream;
  stream << ((ip >> 24) & 0xFFU) << '.' << ((ip >> 16) & 0xFFU) << '.'
         << ((ip >> 8) & 0xFFU) << '.' << (ip & 0xFFU);
  return stream.str();
}

template<std::size_t N>
std::string array_to_string(const unsigned char (&data)[N])
{
  const char * text = reinterpret_cast<const char *>(data);
  return std::string(text, strnlen(text, N));
}

/// MVS 错误码的简短中文说明。
const char * error_text(int error_code)
{
  switch (static_cast<unsigned int>(error_code))
  {
    case MV_OK: return "成功";
    case MV_E_HANDLE: return "错误或无效的句柄";
    case MV_E_SUPPORT: return "设备不支持该功能";
    case MV_E_BUFOVER: return "缓存已满";
    case MV_E_CALLORDER: return "函数调用顺序错误";
    case MV_E_PARAMETER: return "参数错误";
    case MV_E_RESOURCE: return "资源申请失败";
    case MV_E_NODATA: return "无数据（超时）";
    case MV_E_PRECONDITION: return "前置条件有误或运行环境已变化";
    case MV_E_VERSION: return "版本不匹配";
    case MV_E_NOENOUGH_BUF: return "缓存空间不足";
    case MV_E_ABNORMAL_IMAGE: return "异常图像（可能由丢包导致）";
    case MV_E_LOAD_LIBRARY: return "动态库加载失败";
    case MV_E_NOOUTBUF: return "没有可输出的缓存";
    case MV_E_GC_GENERIC: return "GenICam 通用错误";
    case MV_E_GC_ARGUMENT: return "参数非法";
    case MV_E_GC_RANGE: return "值超出范围";
    case MV_E_GC_PROPERTY: return "属性错误";
    case MV_E_GC_RUNTIME: return "运行环境错误";
    case MV_E_GC_LOGICAL: return "逻辑错误";
    case MV_E_GC_ACCESS: return "节点访问条件不满足";
    case MV_E_GC_TIMEOUT: return "访问超时";
    case MV_E_GC_NODE_NOT_FOUND: return "节点不存在（相机不支持该功能）";
    case MV_E_NOT_IMPLEMENTED: return "设备不支持该命令";
    case MV_E_INVALID_ADDRESS: return "访问的目标地址不存在";
    case MV_E_WRITE_PROTECT: return "目标地址不可写";
    case MV_E_ACCESS_DENIED: return "设备无访问权限（可能被其他程序占用）";
    case MV_E_BUSY: return "设备忙或网络断开";
    case MV_E_PACKET: return "网络包数据错误";
    case MV_E_NETER: return "网络错误";
    case MV_E_TIMEOUT: return "超时";
    case MV_E_DEV_DISCONNECT: return "设备已断开连接";
    default: return "未知错误";
  }
}

std::string format_ret(int ret)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%08x", static_cast<unsigned int>(ret));
  return std::string(buffer) + " (" + error_text(ret) + ")";
}

/// 连接类错误的排查提示。
std::string connection_error_hint(int error_code)
{
  switch (static_cast<unsigned int>(error_code))
  {
    case MV_E_ACCESS_DENIED:
      return "，设备可能已被其他程序（如 MVS 客户端）占用，或当前用户权限不足";
    case MV_E_BUSY:
      return "，设备忙或网络连接异常";
    case MV_E_DEV_DISCONNECT:
      return "，设备在连接过程中断开";
    case MV_E_HANDLE:
      return "，句柄无效（设备信息可能已过期，将重新枚举）";
    default:
      return "";
  }
}

/// 设备信息 → 可读描述。
DeviceIdentity describe_device(const MV_CC_DEVICE_INFO * info)
{
  DeviceIdentity identity;
  identity.transport = "Unknown";
  if (info == nullptr)
  {
    return identity;
  }

  if (info->nTLayerType == MV_GIGE_DEVICE)
  {
    identity.transport = "GigE";
    identity.model = array_to_string(info->SpecialInfo.stGigEInfo.chModelName);
    identity.serial_number = array_to_string(info->SpecialInfo.stGigEInfo.chSerialNumber);
    identity.ip = ip_to_string(info->SpecialInfo.stGigEInfo.nCurrentIp);
  }
  else if (info->nTLayerType == MV_USB_DEVICE)
  {
    identity.transport = "USB";
    identity.model = array_to_string(info->SpecialInfo.stUsb3VInfo.chModelName);
    identity.serial_number = array_to_string(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  else
  {
    std::ostringstream stream;
    stream << "TL=0x" << std::hex << info->nTLayerType;
    identity.transport = stream.str();
  }
  return identity;
}

std::string describe_identity(const DeviceIdentity & identity)
{
  std::ostringstream stream;
  stream << identity.transport
         << " 型号=" << (identity.model.empty() ? "未知" : identity.model)
         << " SN=" << (identity.serial_number.empty() ? "未知" : identity.serial_number);
  if (!identity.ip.empty())
  {
    stream << " IP=" << identity.ip;
  }
  return stream.str();
}

/// 设备像素格式 → ROS 图像编码的映射。
/// 返回 false 表示该格式当前不支持发布（相关帧会被跳过并输出一次警告）。
/// 对于 ROS 显示端支持不佳的格式（如 Bayer、YUV），会经 SDK 转换为 RGB8 / Mono16。
bool map_pixel_format(
  MvGvspPixelType source, std::string & encoding, bool & need_convert,
  MvGvspPixelType & target, unsigned int & target_bpp)
{
  need_convert = false;
  target = PixelType_Gvsp_Undefined;
  target_bpp = 0;

  switch (source)
  {
    case PixelType_Gvsp_Mono8:
      encoding = "mono8";
      target_bpp = 1;
      return true;
    case PixelType_Gvsp_Mono16:
      encoding = "mono16";
      target_bpp = 2;
      return true;
    case PixelType_Gvsp_RGB8_Packed:
      encoding = "rgb8";
      target_bpp = 3;
      return true;
    case PixelType_Gvsp_BGR8_Packed:
      encoding = "bgr8";
      target_bpp = 3;
      return true;
    case PixelType_Gvsp_RGBA8_Packed:
      encoding = "rgba8";
      target_bpp = 4;
      return true;
    case PixelType_Gvsp_BGRA8_Packed:
      encoding = "bgra8";
      target_bpp = 4;
      return true;
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
      // Bayer 原始数据在 RViz 等显示端支持不佳，统一转换为 RGB8
      encoding = "rgb8";
      need_convert = true;
      target = PixelType_Gvsp_RGB8_Packed;
      target_bpp = 3;
      return true;
    case PixelType_Gvsp_YUV422_Packed:
      encoding = "rgb8";
      need_convert = true;
      target = PixelType_Gvsp_RGB8_Packed;
      target_bpp = 3;
      return true;
    case PixelType_Gvsp_Mono10:
    case PixelType_Gvsp_Mono12:
    case PixelType_Gvsp_Mono10_Packed:
    case PixelType_Gvsp_Mono12_Packed:
      encoding = "mono16";
      need_convert = true;
      target = PixelType_Gvsp_Mono16;
      target_bpp = 2;
      return true;
    default:
      encoding.clear();
      return false;
  }
}

}  // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  declare_parameters();
  load_parameters();

  // 图像发布者：默认 reliable，保证与 rviz2 / ros2 topic echo 等默认订阅端兼容；
  // 高帧率场景可通过 reliable_qos:=false 切换为 best effort 以降低传输开销。
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(std::max<int64_t>(qos_depth_, 1))));
  if (reliable_qos_)
  {
    qos.reliable();
  }
  else
  {
    qos.best_effort();
  }
  qos.durability_volatile();
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
  RCLCPP_INFO(get_logger(), "图像发布话题: '%s'（%s，depth=%lld，frame_id='%s'）",
    image_topic_.c_str(), reliable_qos_ ? "reliable" : "best_effort",
    static_cast<long long>(qos_depth_), frame_id_.c_str());

  parameters_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {
      return on_set_parameters(parameters);
    });

  parameter_sync_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { flush_parameter_sync(); });

  if (!initialize_sdk())
  {
    RCLCPP_ERROR(get_logger(), "MVS SDK 初始化失败，相机功能不可用。");
    return;
  }

  if (!try_connect())
  {
    RCLCPP_WARN(get_logger(),
      "首次连接相机失败：节点将保持运行，并按 %lld ms 的间隔自动重连（等待相机上电/插拔）。",
      static_cast<long long>(reconnect_interval_ms_));
  }

  grab_thread_ = std::thread(&CameraNode::grab_thread_main, this);
  RCLCPP_INFO(get_logger(), "节点初始化完成。");
}

CameraNode::~CameraNode()
{
  stop_requested_.store(true);
  if (grab_thread_.joinable())
  {
    grab_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    release_device_locked();
  }

  if (sdk_ready_)
  {
    const int ret = MV_CC_Finalize();
    if (ret != MV_OK)
    {
      RCLCPP_WARN(get_logger(), "MV_CC_Finalize 返回 %s。", format_ret(ret).c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "节点已退出，相机资源已释放。");
}

// ============================================================================
// 参数声明与读取
// ============================================================================

void CameraNode::declare_parameters()
{
  auto descriptor = [](const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    parameter_descriptor.description = description;
    return parameter_descriptor;
  };

  try
  {
    declare_parameter("type", "SN", descriptor(
      "设备选择方式：IP=按 IP 选择（仅网口相机）；SN / ID=按序列号选择；空字符串=自动（先按序列号，再按 IP；parameter 也为空时选择第一个设备）"));
    declare_parameter("parameter", "", descriptor(
      "目标设备标识：type=IP 时为相机 IP（如 192.168.1.10）；type=SN / ID 时为相机序列号"));
    declare_parameter("retry_count", 3, descriptor("连接过程中各步骤的最大重试次数（不含首次尝试）"));
    declare_parameter("latency", 100, descriptor("连接重试间隔 [ms]"));
    declare_parameter("reconnect_interval_ms", 1000, descriptor("断线后自动重连的间隔 [ms]"));

    declare_parameter("image_topic", "image_raw", descriptor(
      "图像发布话题名（启动时生效，运行时修改会被拒绝）"));
    declare_parameter("frame_id", "camera", descriptor("图像消息 header.frame_id"));
    declare_parameter("use_device_timestamp", false, descriptor(
      "true=使用相机主机时间戳（nHostTimeStamp）；false=使用 ROS 节点接收时刻"));
    declare_parameter("reliable_qos", true, descriptor(
      "图像话题 QoS 可靠性：true=reliable（与 rviz2 等默认订阅端兼容）；false=best_effort（高帧率时开销更小）"));
    declare_parameter("qos_depth", 10, descriptor("图像话题队列深度（启动时生效）"));
    declare_parameter("stats_interval_s", 5.0, descriptor(
      "帧率统计日志间隔 [s]，<=0 表示关闭；日志会同时给出设置帧率与实测接收帧率"));

    declare_parameter("pixel_format", "", descriptor(
      "目标像素格式（如 Mono8 / BayerRG8 / RGB8Packed），空字符串=保持相机当前设置"));
    declare_parameter("exposure_auto", "Off", descriptor(
      "自动曝光模式：Off / Once / Continuous"));
    declare_parameter("gain_auto", "Off", descriptor(
      "自动增益模式：Off / Once / Continuous"));
    declare_parameter("exposure_time", -1.0, descriptor(
      "曝光时间 [us]，<0 表示不修改；手动设置时节点会自动将 ExposureAuto 置为 Off"));
    declare_parameter("gain", -1.0, descriptor(
      "增益 [dB]，<0 表示不修改；手动设置时节点会自动将 GainAuto 置为 Off"));
    declare_parameter("frame_rate", -1.0, descriptor(
      "目标采集帧率 [fps]，<0 表示不修改；设置时会自动置 AcquisitionFrameRateEnable=true"));
  }
  catch (const std::exception & exception)
  {
    RCLCPP_FATAL(get_logger(),
      "参数声明失败，请检查参数文件中的类型是否匹配（浮点参数需写成小数，如 5000.0）：%s",
      exception.what());
    throw;
  }
}

void CameraNode::load_parameters()
{
  type_ = trim(get_parameter("type").as_string());
  identifier_ = trim(get_parameter("parameter").as_string());
  retry_count_ = get_parameter("retry_count").as_int();
  latency_ms_ = get_parameter("latency").as_int();
  reconnect_interval_ms_ = get_parameter("reconnect_interval_ms").as_int();

  image_topic_ = trim(get_parameter("image_topic").as_string());
  frame_id_ = get_parameter("frame_id").as_string();
  use_device_timestamp_ = get_parameter("use_device_timestamp").as_bool();
  reliable_qos_ = get_parameter("reliable_qos").as_bool();
  qos_depth_ = get_parameter("qos_depth").as_int();
  stats_interval_s_ = get_parameter("stats_interval_s").as_double();

  pixel_format_ = trim(get_parameter("pixel_format").as_string());
  exposure_auto_ = trim(get_parameter("exposure_auto").as_string());
  gain_auto_ = trim(get_parameter("gain_auto").as_string());
  exposure_time_us_ = get_parameter("exposure_time").as_double();
  gain_db_ = get_parameter("gain").as_double();
  frame_rate_fps_ = get_parameter("frame_rate").as_double();

  if (image_topic_.empty())
  {
    image_topic_ = "image_raw";
    RCLCPP_WARN(get_logger(), "image_topic 为空，已回退为默认值 'image_raw'。");
  }
  if (qos_depth_ < 1)
  {
    qos_depth_ = 1;
  }
  if (retry_count_ < 0)
  {
    RCLCPP_WARN(get_logger(), "retry_count=%lld 无效，已按 0 处理。", static_cast<long long>(retry_count_));
    retry_count_ = 0;
  }
  if (latency_ms_ <= 0)
  {
    RCLCPP_WARN(get_logger(), "latency=%lld 无效，已按 100 ms 处理。", static_cast<long long>(latency_ms_));
    latency_ms_ = 100;
  }
  if (reconnect_interval_ms_ <= 0)
  {
    RCLCPP_WARN(get_logger(), "reconnect_interval_ms=%lld 无效，已按 1000 ms 处理。",
      static_cast<long long>(reconnect_interval_ms_));
    reconnect_interval_ms_ = 1000;
  }
}

// ============================================================================
// SDK 初始化与设备发现
// ============================================================================

bool CameraNode::initialize_sdk()
{
  const int ret = MV_CC_Initialize();
  if (ret == MV_OK || static_cast<unsigned int>(ret) == MV_E_CALLORDER)
  {
    sdk_ready_ = true;
    RCLCPP_INFO(get_logger(), "MVS SDK 初始化成功（SDK 版本 0x%08x）。", MV_CC_GetSDKVersion());
    return true;
  }

  sdk_ready_ = false;
  RCLCPP_ERROR(get_logger(),
    "MVS SDK 初始化失败：%s。请确认已安装 MVS SDK（默认 /opt/MVS）并且运行用户具有访问权限。",
    format_ret(ret).c_str());
  return false;
}

bool CameraNode::enumerate_devices(MV_CC_DEVICE_INFO_LIST & device_list, std::string & reason) const
{
  std::memset(&device_list, 0, sizeof(device_list));
  const int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (ret != MV_OK)
  {
    reason = "枚举设备失败: " + format_ret(ret);
    return false;
  }
  return true;
}

void CameraNode::log_device_list(const MV_CC_DEVICE_INFO_LIST & device_list) const
{
  if (device_list.nDeviceNum == 0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kDeviceListLogThrottleMs,
      "未发现任何 GigE / USB 相机，请检查相机供电、网线 / USB 连接与网络配置。");
    return;
  }

  std::ostringstream text;
  text << "发现 " << device_list.nDeviceNum << " 个相机设备:";
  for (unsigned int index = 0; index < device_list.nDeviceNum; ++index)
  {
    const MV_CC_DEVICE_INFO * info = device_list.pDeviceInfo[index];
    if (info == nullptr)
    {
      continue;
    }
    text << "\n  [" << index << "] " << describe_identity(describe_device(info));
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kDeviceListLogThrottleMs, "%s", text.str().c_str());
}

const MV_CC_DEVICE_INFO * CameraNode::select_device(
  const MV_CC_DEVICE_INFO_LIST & device_list, DeviceIdentity & identity, std::string & reason) const
{
  if (device_list.nDeviceNum == 0)
  {
    reason = "当前没有任何在线的 GigE / USB 相机";
    return nullptr;
  }

  const std::string mode = to_upper(type_);
  if (!mode.empty() && mode != "IP" && mode != "SN" && mode != "ID")
  {
    reason = "无效的 type='" + type_ + "'，可选值：IP（网口相机 IP）、SN / ID（序列号）、空字符串（自动选择）";
    return nullptr;
  }

  if (identifier_.empty())
  {
    if (!mode.empty())
    {
      reason = "type=" + mode + " 但 parameter 为空：请填写目标相机的 IP 或序列号，"
        "或将 type 置空以自动选择第一个设备";
      return nullptr;
    }
    const MV_CC_DEVICE_INFO * first = device_list.pDeviceInfo[0];
    if (first == nullptr)
    {
      reason = "设备列表内容无效（首项为空指针）";
      return nullptr;
    }
    identity = describe_device(first);
    RCLCPP_INFO(get_logger(), "未指定设备标识（parameter 为空），自动选择第一个设备：%s",
      describe_identity(identity).c_str());
    return first;
  }

  auto matches_identifier = [this, &mode](const DeviceIdentity & candidate)
  {
    if (mode == "IP")
    {
      return candidate.transport == "GigE" && candidate.ip == identifier_;
    }
    // SN / ID 或自动模式：先按序列号匹配
    return equals_ignore_case(candidate.serial_number, identifier_);
  };

  std::vector<const MV_CC_DEVICE_INFO *> matches;
  std::vector<DeviceIdentity> matched_identities;

  for (unsigned int index = 0; index < device_list.nDeviceNum; ++index)
  {
    const MV_CC_DEVICE_INFO * info = device_list.pDeviceInfo[index];
    if (info == nullptr)
    {
      continue;
    }
    DeviceIdentity candidate = describe_device(info);
    if (matches_identifier(candidate))
    {
      matches.push_back(info);
      matched_identities.push_back(std::move(candidate));
    }
  }

  // 自动模式下序列号未匹配时，再尝试按 IP 匹配（方便只记得 IP 的场景）
  if (matches.empty() && mode.empty())
  {
    for (unsigned int index = 0; index < device_list.nDeviceNum; ++index)
    {
      const MV_CC_DEVICE_INFO * info = device_list.pDeviceInfo[index];
      if (info == nullptr)
      {
        continue;
      }
      DeviceIdentity candidate = describe_device(info);
      if (candidate.transport == "GigE" && candidate.ip == identifier_)
      {
        matches.push_back(info);
        matched_identities.push_back(std::move(candidate));
      }
    }
    if (!matches.empty())
    {
      RCLCPP_INFO(get_logger(), "按序列号未匹配到设备，改按 IP '%s' 匹配成功。", identifier_.c_str());
    }
  }

  if (matches.empty())
  {
    reason = "未找到" + std::string(mode == "IP" ? " IP" : "序列号") + "为 '" + identifier_ +
      "' 的相机（当前在线 " + std::to_string(device_list.nDeviceNum) + " 台设备）";
    return nullptr;
  }

  if (matches.size() > 1)
  {
    RCLCPP_WARN(get_logger(), "标识 '%s' 匹配到 %zu 台设备（标识冲突），本次使用第一台：%s",
      identifier_.c_str(), matches.size(), describe_identity(matched_identities.front()).c_str());
  }

  identity = matched_identities.front();
  return matches.front();
}

// ============================================================================
// 异常回调
// ============================================================================

void __stdcall CameraNode::exception_callback(unsigned int message, void * user_data)
{
  if (user_data == nullptr)
  {
    return;
  }
  static_cast<CameraNode *>(user_data)->on_device_exception(message);
}

void CameraNode::on_device_exception(unsigned int message)
{
  if (message == MV_EXCEPTION_DEV_DISCONNECT)
  {
    RCLCPP_WARN(get_logger(), "收到 SDK 异常回调：相机已断开连接。");
    disconnect_detected_.store(true);
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "收到 SDK 异常回调：type=0x%x。", message);
  }
}

// ============================================================================
// 连接 / 断开 / 重连
// ============================================================================

bool CameraNode::try_connect()
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!sdk_ready_)
  {
    return false;
  }

  MV_CC_DEVICE_INFO_LIST device_list;
  std::string reason;
  if (!enumerate_devices(device_list, reason))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs, "连接失败：%s", reason.c_str());
    return false;
  }
  log_device_list(device_list);

  DeviceIdentity identity;
  const MV_CC_DEVICE_INFO * device = select_device(device_list, identity, reason);
  if (device == nullptr)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs, "连接失败：%s", reason.c_str());
    return false;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
    "正在连接相机：%s", describe_identity(identity).c_str());

  void * handle = nullptr;
  int ret = MV_OK;

  // 1) 创建句柄
  for (int64_t attempt = 0; attempt <= retry_count_; ++attempt)
  {
    if (attempt > 0)
    {
      RCLCPP_WARN(get_logger(), "创建相机句柄失败(%s)，%lld ms 后重试（第 %lld 次）",
        format_ret(ret).c_str(), static_cast<long long>(latency_ms_), static_cast<long long>(attempt));
      interruptible_sleep(to_sleep_ms(latency_ms_));
      if (stop_requested_.load() || !rclcpp::ok())
      {
        return false;
      }
    }
    ret = MV_CC_CreateHandle(&handle, device);
    if (ret == MV_OK)
    {
      break;
    }
  }
  if (ret != MV_OK)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
      "连接失败：创建句柄失败 %s%s", format_ret(ret).c_str(), connection_error_hint(ret).c_str());
    return false;
  }

  // 2) 打开设备
  for (int64_t attempt = 0; attempt <= retry_count_; ++attempt)
  {
    if (attempt > 0)
    {
      RCLCPP_WARN(get_logger(), "打开设备失败(%s)，%lld ms 后重试（第 %lld 次）",
        format_ret(ret).c_str(), static_cast<long long>(latency_ms_), static_cast<long long>(attempt));
      interruptible_sleep(to_sleep_ms(latency_ms_));
      if (stop_requested_.load() || !rclcpp::ok())
      {
        MV_CC_DestroyHandle(handle);
        return false;
      }
    }
    ret = MV_CC_OpenDevice(handle);
    if (ret == MV_OK)
    {
      break;
    }
  }
  if (ret != MV_OK)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
      "连接失败：打开设备失败 %s%s", format_ret(ret).c_str(), connection_error_hint(ret).c_str());
    MV_CC_DestroyHandle(handle);
    return false;
  }

  handle_ = handle;
  identity_ = identity;
  grabbing_ = false;
  connected_ = false;

  // 3) GigE 相机：自动探测并设置最佳包长（受 MTU 限制），可显著改善实际帧率
  if (identity.transport == "GigE")
  {
    const int packet_size = MV_CC_GetOptimalPacketSize(handle_);
    if (packet_size > 0)
    {
      ret = MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size);
      if (ret != MV_OK)
      {
        RCLCPP_WARN(get_logger(), "设置最佳包长 GevSCPSPacketSize=%d 失败(%s)，不影响基本取流。",
          packet_size, format_ret(ret).c_str());
      }
      else
      {
        RCLCPP_INFO(get_logger(), "已设置网口相机包长 GevSCPSPacketSize=%d。", packet_size);
      }
    }
    else
    {
      RCLCPP_WARN(get_logger(), "获取最佳包长失败：%s", format_ret(packet_size).c_str());
    }
  }

  // 4) 注册异常回调（设备断开时能更快通知采集线程）
  ret = MV_CC_RegisterExceptionCallBack(handle_, &CameraNode::exception_callback, this);
  if (ret != MV_OK)
  {
    RCLCPP_WARN(get_logger(), "注册异常回调失败(%s)，仍可通过取流错误发现断线。", format_ret(ret).c_str());
  }

  // 5) 应用期望配置（像素格式、曝光、增益、帧率等）
  configure_device_locked();

  // 6) 开始取流
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
      "连接失败：开始取流失败 %s%s", format_ret(ret).c_str(), connection_error_hint(ret).c_str());
    release_device_locked();
    return false;
  }

  grabbing_ = true;
  connected_ = true;
  RCLCPP_INFO(get_logger(), "相机连接成功并开始采集：%s（图像发布话题 '%s'）",
    describe_identity(identity_).c_str(), image_topic_.c_str());
  return true;
}

void CameraNode::release_device_locked()
{
  connected_ = false;
  if (handle_ == nullptr)
  {
    grabbing_ = false;
    return;
  }

  int ret = MV_OK;
  if (grabbing_)
  {
    ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK)
    {
      RCLCPP_WARN(get_logger(), "停止取流返回 %s（断线场景下属正常）。", format_ret(ret).c_str());
    }
    grabbing_ = false;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_WARN(get_logger(), "关闭设备返回 %s（断线场景下属正常）。", format_ret(ret).c_str());
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
  {
    RCLCPP_WARN(get_logger(), "销毁句柄返回 %s。", format_ret(ret).c_str());
  }

  handle_ = nullptr;
  RCLCPP_INFO(get_logger(), "相机资源已释放。");
}

void CameraNode::configure_device_locked()
{
  if (handle_ == nullptr)
  {
    return;
  }

  std::string reason;
  int ret = MV_OK;

  // 关闭触发模式，保证连续自由采集
  ret = MV_CC_SetEnumValue(handle_, "TriggerMode", static_cast<unsigned int>(MV_TRIGGER_MODE_OFF));
  if (ret != MV_OK)
  {
    RCLCPP_WARN(get_logger(),
      "设置 TriggerMode=Off 失败(%s)：若相机当前处于触发模式，将无法连续出图。", format_ret(ret).c_str());
  }

  // 增大内部图像缓存节点数，降低高帧率下丢帧概率
  ret = MV_CC_SetImageNodeNum(handle_, 3);
  if (ret != MV_OK)
  {
    RCLCPP_DEBUG(get_logger(), "设置图像缓存节点数失败：%s", format_ret(ret).c_str());
  }

  // 像素格式
  if (!pixel_format_.empty() && !apply_pixel_format_locked(pixel_format_, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用像素格式 '%s' 失败：%s", pixel_format_.c_str(), reason.c_str());
  }

  // 曝光：先下发手动值（会自动关闭自动曝光），再应用自动模式设置
  bool auto_changed = false;
  if (exposure_time_us_ >= 0.0 && !apply_exposure_locked(exposure_time_us_, auto_changed, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用曝光时间 %s us 失败：%s",
      format_number(exposure_time_us_).c_str(), reason.c_str());
  }

  // 增益
  if (gain_db_ >= 0.0 && !apply_gain_locked(gain_db_, auto_changed, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用增益 %s dB 失败：%s", format_number(gain_db_).c_str(), reason.c_str());
  }

  // 自动模式（顺序在手动值之后，保证“期望状态”最终生效）
  std::string canonical;
  if (!apply_auto_mode_locked("ExposureAuto", exposure_auto_, canonical, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用曝光模式 '%s' 失败：%s", exposure_auto_.c_str(), reason.c_str());
  }
  else
  {
    exposure_auto_ = canonical;
  }

  if (!apply_auto_mode_locked("GainAuto", gain_auto_, canonical, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用增益模式 '%s' 失败：%s", gain_auto_.c_str(), reason.c_str());
  }
  else
  {
    gain_auto_ = canonical;
  }

  // 帧率
  if (frame_rate_fps_ >= 0.0 && !apply_frame_rate_locked(frame_rate_fps_, reason))
  {
    RCLCPP_ERROR(get_logger(), "应用帧率 %s fps 失败：%s",
      format_number(frame_rate_fps_).c_str(), reason.c_str());
  }
}

void CameraNode::handle_lost_connection(const std::string & why, int error_code)
{
  RCLCPP_WARN(get_logger(), "与相机的连接中断：%s（%s），正在释放设备资源并准备重连。",
    why.c_str(), format_ret(error_code).c_str());

  std::lock_guard<std::mutex> lock(device_mutex_);
  release_device_locked();
}

// ============================================================================
// 采集线程
// ============================================================================

void CameraNode::grab_thread_main()
{
  RCLCPP_INFO(get_logger(), "采集线程已启动。");
  stats_window_start_ = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !stop_requested_.load())
  {
    if (disconnect_detected_.exchange(false))
    {
      handle_lost_connection("收到设备断开异常的 SDK 回调", MV_E_DEV_DISCONNECT);
    }

    if (!connected_)
    {
      reconnect_until_connected();
      continue;
    }

    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    const int ret = MV_CC_GetImageBuffer(handle_, &frame, kGrabTimeoutMs);
    const unsigned int code = static_cast<unsigned int>(ret);

    if (code == MV_OK)
    {
      ++stats_frames_;
      process_and_publish(frame);
    }
    else if (code == MV_E_NODATA)
    {
      ++stats_timeouts_;
    }
    else if (code == MV_E_ABNORMAL_IMAGE)
    {
      ++stats_skipped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
        "接收到不完整图像（可能由丢包导致），已丢弃该帧。");
    }
    else if (rclcpp::ok() && !stop_requested_.load())
    {
      handle_lost_connection("取流失败", ret);
    }

    // 只要句柄仍有效且 SDK 返回了缓存指针，就需要成对释放（与取流结果无关）
    if ((code == MV_OK || code == MV_E_NODATA || code == MV_E_ABNORMAL_IMAGE) && frame.pBufAddr != nullptr)
    {
      const int free_ret = MV_CC_FreeImageBuffer(handle_, &frame);
      if (free_ret != MV_OK)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
          "释放图像缓存失败：%s", format_ret(free_ret).c_str());
      }
    }

    log_stats();
  }

  RCLCPP_INFO(get_logger(), "采集线程已退出。");
}

void CameraNode::reconnect_until_connected()
{
  while (!stop_requested_.load() && rclcpp::ok() && !connected_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
      "相机未连接：正在重试连接（间隔 %lld ms）...", static_cast<long long>(reconnect_interval_ms_));

    if (try_connect())
    {
      RCLCPP_INFO(get_logger(), "相机重连成功，配置已恢复，继续采集。");
      return;
    }

    interruptible_sleep(to_sleep_ms(reconnect_interval_ms_));
  }
}

void CameraNode::interruptible_sleep(int milliseconds) const
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
  while (!stop_requested_.load() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// ============================================================================
// 图像处理与发布
// ============================================================================

void CameraNode::process_and_publish(MV_FRAME_OUT & frame)
{
  const MV_FRAME_OUT_INFO_EX & info = frame.stFrameInfo;
  const uint32_t width = info.nExtendWidth > 0 ? info.nExtendWidth : info.nWidth;
  const uint32_t height = info.nExtendHeight > 0 ? info.nExtendHeight : info.nHeight;

  if (frame.pBufAddr == nullptr || width == 0 || height == 0)
  {
    ++stats_skipped_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
      "收到无效帧（宽=%u，高=%u），已跳过。", width, height);
    return;
  }

  std::string encoding;
  bool need_convert = false;
  MvGvspPixelType target_type = PixelType_Gvsp_Undefined;
  unsigned int target_bpp = 0;
  if (!map_pixel_format(info.enPixelType, encoding, need_convert, target_type, target_bpp))
  {
    ++stats_skipped_;
    const int raw_type = static_cast<int>(info.enPixelType);
    if (!has_warned_pixel_type_ || last_warned_pixel_type_ != raw_type)
    {
      RCLCPP_WARN(get_logger(),
        "暂不支持发布的像素格式 0x%08x，相关帧将被跳过；可将 pixel_format 参数改为 Mono8 / BayerRG8 / RGB8Packed 等受支持格式。",
        static_cast<unsigned int>(info.enPixelType));
      last_warned_pixel_type_ = raw_type;
      has_warned_pixel_type_ = true;
    }
    return;
  }

  const std::size_t source_length = info.nFrameLen > 0
    ? static_cast<std::size_t>(info.nFrameLen)
    : static_cast<std::size_t>(info.nFrameLenEx);
  const std::size_t expected_length = static_cast<std::size_t>(width) * height * target_bpp;

  const uint8_t * data = nullptr;
  if (!need_convert)
  {
    if (source_length < expected_length)
    {
      ++stats_skipped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
        "帧数据长度不足（%zu < %zu 字节），已跳过该帧。", source_length, expected_length);
      return;
    }
    data = frame.pBufAddr;
  }
  else
  {
    convert_buffer_.resize(expected_length);
    MV_CC_PIXEL_CONVERT_PARAM_EX convert_param;
    std::memset(&convert_param, 0, sizeof(convert_param));
    convert_param.nWidth = width;
    convert_param.nHeight = height;
    convert_param.enSrcPixelType = info.enPixelType;
    convert_param.pSrcData = frame.pBufAddr;
    convert_param.nSrcDataLen = static_cast<unsigned int>(
      std::min<std::size_t>(source_length, std::numeric_limits<unsigned int>::max()));
    convert_param.enDstPixelType = target_type;
    convert_param.pDstBuffer = convert_buffer_.data();
    convert_param.nDstBufferSize = static_cast<unsigned int>(expected_length);

    const int ret = MV_CC_ConvertPixelTypeEx(handle_, &convert_param);
    if (ret != MV_OK)
    {
      ++stats_skipped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
        "像素格式转换失败（0x%08x -> 0x%08x）：%s，已跳过该帧。",
        static_cast<unsigned int>(info.enPixelType), static_cast<unsigned int>(target_type),
        format_ret(ret).c_str());
      return;
    }
    if (convert_param.nDstLen > expected_length)
    {
      ++stats_skipped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kRetryLogThrottleMs,
        "像素格式转换输出长度异常（%u > %zu 字节），已跳过该帧。",
        convert_param.nDstLen, expected_length);
      return;
    }
    data = convert_buffer_.data();
  }

  sensor_msgs::msg::Image message;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    message.header.frame_id = frame_id_;
  }
  fill_stamp(message.header.stamp, info);
  message.height = height;
  message.width = width;
  message.is_bigendian = false;
  message.step = static_cast<uint32_t>(static_cast<std::size_t>(width) * target_bpp);
  message.encoding = encoding;
  message.data.assign(data, data + expected_length);

  if (last_logged_encoding_ != encoding)
  {
    RCLCPP_INFO(get_logger(),
      "发布图像：%ux%u，encoding=%s，step=%u，帧数据长度=%zu 字节（设备像素格式 0x%08x）。",
      width, height, encoding.c_str(), message.step, expected_length,
      static_cast<unsigned int>(info.enPixelType));
    last_logged_encoding_ = encoding;
  }

  ++stats_published_;
  stats_lost_packets_ += info.nLostPacket;
  image_publisher_->publish(std::move(message));
}

void CameraNode::fill_stamp(builtin_interfaces::msg::Time & stamp, const MV_FRAME_OUT_INFO_EX & info)
{
  bool use_device_timestamp = false;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    use_device_timestamp = use_device_timestamp_;
  }

  // nHostTimeStamp 为相机主机时间戳，单位 ms；仅在用户显式选择时使用。
  if (use_device_timestamp && info.nHostTimeStamp > 0)
  {
    const int64_t nanoseconds = info.nHostTimeStamp * 1000000LL;
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return;
  }

  // 默认语义：ROS 节点接收到该帧的时刻。
  stamp = this->now();
}

void CameraNode::log_stats()
{
  double interval = 0.0;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    interval = stats_interval_s_;
  }
  if (interval <= 0.0)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - stats_window_start_).count();
  if (elapsed < interval)
  {
    return;
  }

  double configured_frame_rate = -1.0;
  try
  {
    configured_frame_rate = get_parameter("frame_rate").as_double();
  }
  catch (const std::exception &)
  {
    configured_frame_rate = -1.0;
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(1)
       << "帧率统计（最近 " << elapsed << " s）: 实际接收 "
       << static_cast<double>(stats_frames_) / elapsed << " fps，发布 "
       << static_cast<double>(stats_published_) / elapsed << " fps";
  if (configured_frame_rate > 0.0)
  {
    text << "；设置帧率 " << configured_frame_rate << " fps";
  }
  else
  {
    text << "；未设置帧率参数（使用相机默认）";
  }
  text << "；取流超时 " << stats_timeouts_ << " 次，跳过 " << stats_skipped_
       << " 帧，丢包 " << stats_lost_packets_ << " 个";
  RCLCPP_INFO(get_logger(), "%s", text.str().c_str());

  stats_window_start_ = now;
  stats_frames_ = 0;
  stats_published_ = 0;
  stats_skipped_ = 0;
  stats_timeouts_ = 0;
  stats_lost_packets_ = 0;
}

// ============================================================================
// 参数回调与下发
// ============================================================================

rcl_interfaces::msg::SetParametersResult CameraNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(device_mutex_);

  for (const rclcpp::Parameter & parameter : parameters)
  {
    const std::string & name = parameter.get_name();
    std::string reason;

    auto fail = [&result, &reason]()
    {
      result.successful = false;
      result.reason = reason;
      return result;
    };

    try
    {
      if (name == "exposure_time")
      {
        bool auto_changed = false;
        if (!apply_exposure_locked(parameter.as_double(), auto_changed, reason))
        {
          return fail();
        }
        if (auto_changed)
        {
          exposure_auto_ = "Off";
          schedule_parameter_sync("exposure_auto", "Off");
        }
      }
      else if (name == "gain")
      {
        bool auto_changed = false;
        if (!apply_gain_locked(parameter.as_double(), auto_changed, reason))
        {
          return fail();
        }
        if (auto_changed)
        {
          gain_auto_ = "Off";
          schedule_parameter_sync("gain_auto", "Off");
        }
      }
      else if (name == "frame_rate")
      {
        if (!apply_frame_rate_locked(parameter.as_double(), reason))
        {
          return fail();
        }
      }
      else if (name == "pixel_format")
      {
        if (!apply_pixel_format_locked(parameter.as_string(), reason))
        {
          return fail();
        }
      }
      else if (name == "exposure_auto")
      {
        std::string canonical;
        if (!apply_auto_mode_locked("ExposureAuto", parameter.as_string(), canonical, reason))
        {
          return fail();
        }
        exposure_auto_ = canonical;
      }
      else if (name == "gain_auto")
      {
        std::string canonical;
        if (!apply_auto_mode_locked("GainAuto", parameter.as_string(), canonical, reason))
        {
          return fail();
        }
        gain_auto_ = canonical;
      }
      else if (name == "frame_id")
      {
        const std::string value = parameter.as_string();
        if (value.empty())
        {
          reason = "frame_id 不能为空";
          return fail();
        }
        std::lock_guard<std::mutex> config_lock(config_mutex_);
        frame_id_ = value;
      }
      else if (name == "use_device_timestamp")
      {
        std::lock_guard<std::mutex> config_lock(config_mutex_);
        use_device_timestamp_ = parameter.as_bool();
      }
      else if (name == "stats_interval_s")
      {
        const double value = parameter.as_double();
        if (value < 0.0)
        {
          reason = "stats_interval_s 不能为负数（0 表示关闭统计日志）";
          return fail();
        }
        std::lock_guard<std::mutex> config_lock(config_mutex_);
        stats_interval_s_ = value;
      }
      else if (name == "retry_count")
      {
        const int64_t value = parameter.as_int();
        if (value < 0)
        {
          reason = "retry_count 不能为负数";
          return fail();
        }
        retry_count_ = value;
      }
      else if (name == "latency")
      {
        const int64_t value = parameter.as_int();
        if (value <= 0)
        {
          reason = "latency 必须为正数 [ms]";
          return fail();
        }
        latency_ms_ = value;
      }
      else if (name == "reconnect_interval_ms")
      {
        const int64_t value = parameter.as_int();
        if (value <= 0)
        {
          reason = "reconnect_interval_ms 必须为正数 [ms]";
          return fail();
        }
        reconnect_interval_ms_ = value;
      }
      else if (name == "type")
      {
        if (!equals_ignore_case(parameter.as_string(), type_))
        {
          reason = "type 修改需要重启节点生效（当前 '" + type_ + "'）";
          return fail();
        }
        type_ = parameter.as_string();
      }
      else if (name == "parameter")
      {
        if (parameter.as_string() != identifier_)
        {
          reason = "parameter 修改需要重启节点生效（当前 '" + identifier_ + "'）";
          return fail();
        }
      }
      else if (name == "image_topic")
      {
        if (parameter.as_string() != image_topic_)
        {
          reason = "image_topic 修改需要重启节点生效（当前 '" + image_topic_ + "'）";
          return fail();
        }
      }
      else if (name == "reliable_qos")
      {
        if (parameter.as_bool() != reliable_qos_)
        {
          reason = "reliable_qos 修改需要重启节点生效";
          return fail();
        }
      }
      else if (name == "qos_depth")
      {
        if (parameter.as_int() != qos_depth_)
        {
          reason = "qos_depth 修改需要重启节点生效";
          return fail();
        }
      }
      // 其余参数（例如 use_sim_time）不做特殊处理，直接放行
    }
    catch (const std::exception & exception)
    {
      result.successful = false;
      result.reason = std::string("参数类型错误（浮点参数请写成小数，如 5000.0）：") + exception.what();
      return result;
    }
  }

  return result;
}

bool CameraNode::apply_auto_mode_locked(
  const char * node_name, const std::string & mode, std::string & canonical, std::string & reason)
{
  canonical.clear();
  if (handle_ == nullptr)
  {
    reason = std::string("相机未连接，无法设置 ") + node_name;
    return false;
  }

  std::vector<std::string> symbols;
  if (!get_enum_symbols_locked(node_name, symbols, reason))
  {
    return false;
  }

  canonical = find_symbol_ignore_case(symbols, mode);
  if (canonical.empty())
  {
    reason = std::string(node_name) + " 不支持值 '" + mode + "'，设备支持: " + join_symbols(symbols);
    return false;
  }

  return set_enum_property_locked(node_name, canonical, reason);
}

bool CameraNode::ensure_manual_mode_locked(
  const char * auto_node_name, bool & changed, std::string & reason)
{
  changed = false;
  if (handle_ == nullptr)
  {
    reason = std::string("相机未连接，无法读取 ") + auto_node_name;
    return false;
  }

  MVCC_ENUMVALUE value;
  std::memset(&value, 0, sizeof(value));
  int ret = MV_CC_GetEnumValue(handle_, auto_node_name, &value);
  if (ret != MV_OK)
  {
    reason = std::string("读取 ") + auto_node_name + " 失败: " + format_ret(ret);
    return false;
  }

  std::string current_symbol;
  if (!get_enum_symbol_locked(auto_node_name, value.nCurValue, current_symbol, reason))
  {
    return false;
  }
  if (equals_ignore_case(current_symbol, "Off"))
  {
    return true;
  }

  ret = MV_CC_SetEnumValueByString(handle_, auto_node_name, "Off");
  if (ret != MV_OK)
  {
    reason = std::string(auto_node_name) + " 自动切换为 Off 失败: " + format_ret(ret);
    return false;
  }

  changed = true;
  RCLCPP_INFO(get_logger(), "为应用手动设置，已将 %s 由 '%s' 切换为 Off。",
    auto_node_name, current_symbol.c_str());
  return true;
}

bool CameraNode::apply_exposure_locked(double microseconds, bool & auto_changed, std::string & reason)
{
  auto_changed = false;
  if (handle_ == nullptr)
  {
    reason = "相机未连接，无法设置曝光时间";
    return false;
  }
  if (microseconds <= 0.0)
  {
    reason = "曝光时间必须为正数 [us]";
    return false;
  }

  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  if (!get_float_range_locked("ExposureTime", range, reason))
  {
    return false;
  }
  if (microseconds < static_cast<double>(range.fMin) || microseconds > static_cast<double>(range.fMax))
  {
    reason = "曝光时间 " + format_number(microseconds) + " us 超出设备范围 ["
      + format_number(range.fMin) + ", " + format_number(range.fMax) + "] us";
    return false;
  }

  if (!ensure_manual_mode_locked("ExposureAuto", auto_changed, reason))
  {
    return false;
  }

  float actual = 0.0F;
  if (!set_float_property_locked("ExposureTime", static_cast<float>(microseconds), actual, reason))
  {
    return false;
  }

  exposure_time_us_ = actual;
  RCLCPP_INFO(get_logger(), "曝光时间已设置：请求 %s us，设备回读 %s us。",
    format_number(microseconds).c_str(), format_number(actual).c_str());
  return true;
}

bool CameraNode::apply_gain_locked(double db, bool & auto_changed, std::string & reason)
{
  auto_changed = false;
  if (handle_ == nullptr)
  {
    reason = "相机未连接，无法设置增益";
    return false;
  }
  if (db < 0.0)
  {
    reason = "增益必须为非负数 [dB]";
    return false;
  }

  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  if (!get_float_range_locked("Gain", range, reason))
  {
    return false;
  }
  if (db < static_cast<double>(range.fMin) || db > static_cast<double>(range.fMax))
  {
    reason = "增益 " + format_number(db) + " dB 超出设备范围 ["
      + format_number(range.fMin) + ", " + format_number(range.fMax) + "] dB";
    return false;
  }

  if (!ensure_manual_mode_locked("GainAuto", auto_changed, reason))
  {
    return false;
  }

  float actual = 0.0F;
  if (!set_float_property_locked("Gain", static_cast<float>(db), actual, reason))
  {
    return false;
  }

  gain_db_ = actual;
  RCLCPP_INFO(get_logger(), "增益已设置：请求 %s dB，设备回读 %s dB。",
    format_number(db).c_str(), format_number(actual).c_str());
  return true;
}

bool CameraNode::apply_frame_rate_locked(double fps, std::string & reason)
{
  if (handle_ == nullptr)
  {
    reason = "相机未连接，无法设置帧率";
    return false;
  }
  if (fps <= 0.0)
  {
    reason = "帧率必须为正数 [fps]";
    return false;
  }

  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  if (!get_float_range_locked("AcquisitionFrameRate", range, reason))
  {
    return false;
  }
  if (fps < static_cast<double>(range.fMin) || fps > static_cast<double>(range.fMax))
  {
    reason = "帧率 " + format_number(fps) + " fps 超出设备范围 ["
      + format_number(range.fMin) + ", " + format_number(range.fMax) + "] fps";
    return false;
  }

  // 大多数机型需要先使能帧率控制，AcquisitionFrameRate 才会生效
  int ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  if (ret != MV_OK)
  {
    reason = "启用帧率控制（AcquisitionFrameRateEnable=true）失败: " + format_ret(ret);
    return false;
  }

  float actual = 0.0F;
  if (!set_float_property_locked("AcquisitionFrameRate", static_cast<float>(fps), actual, reason))
  {
    return false;
  }

  frame_rate_fps_ = actual;
  RCLCPP_INFO(get_logger(), "帧率已设置：请求 %s fps，设备回读 %s fps（实际接收帧率请参考周期统计日志）。",
    format_number(fps).c_str(), format_number(actual).c_str());
  return true;
}

bool CameraNode::apply_pixel_format_locked(const std::string & format, std::string & reason)
{
  if (handle_ == nullptr)
  {
    reason = "相机未连接，无法设置像素格式";
    return false;
  }
  if (format.empty())
  {
    reason = "像素格式不能为空（如需保持相机默认，请不要设置该参数）";
    return false;
  }

  std::vector<std::string> symbols;
  if (!get_enum_symbols_locked("PixelFormat", symbols, reason))
  {
    return false;
  }

  const std::string canonical = find_symbol_ignore_case(symbols, format);
  if (canonical.empty())
  {
    reason = "不支持的像素格式 '" + format + "'，设备支持: " + join_symbols(symbols);
    return false;
  }

  if (!set_enum_property_locked("PixelFormat", canonical, reason))
  {
    return false;
  }

  pixel_format_ = canonical;
  RCLCPP_INFO(get_logger(), "像素格式已设置为 %s。", canonical.c_str());
  return true;
}

// ============================================================================
// SDK 属性读写辅助
// ============================================================================

bool CameraNode::set_float_property_locked(
  const char * node_name, float value, float & actual, std::string & reason) const
{
  int ret = MV_CC_SetFloatValue(handle_, node_name, value);
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 设置失败: " + format_ret(ret);
    return false;
  }

  MVCC_FLOATVALUE read_back;
  std::memset(&read_back, 0, sizeof(read_back));
  ret = MV_CC_GetFloatValue(handle_, node_name, &read_back);
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 回读失败: " + format_ret(ret);
    return false;
  }

  actual = read_back.fCurValue;
  const double tolerance = std::max(
    kReadBackToleranceAbsolute,
    kReadBackToleranceRatio * std::fabs(static_cast<double>(value)));
  if (std::fabs(static_cast<double>(actual) - static_cast<double>(value)) > tolerance)
  {
    reason = std::string(node_name) + " 未按请求生效：请求 " + format_number(value)
      + "，设备回读 " + format_number(actual) + "（可能被设备取整或限制）";
    return false;
  }
  return true;
}

bool CameraNode::set_enum_property_locked(
  const char * node_name, const std::string & value, std::string & reason) const
{
  const int ret = MV_CC_SetEnumValueByString(handle_, node_name, value.c_str());
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 设置为 '" + value + "' 失败: " + format_ret(ret);
    return false;
  }

  MVCC_ENUMVALUE read_back;
  std::memset(&read_back, 0, sizeof(read_back));
  if (MV_CC_GetEnumValue(handle_, node_name, &read_back) != MV_OK)
  {
    reason = std::string(node_name) + " 回读失败";
    return false;
  }

  std::string symbol;
  if (!get_enum_symbol_locked(node_name, read_back.nCurValue, symbol, reason))
  {
    return false;
  }
  if (!equals_ignore_case(symbol, value))
  {
    reason = std::string(node_name) + " 回读为 '" + symbol + "'，与请求 '" + value + "' 不一致";
    return false;
  }
  return true;
}

bool CameraNode::get_float_range_locked(
  const char * node_name, MVCC_FLOATVALUE & range, std::string & reason) const
{
  if (handle_ == nullptr)
  {
    reason = "相机未连接";
    return false;
  }

  MVCC_FLOATVALUE value;
  std::memset(&value, 0, sizeof(value));
  const int ret = MV_CC_GetFloatValue(handle_, node_name, &value);
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 读取范围失败: " + format_ret(ret);
    return false;
  }

  range = value;
  return true;
}

bool CameraNode::get_enum_symbols_locked(
  const char * node_name, std::vector<std::string> & symbols, std::string & reason) const
{
  if (handle_ == nullptr)
  {
    reason = "相机未连接";
    return false;
  }

  MVCC_ENUMVALUE value;
  std::memset(&value, 0, sizeof(value));
  int ret = MV_CC_GetEnumValue(handle_, node_name, &value);
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 读取失败: " + format_ret(ret);
    return false;
  }

  symbols.clear();
  const unsigned int max_count = static_cast<unsigned int>(
    sizeof(value.nSupportValue) / sizeof(value.nSupportValue[0]));
  for (unsigned int index = 0; index < value.nSupportedNum && index < max_count; ++index)
  {
    std::string symbol;
    std::string sub_reason;
    if (get_enum_symbol_locked(node_name, value.nSupportValue[index], symbol, sub_reason))
    {
      symbols.push_back(symbol);
    }
  }

  if (symbols.empty())
  {
    reason = std::string(node_name) + " 没有可用的枚举值";
    return false;
  }
  return true;
}

bool CameraNode::get_enum_symbol_locked(
  const char * node_name, unsigned int value, std::string & symbol, std::string & reason) const
{
  if (handle_ == nullptr)
  {
    reason = "相机未连接";
    return false;
  }

  MVCC_ENUMENTRY entry;
  std::memset(&entry, 0, sizeof(entry));
  entry.nValue = value;
  const int ret = MV_CC_GetEnumEntrySymbolic(handle_, node_name, &entry);
  if (ret != MV_OK)
  {
    reason = std::string(node_name) + " 枚举值 0x" + std::to_string(value) + " 的符号读取失败: "
      + format_ret(ret);
    return false;
  }

  symbol = entry.chSymbolic;
  return true;
}

// ============================================================================
// 参数同步（回调外修正 ROS 参数值，使其与设备实际状态一致）
// ============================================================================

void CameraNode::schedule_parameter_sync(const std::string & name, const std::string & value)
{
  std::lock_guard<std::mutex> lock(sync_mutex_);
  pending_sync_[name] = value;
}

void CameraNode::flush_parameter_sync()
{
  std::map<std::string, std::string> pending;
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    if (pending_sync_.empty())
    {
      return;
    }
    pending.swap(pending_sync_);
  }

  for (const auto & entry : pending)
  {
    try
    {
      set_parameter(rclcpp::Parameter(entry.first, entry.second));
      RCLCPP_INFO(get_logger(), "已将参数 %s 同步为设备实际状态 '%s'。",
        entry.first.c_str(), entry.second.c_str());
    }
    catch (const std::exception & exception)
    {
      RCLCPP_WARN(get_logger(), "同步参数 %s 失败：%s", entry.first.c_str(), exception.what());
    }
  }
}

}  // namespace hikrobot_camera
