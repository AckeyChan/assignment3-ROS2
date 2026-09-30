# hikrobot_camera — 海康机器人 MVS 相机的 ROS 2 封装

在 Ubuntu + ROS 2 下封装海康机器人（HIKROBOT）机器视觉相机的 MVS SDK，实现设备发现、图像发布、参数在线读写与断线重连。

- 设备发现：列出全部 GigE / USB 相机，按 **IP 或序列号** 选择目标相机；设备不存在、标识冲突、被占用时给出明确日志。
- 图像发布：发布标准 `sensor_msgs/msg/Image`，话题可配置（默认 `/image_raw`），可在 `rviz2` 中直接查看。
- 参数读写：曝光时间（µs）、增益（dB）、帧率（fps）、像素格式均可通过 ROS 2 参数读取与动态设置；设置前校验范围、设置后回读校验，失败时返回明确原因；手动曝光/增益会自动关闭对应的自动模式。
- 断线重连：采集过程中掉线（或启动时未连接）会自动重试连接，重连成功后恢复全部配置。
- 资源管理：退出（Ctrl+C）时停止取流、关闭设备、销毁句柄并反初始化 SDK。
- 帧率统计：周期日志同时给出「设置帧率」与「实际接收帧率」，便于调优。

> 实测记录：MV-CS016-10UC（USB3，BayerRG8 1440×1080）→ 发布 `rgb8`，设置 30 fps 后实测接收 30.0 fps；SIGINT 退出资源正常释放。

## 1. 环境要求

| 依赖 | 说明 |
| --- | --- |
| Ubuntu 20.04 / 22.04+ | 开发环境为 Ubuntu + ROS 2 Lyrical，兼容 ROS 2 Humble 及更新发行版 |
| ROS 2 | `rclcpp`、`sensor_msgs`、`rcl_interfaces`（已在 `package.xml` 声明） |
| MVS SDK（厂商，需单独安装） | 默认安装到 `/opt/MVS`；安装后应存在 `/opt/MVS/include/MvCameraControl.h` 与 `/opt/MVS/lib/64/libMvCameraControl.so` |

### 安装 MVS SDK

1. 到海康机器人官网下载 Linux x64 版 MVS SDK：
   <https://www.hikrobotics.com/cn/machinevision/service/download/?module=0>
2. 按官方文档安装（.deb 包直接安装，或运行安装脚本），默认路径 `/opt/MVS`。
3. USB 相机权限由 SDK 自带的 udev 规则处理（`/etc/udev/rules.d/80-drivers-SDK-2bdf.rules`），如提示无访问权限请确认该文件存在并已 `udevadm control --reload`。

### 安装 ROS 依赖

```bash
cd <你的工作空间>
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 2. 编译

```bash
cd <你的工作空间>
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select hikrobot_camera
source install/setup.bash
```

MVS SDK 不在默认路径时：

```bash
colcon build --packages-select hikrobot_camera --cmake-args -DMVS_ROOT=<MVS 安装路径>
```

## 3. 运行

1. 按自己的相机修改参数文件 `src/hikrobot_camera/config/camera.yaml`：

```yaml
/hikrobot_camera:
  ros__parameters:
    type: "SN"               # "IP"=按网口相机 IP；"SN"/"ID"=按序列号；""=自动
    parameter: "DA0734524"   # 相机 IP 或序列号（留空则自动选择第一个设备）
```

2. 启动节点（运行目录任意，但**每个新终端都要先 source 工作空间**，否则会报 `Package 'hikrobot_camera' not found`）：

```bash
cd ~/your/workspace/project/assignment3-ROS2
source install/setup.bash 
ros2 launch hikrobot_camera camera.launch.py
# 或指定自己的参数文件
ros2 launch hikrobot_camera camera.launch.py params_file:=/path/to/camera.yaml
```

连接成功后日志示例：

```text
[INFO] ...: 相机连接成功并开始采集：USB 型号=MV-CS016-10UC SN=DA0734524（图像发布话题 'image_raw'）
[INFO] ...: 发布图像：1440x1080，encoding=rgb8，step=4320，帧数据长度=4665600 字节（设备像素格式 0x01080009）。
[INFO] ...: 帧率统计（最近 5.0 s）: 实际接收 30.0 fps，发布 30.0 fps；设置帧率 30.0 fps；取流超时 0 次，跳过 0 帧，丢包 0 个
```

3. 查看图像（任选）：

```bash
rviz2                                     # Add → By topic → /image_raw → Image
ros2 topic hz /image_raw                  # 实际发布帧率
ros2 topic echo /image_raw --field header # 查看时间戳 / frame_id
```

4. 退出：在运行 launch 的终端按 `Ctrl+C`，节点会停止取流并释放相机资源。

## 4. 参数说明

| 参数 | 类型 | 默认值 | 单位/取值 | 说明 | 可运行时修改 |
| --- | --- | --- | --- | --- | --- |
| `type` | string | `"SN"` | `IP` / `SN` / `ID` / `""` | 设备选择方式；`ID` 与 `SN` 等价 | 否（需重启） |
| `parameter` | string | `""` | IP 或序列号 | 目标设备标识；留空选第一个设备 | 否（需重启） |
| `retry_count` | int | `3` | 次 | 连接各步骤最大重试次数 | 是 |
| `latency` | int | `100` | ms | 连接重试间隔 | 是 |
| `reconnect_interval_ms` | int | `1000` | ms | 断线重连间隔 | 是 |
| `image_topic` | string | `"image_raw"` | 话题名 | 图像发布话题（相对名，默认解析为 `/image_raw`） | 否（需重启） |
| `frame_id` | string | `"camera"` | 字符串 | 图像消息 `header.frame_id` | 是 |
| `use_device_timestamp` | bool | `false` | — | `true`=相机主机时间戳；`false`=ROS 接收时刻 | 是 |
| `reliable_qos` | bool | `true` | — | `false` 时使用 best effort（高帧率时开销更小） | 否（需重启） |
| `qos_depth` | int | `10` | — | 图像话题队列深度 | 否（需重启） |
| `stats_interval_s` | double | `5.0` | s | 帧率统计日志间隔，`<=0` 关闭 | 是 |
| `pixel_format` | string | `""` | 设备支持的像素格式 | 如 `Mono8` / `BayerRG8` / `RGB8Packed`；空=保持相机当前设置 | 是 |
| `exposure_auto` | string | `"Off"` | `Off` / `Once` / `Continuous` | 自动曝光模式 | 是 |
| `gain_auto` | string | `"Off"` | `Off` / `Once` / `Continuous` | 自动增益模式 | 是 |
| `exposure_time` | double | `-1.0` | µs | 手动曝光时间；`<0` 表示不修改 | 是 |
| `gain` | double | `-1.0` | dB | 手动增益；`<0` 表示不修改 | 是 |
| `frame_rate` | double | `-1.0` | fps | 目标帧率；`<0` 表示不修改 | 是 |

> 注意：浮点参数需写成小数（例如 `5000.0`），整数形式（`5000`）会因类型不匹配被拒绝。
> 曝光、增益、帧率的具体取值范围由相机型号决定（节点在设置失败时会打印设备实际范围）。

### 运行时设置示例

```bash
# 手动曝光：节点会自动将 ExposureAuto 切换为 Off
ros2 param set /hikrobot_camera exposure_time 5000.0

# 手动增益：节点会自动将 GainAuto 切换为 Off
ros2 param set /hikrobot_camera gain 10.0

# 设置帧率（自动置 AcquisitionFrameRateEnable=true）
ros2 param set /hikrobot_camera frame_rate 30.0

# 切换像素格式（如从 BayerRG8 改为 Mono8）
ros2 param set /hikrobot_camera pixel_format Mono8

# 恢复自动曝光
ros2 param set /hikrobot_camera exposure_auto Continuous

# 查看参数描述（单位、默认值）
ros2 param describe /hikrobot_camera exposure_time
```

设置失败时命令行会直接显示原因，例如：

```text
Setting parameter failed: 曝光时间 1e+09 us 超出设备范围 [15, 9.99981e+06] us
Setting parameter failed: 不支持的像素格式 'NotAFormat'，设备支持: Mono8, Mono10, ...
```

## 5. 设计与行为说明

- **图像编码**：`Mono8→mono8`、`Mono16→mono16`、`RGB8/BGR8/RGBA8/BGRA8` 直接透传；`BayerRG8` 等 Bayer 格式与 `YUV422` 由 SDK 转换为 `rgb8`，`Mono10/Mono12` 转换为 `mono16`，保证在 `rviz2` 中正常显示。图像消息的 `width/height/step/data/encoding/stamp` 均由帧信息计算并具有明确含义。
- **话题 QoS**：默认 `reliable`，保证与 `rviz2`、`ros2 topic echo` 等默认订阅端兼容；高帧率场景可用 `reliable_qos:=false` 降低开销。
- **自动模式**：设置手动曝光/增益时会自动将对应自动模式切换为 `Off`，并把 ROS 参数同步为设备实际状态；再次设置 `exposure_auto/gain_auto` 可恢复自动。
- **断线重连**：通过 SDK 异常回调与取流错误双重检测；掉线后释放句柄并按 `reconnect_interval_ms` 重试，重连成功后重新下发（恢复）像素格式、曝光、增益、帧率等全部配置。启动时相机未连接或短暂被占用也会自动等待重连，无需重启节点。
- **帧率统计**：周期日志给出「实际接收 fps / 发布 fps / 设置帧率」，便于区分逻辑帧率与实际吞吐；GigE 相机自动设置最佳包长（`GevSCPSPacketSize`）以提高实际帧率。
- **退出清理**：`Ctrl+C` 触发采集线程退出 → 停止取流 → 关闭设备 → 销毁句柄 → `MV_CC_Finalize`，不残留资源。
- **已知限制**：
  - `type/parameter/image_topic/reliable_qos/qos_depth` 的修改需重启节点（运行时修改会被拒绝并说明原因）；
  - 一次请求批量修改多个参数时（少见），若中途某个参数失败，之前已成功下发的参数不会回滚，重新设置即可；
  - 相机设置项的名称遵循 GenICam 标准（`ExposureTime`、`Gain`、`AcquisitionFrameRate`、`PixelFormat` 等），个别机型缺少某节点时对应设置会返回明确错误。

## 6. 故障排查

| 现象 | 处理 |
| --- | --- |
| 日志：`0x80000203 设备无访问权限（可能被其他程序占用）` | 关闭 MVS 客户端等占用程序；节点会自动重试，通常数秒内恢复；必要时重新插拔相机 |
| 日志：`未找到 ... 相机` | 核对 `type`/`parameter`；`ros2 param set` 前先看启动日志中打印的在线设备列表 |
| 收不到图像 | 确认 `TriggerMode` 为 Off（节点自动设置）；确认 `pixel_format` 为设备支持值 |
| `rviz2` 无画面 | 订阅话题选择 `/image_raw`；本驱动默认 reliable QoS，与 RViz 兼容；确认 Fixed Frame 设置不影响 Image 显示 |
| 帧率低于预期 | 参考统计日志区分「设置帧率」与「实际接收帧率」；调低 `frame_rate` 反而可能提升稳定性；USB 相机注意总线带宽 |

## 7. 目录结构

```text
src/hikrobot_camera/
├── CMakeLists.txt              # 构建配置（含 MVS SDK 查找，支持 -DMVS_ROOT）
├── package.xml                 # ROS 依赖（厂商 SDK 需单独安装）
├── config/camera.yaml          # 参数文件（设备选择、图像、成像参数）
├── launch/camera.launch.py     # 启动文件
├── include/hikrobot_camera/camera_node.hpp
└── src/
    ├── main.cpp
    └── camera_node.cpp         # 设备发现/选择、取流发布、参数读写、重连、资源管理
```

更多背景见 `docs/assignment.md`；ROS 2 基础操作见 `docs/ROS2Tutorial.md`。
