## 项目介绍

本项目是东北大学 T-DT 战队面向 RoboMaster 反制无人机程序。

## 项目优势

1. **使用雷达距离动态计算激光落点**
   激光雷达提供目标三维位置和距离，程序根据多组距离标定数据动态计算图像中的激光落点。相机、激光和云台可以采用非同轴安装，不需要强制保证光轴重合。

2. **使用球坐标卡尔曼预测无人机运动**
   程序将目标像素误差转换为球坐标中的 yaw、pitch 角度误差，在角度域估计目标角度与角速度，并预测控制指令生效时刻的无人机位置，降低检测噪声与系统延迟对跟踪的影响。

3. **配合哨兵行动进行反制**
   程序通过 `match_info` 接收哨兵指令和当前反制状态，将哨兵行动纳入反制时机判断，使无人机反制与哨兵作战策略协同执行。

## 核心技术与算法

### 基于雷达距离的动态激光落点

`fly_target.yaml` 保存 12、16、20、24 米处的激光落点标定坐标。程序分别对 x、y 方向拟合以下关系：

激光落点标定程序属于雷达主程序，位于 `src/tdt_vision/calibrate_fly/`，对应启动文件为 `src/tdt_vision/launch/calibrate_fly.launch.py`。Lock 模块不负责执行标定，只读取雷达主程序生成并保存在根目录 `config/fly_target.yaml` 中的标定结果。

```text
target = A / distance + B
```

收到 `/livox/lidar_fly_point` 后，程序计算目标三维距离，并实时更新当前距离对应的 `target_x` 和 `target_y`。视觉跟踪使用动态落点与无人机像素位置之间的角度差，而不是固定使用图像中心。

### 非同轴坐标转换

模块发布 `pitch_link` 到 `camera_link` 的静态坐标变换，并将雷达目标点转换到相机坐标系后计算首次锁定角度。相机、激光和云台的安装位置可以不同，但机械结构变化后需要重新测量外参。

当前静态平移定义在 `src/lock/src/lock.cpp` 的 `publish_static_tf()` 中。后续若调整硬件位置，应同步修改该变换与首次锁定偏置。

### 球坐标角度域卡尔曼

视觉检测给出无人机在图像中的像素坐标，程序利用相机焦距将像素误差转换为 yaw、pitch 角度误差，再与图像时间戳附近的云台姿态相加，得到目标的绝对角度测量。

卡尔曼状态包含目标角度与角速度。每次收到视觉测量后更新滤波器，再根据图像采集到当前时刻的延迟和 `control_delay_s` 预测控制真正生效时的目标角度。视觉消息超过 1 秒未更新时，滤波器会被重置，系统改用雷达目标点进行首次锁定和巡航。

### 反制状态控制

`match_info` 提供比赛剩余时间、对方无人机反制状态和哨兵指令。程序记录反制次数，在一次反制结束后按照 `countermeasure_interval_s` 等待，并结合剩余比赛时间决定是否跳过等待。`match_info` 超时后，系统按照配置中的失联逻辑处理开火状态。

## 硬件条件

- 两个6020电机
- 单目相机 Hikvision CS-016

## 项目结构说明

```text
src/lock/
├── CMakeLists.txt              # tdt_lock 构建和组件注册
├── package.xml                # ROS 2 包信息与依赖
├── README.md                  # 模块说明
├── include/
│   ├── lock.h                 # ROS 2 接口、控制状态和配置定义
│   ├── kalman_cv.h            # yaw、pitch 角度域卡尔曼滤波
│   └── kalman_ca.h            # 匀加速卡尔曼实现
└── src/
    ├── lock.cpp               # 锁定、巡航、坐标转换和反制决策
    └── read_config.cpp        # 配置读取与默认值处理

config/
├── lock_config.yaml           # 滤波、延迟、反制决策和控制参数
├── f.yaml                     # 相机焦距
└── fly_target.yaml            # 距离与激光落点标定数据
```

## 模块介绍

| 模块 | 说明 |
| --- | --- |
| `tdt_lock::Lock` | ROS 2 主节点，管理输入输出、锁定状态、巡航和反制决策 |
| `Kalman_filter_plus` | 在 yaw、pitch 角度域跟踪并预测无人机运动 |
| `DetectFly` | 位于 `tdt_vision`，向本模块提供飞行目标像素坐标 |
| `Cluster` | 位于激光雷达链路，向本模块提供飞行目标三维坐标 |
| `CalibrateFly` | 位于雷达主程序的 `tdt_vision`，负责生成激光落点标定数据 |

## 编译

在项目根目录执行：

```bash
./build.sh
source install/setup.zsh
```

Bash 用户将 `setup.zsh` 替换为 `setup.bash`。模块通过相对路径读取根目录下的配置文件，因此运行时也应位于项目根目录。

## 运行

由外部节点提供全部输入时，可以独立启动：

```bash
ros2 run tdt_lock lock_node
```

单独模块测试，可以启动飞行目标检测与反制链路：

```bash
ros2 launch tdt_vision radar_fly.launch.py
```

赛场上全部启动时：

```bash
ros2 launch tdt_vision radar_all.launch.py
```

## 进程间通信接口

### ROS 2

| 方向 | Topic | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `detect_fly` | `vision_interface/msg/DetectFly` | 目标像素坐标 |
| 订阅 | `/livox/lidar_fly_point` | `geometry_msgs/msg/Point32` | 飞行目标三维坐标和距离 |
| 订阅 | `gimbalUsartData` | `gimbal_interface/msg/GimbalAngle` | 云台 yaw、pitch 反馈 |
| 订阅 | `match_info` | `vision_interface/msg/MatchInfo` | 比赛时间、反制状态和哨兵指令 |
| 发布 | `GimbalPub` | `gimbal_interface/msg/GimbalAngle` | 云台信息 |

## 配置

### `lock_config.yaml`

| 参数 | 用途 |
| --- | --- |
| `kf_measurement_noise_px` | yaw 方向视觉测量标准差，单位 px |
| `kf_measurement_noise_y_px` | pitch 方向视觉测量标准差，单位 px |
| `kf_q_x_rad2_s3` | yaw 方向过程噪声，单位 rad²/s³ |
| `kf_q_y_rad2_s3` | pitch 方向过程噪声，单位 rad²/s³ |
| `kf_initial_velocity_std_deg_s` | 初始角速度标准差，单位 deg/s |
| `control_delay_s` | 云台控制延迟补偿，单位 s |
| `countermeasure_interval_s` | 一次反制结束后的等待时间，单位 s |
| `match_info_timeout_s` | 比赛信息失联判定时间，单位 s |
| `first_lock_yaw_offset_deg` | 雷达首次锁定 yaw 偏置，单位 deg |
| `first_lock_pitch_offset_deg` | 雷达首次锁定 pitch 偏置，单位 deg |
| `kp_x` | yaw 控制比例增益 |
| `kp_y` | pitch 控制比例增益 |

建议先在雷达主程序中完成焦距、激光落点和机械外参标定，再调节首次锁定偏置与比例增益，最后根据动态跟踪表现调节卡尔曼噪声和延迟补偿。
