# ur7e_nvblox_in_hand

UR7e眼在手上的D435/D435i感知工作区，提供Gazebo实验台仿真、深度与颜色融合、彩色Mesh重建、二维/三维ESDF及ROS可视化。

ROS输入与调度由Python实现，核心地图计算使用官方`nvblox_torch` wheel中的nvBlox C++/CUDA。本地`native/`桥接补充原生地图层导出等能力。运行不需要原来的`perception_eye_in_hand`父工程或`workspaces/isaac_ros-dev`，但需要安装匹配的ROS、CUDA、Torch与nvBlox依赖，并在目标机编译。

本文以工作区位于`$HOME/ur7e_nvblox_in_hand`为例。命令须在满足对应前置条件后执行；源码复制完成不代表已在新路径完成编译或性能验证。

## 1. 功能

| 功能 | 内容 |
| --- | --- |
| 深度融合 | 接收深度图和CameraInfo，按图像时刻查询相机TF，调用官方Mapper更新TSDF |
| 颜色融合 | 使用彩色图自身内参与位姿更新颜色地图；不是UV纹理贴图 |
| Mesh重建 | 从融合地图提取彩色三角网格，发布MarkerArray，并支持保存PLY |
| 3D ESDF | 计算距离场，导出已观测体素中心、距离及inside/site等状态 |
| 2D ESDF | 根据高度配置生成ESDF切片，和3D模式择一运行 |
| 地图层 | 发布TSDF层和颜色层供诊断 |
| 相机轨迹 | 发布机械臂TF对应的相机位姿和路径，不是视觉SLAM估计 |
| 地图服务 | 保存/加载地图、保存Mesh和统计、清空地图、查询ESDF距离与梯度 |
| 仿真 | 自带UR7e、D435、工作台、试管架、瓶子及托盘等场景资源 |
| 诊断 | 输入/融合/输出计数、主机调用耗时、实际ESDF接收频率和仿真测试工具 |

当前主路径使用静态场景融合，默认关闭衰减与半径清图。未提供默认机器人自体剔除、动态物体识别或抓取规划。机械臂经过视野、深度缺测、位姿/颜色投影误差仍可能影响地图。

## 2. 代码结构与各部分作用

```text
ur7e_nvblox_in_hand/
├── src/
│   ├── ur7e_nvblox_in_hand/       # Python ROS感知包
│   │   ├── ur7e_nvblox_in_hand/   # node.py / backend.py / messages.py
│   │   ├── launch/               # 场景、输入、Mapper和RViz编排
│   │   ├── config/               # 地图参数、profile、D435和DDS配置
│   │   ├── rviz/                 # diagnostic / mesh / 2d显示配置
│   │   └── resource/             # ament包索引标记
│   ├── ur7e_nvblox_interfaces/    # FilePath与QueryEsdf服务定义
│   └── ur7e_d435_gazebo/          # UR7e和D435完整仿真包
│       ├── launch/               # 场景及可选标定启动入口
│       ├── config/               # 控制器、初始姿态、物品布局和相机安装
│       ├── worlds/               # 实验台world
│       ├── urdf/                 # 机械臂和相机装配xacro
│       ├── meshes/               # 机械臂、D435及实验台模型
│       ├── scripts/              # 物品生成、几何验证及仿真扫描
│       ├── src/                  # 安装网格和扫描碰撞检查C++工具
│       ├── third_party/          # RealSense Gazebo插件源码
│       ├── test/                 # 场景/手眼/扫描几何测试
│       └── docs/、licenses/      # 场景说明与第三方许可证
├── native/                       # C++/CUDA桥接源码
├── vendor/                       # 固定版本nvblox/Torch绑定/stdgpu头文件
├── tools/                        # GPU检查、扫描、监测、截图与验收工具
├── tests/                        # 项目级回归测试
├── provenance/                   # 来源提交、历史版本与验证记录
├── docs/                         # 项目说明预留目录
├── build.sh                      # 检查wheel、构建桥接及三个ROS包
├── install_dependencies.sh       # 显式安装依赖
├── env.sh                        # 加载本工作区环境
├── run.sh                        # 启动Mapper及可选场景/RViz/相机/bag
├── scene.sh                      # 仅启动Gazebo场景
├── rviz.sh                       # 仅启动RViz
├── README_REAL_TEST.md           # 已有实机操作说明
├── THIRD_PARTY.md                # 第三方来源和版本边界
└── LICENSE
```

### 感知代码的分工

- `node.py`：订阅Image和CameraInfo，缓存输入，按图像时间查询TF，串行调度GPU地图任务，独立线程打包输出，并提供服务和metrics。
- `backend.py`：检查深度/彩色编码、转换深度单位、设置Mapper参数，调用官方融合/ESDF/Mesh接口及本地桥接。
- `messages.py`：将CPU数组转换为PointCloud2和MarkerArray，不负责计算TSDF。
- `native/bridge.cpp`：将原生地图访问功能注册为PyTorch算子。
- `native/export.cu`：在GPU上筛选体素，导出位置、距离、权重/颜色/状态，再回读CPU。包含GPU同步和复制开销。
- `vendor/`：编译所需的匹配头文件，不是完整官方源码或运行库。存在语义/动态等头文件不表示当前启用了这些功能。

### 数据流

```text
Gazebo相机 / 真实D435i
  ├── 深度Image + 深度CameraInfo
  └── 彩色Image + 彩色CameraInfo
               + 机械臂TF（各图像采集时刻）
                         ↓
                  node.py输入与调度
                         ↓
              backend.py → nvblox_torch C++/CUDA
                         ↓
              TSDF / 颜色地图 / ESDF / Mesh
                         ↓
           原生导出 / CPU回读 / ROS消息打包
                         ↓
                  ROS话题、RViz、文件服务
```

CameraInfo当前采用每路最新有效缓存，并检查图像分辨率和frame是否匹配，没有强制其时间戳与每张Image完全相同；TF严格按各图像时间戳查找。深度与颜色分开处理，没有在本节点内强制逐帧RGB-D配对。输入的内参、光学坐标系、TF和时间必须正确。

## 3. 依赖与迁移约束

参考组合来自当前源码和历史环境记录，不代表任意主机无需验证即可使用：

| 项目 | 参考环境/要求 |
| --- | --- |
| 系统 | Ubuntu22.04 x86_64，系统Python3.10 |
| ROS | ROS2 Humble、CycloneDDS、RViz、接口生成与TF相关包 |
| 仿真 | Gazebo Classic11、gazebo_ros、gazebo_ros2_control及ROS控制器 |
| GPU | 当前Torch/wheel支持的NVIDIA GPU及兼容驱动 |
| 编译 | CMake、C++17、CUDA Toolkit；历史验证使用12.6 |
| Python计算 | Torch2.9.1 cu128、torchvision0.24.1、NumPy1.26.4及脚本列出的依赖 |
| nvBlox | 官方v0.0.10 Ubuntu22/CUDA12/x86_64 wheel，源码SHA必须匹配vendor |
| 开发库 | Eigen、glog、gflags、FCL等 |

匹配的nvBlox源码SHA：

```text
e4a2b00f45ecf989682d72a33a2c29defe39fd2d
```

`tools/check_wheel.py`会检查SHA及Torch CUDA可用性。这个wheel不是Jetson ARM安装包。Torch自带的CUDA运行依赖不等于完整CUDA Toolkit，本地桥接仍需nvcc。

`build.sh`和CMake使用`/usr/bin/python3`。仅激活venv不会改变这些调用；不要混用不同解释器、Torch头文件和旧桥接.so。更换用户、安装路径、GPU或库版本后重新编译。

### 已有ROS/CUDA主机安装工程依赖

先确保ROS apt源、ROS Humble、兼容驱动和CUDA Toolkit已安装。下面的脚本不负责安装这些前置环境：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
nvidia-smi
/usr/local/cuda-12.6/bin/nvcc --version
source /opt/ros/humble/setup.bash

./install_dependencies.sh --system

# rosdep尚未初始化时才执行sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

/usr/bin/python3 tools/check_wheel.py
/usr/bin/python3 -m pip check
```

安装脚本会更改系统/用户依赖，已有匹配环境无需反复执行。全新空系统需先完成ROS、驱动和CUDA的官方安装流程，不能把本脚本当作空机器一键部署。只做Gazebo不需要RealSense实机SDK；当前完整构建仍会编译Gazebo插件。

## 4. 编译与环境

```bash
cd "$HOME/ur7e_nvblox_in_hand"
/usr/bin/python3 -c 'import torch; print(torch.cuda.get_device_name()); print(torch.cuda.get_device_capability())'

# 按上条结果填写：例如(8,9)为89，(8,6)为86，并确认wheel/Toolkit支持
export UR7E_CUDA_ARCH=89
export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export UR7E_BUILD_JOBS=1
./build.sh
source ./env.sh

ros2 pkg prefix ur7e_nvblox_in_hand
ros2 pkg prefix ur7e_d435_gazebo
/usr/bin/python3 tools/check_backend.py
```

GPU架构89是原RTX4060参考值，不应在其他GPU上盲目沿用。包前缀应在本工作区`install/`内。

`build.sh`先生成`install_native/lib/libur7e_nvblox_bridge.so`，再构建三个ROS包。根`COLCON_IGNORE`用于避免被父工程重复扫描；本脚本显式构建src。`check_backend.py`会使用GPU和合成数据进行接口检查，不驱动机械臂。

每个运行/监测终端均执行`source ./env.sh`。此脚本会清理其他工作区的相关环境，默认DDS域135、仅本机通信、Gazebo端口11565，并设置本工作区日志及桥接路径。不要再混入兄弟工作区的setup。

## 5. 仿真快速启动

### 5.1 场景、Mapper与RViz一起启动

在有正常图形桌面的终端中：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
./run.sh profile:=realtime scene_variant:=lab \
  start_scene:=true launch_camera:=false use_sim_time:=true \
  gazebo_gui:=true run_rviz:=true rviz_mode:=diagnostic
```

这会显示实验台与地图，但不会自动扫描机械臂。默认5mm地图负担较大，首次检查可显式追加`voxel_size:=0.01`。参数更改需重新启动建立对应地图。

### 5.2 先调整观察姿态，再启动Mapper

此方式避免把机械臂接近工作台的过程融合进静态地图。先停止上一种一体启动实例，不同时创建两个Gazebo。

终端A，启动场景：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
./scene.sh scene_variant:=lab gazebo_gui:=true
```

终端B，检查仿真并移动至已保存观察姿态：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
/usr/bin/python3 tools/view_sweep.py --check-only
/usr/bin/python3 tools/view_sweep.py --seconds 0
```

终端C，启动Mapper和RViz：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
./run.sh start_scene:=false launch_camera:=false use_sim_time:=true \
  profile:=realtime input_qos:=reliable \
  run_rviz:=true rviz_mode:=diagnostic
```

分开启动时，Mapper不会通过start_scene分支自动选择Reliable，因此这里显式与默认Reliable仿真发布端匹配。

需要轻微左右扫描时，在终端B执行：

```bash
/usr/bin/python3 tools/view_sweep.py --seconds 20 --amplitude 0.04
```

该工具会发布仿真关节命令，依赖保存的初始/观察姿态和路径，不能用于真实机械臂。`tools/view_sweep.py`和场景包内同名脚本用途不同，不要混用。

### 5.3 2D显示与单独打开RViz

```bash
# 与3D择一启动
./run.sh esdf_mode:=2d rviz_mode:=2d

# 已有Mapper时，仅打开显示窗口
./rviz.sh diagnostic
./rviz.sh mesh
```

`rviz.sh`不生成地图。实机单独使用它时需设置`USE_SIM_TIME=false`，且自行确认其显示话题与真实输入一致；使用bringup启动的RViz会根据输入话题进行映射。

## 6. 配置与参数

主要配置位于`src/ur7e_nvblox_in_hand/config/`。一般覆盖顺序：`base.yaml` → profile YAML → `mapper_config` → 显式启动参数。图像话题还会根据`use_sim_time`选择仿真/实机默认值，显式话题参数优先。

| 文件 | 作用 |
| --- | --- |
| base.yaml | 公共地图、队列、深度范围、话题与算法参数 |
| realtime.yaml | 默认实时profile，覆盖频率、5mm精度、关闭衰减和清图 |
| reference.yaml | 历史参考profile，使用2cm、较低颜色/ESDF上限 |
| d435.yaml | 真实相机640×480×30，RGB/深度开启，infra/IMU关闭 |
| cyclonedds_local.xml | 本机DDS配置 |

默认realtime的关键有效参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| voxel_size | 0.005 m | TSDF/ESDF地图体素边长 |
| truncation_vox | 4.0 | 截断距离为4个体素；5mm地图对应2cm |
| esdf_mode | 3d | 三维ESDF模式 |
| esdf_view_mode | observed | 导出已观测ESDF体素，不等于只显示物体表面 |
| esdf_view_stride | 2 | 显示抽样间隔约为2×体素，不改变融合分辨率 |
| depth_hz / color_hz | 40 / 40 | 允许融合的频率上限 |
| esdf_hz / esdf_view_hz | 33.4 / 33.4 | ESDF更新/导出的调度上限 |
| mesh_hz / layer_hz | 5 / 5 | Mesh/层输出上限 |
| min_depth_m / max_range_m | 0.05 / 2.05 | 接受的深度范围 |
| workspace_bounds_type | bounding_box | 限制地图工作范围 |
| workspace x / y / z | [-0.1,1.1] / [-0.6,0.6] / [-0.05,1.05] m | 当前实验台的地图边界 |
| clearing_hz / decay_hz | 0 / 0 | realtime默认保留静态地图 |

仿真图像设置30Hz，融合上限40Hz不会产生40Hz新图像。ESDF更新调用、实际点云接收和RViz FPS是不同指标，不承诺5mm配置在所有硬件上达到30Hz。

**现有配置注意事项：** base.yaml中clearing_hz=1且clearing_radius_m=-1。realtime覆盖为clearing_hz=0，因此默认可以通过这项校验；reference未覆盖。如果使用reference或直接只加载base，应显式关闭清图，例如`profile:=reference clearing_hz:=0.0`，或设置经过审查的正半径。本文没有修改这些配置。

动态参数仅限节点允许的功能开关、频率和部分显示参数。voxel_size等地图结构参数需重启，不能当作在线调节旋钮。

## 7. 话题、实时性监测与保存

节点名默认为`ur7e_nvblox`：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| /ur7e_nvblox/esdf_points | sensor_msgs/PointCloud2 | 3D原生ESDF体素采样和距离/状态 |
| /ur7e_nvblox/static_esdf_pointcloud | sensor_msgs/PointCloud2 | 2D模式距离切片 |
| /ur7e_nvblox/mesh | visualization_msgs/MarkerArray | 完整彩色Mesh，使用标准MarkerArray显示 |
| /ur7e_nvblox/tsdf_layer | sensor_msgs/PointCloud2 | TSDF层诊断 |
| /ur7e_nvblox/color_layer | sensor_msgs/PointCloud2 | 颜色层诊断 |
| /ur7e_nvblox/camera_pose | geometry_msgs/PoseStamped | 图像对应相机位姿 |
| /ur7e_nvblox/camera_path | nav_msgs/Path | 机械臂相机轨迹 |
| /ur7e_nvblox/metrics | std_msgs/String | JSON统计、配置及库信息 |

ESDF/Mesh/地图层的导出会检查订阅者，地图有变化且调度到期才产生相应输出，不靠重复发送旧地图填满频率。

监测终端：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 topic list -t
ros2 topic echo /ur7e_nvblox/metrics --once
/usr/bin/python3 tools/watch_realtime.py
```

结束上一个监测后，按需运行：

```bash
/usr/bin/python3 tools/watch_esdf_hz.py
ros2 topic info /ur7e_nvblox/esdf_points --verbose
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time --window 20
```

hz没有输出时，先检查发布者、DDS域、RMW、QoS和Mapper存活状态。metrics中的主机调用耗时可能包含等待，不等于独立GPU kernel耗时。

### 保存结果

FilePath服务接受绝对路径，结果统一写入artifacts，避免混入源码：

```bash
mkdir -p "$HOME/ur7e_nvblox_in_hand/artifacts/manual"
ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$HOME/ur7e_nvblox_in_hand/artifacts/manual/mesh.ply'}"
ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$HOME/ur7e_nvblox_in_hand/artifacts/manual/map.blox'}"
ros2 service call /ur7e_nvblox/save_timings ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$HOME/ur7e_nvblox_in_hand/artifacts/manual/metrics.json'}"
```

保存时使用新文件名避免覆盖。PLY用于外部查看Mesh；原生地图用于后续加载。若需保留当时发布的ESDF点云，可另外录制话题：

```bash
ros2 bag record -o "$HOME/ur7e_nvblox_in_hand/artifacts/manual/esdf_bag" \
  /ur7e_nvblox/esdf_points
```

Ctrl+C结束录制，目标bag目录必须尚不存在。其他服务包括`load_map`、`save_rates`、`clear_map`和`get_esdf_and_gradient`。加载地图检查体素尺寸，查询可接收点集或有限AABB，unknown不应当作自由空间使用。

## 8. 实机输入

真实测试需要另行运行UR机器人驱动、D435i ROS驱动和正确手眼TF。默认仿真命令不能代替实机硬件启动。已有说明见[README_REAL_TEST.md](README_REAL_TEST.md)，其中历史IP、相机序列号和标定仅适用于核对后的已有设备。

假设真实相机和机器人TF已经启动：

```bash
cd "$HOME/ur7e_nvblox_in_hand"
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
./run.sh start_scene:=false launch_camera:=false use_sim_time:=false \
  profile:=realtime input_qos:=sensor_data run_rviz:=true \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  depth_info_topic:=/camera/camera/depth/camera_info \
  color_topic:=/camera/camera/color/image_raw \
  color_info_topic:=/camera/camera/color/camera_info
```

这条命令不向机械臂发布运动。手动拖动时需持续提供正确关节状态及TF。应按图像实际header.frame_id验证到base_link的链路：depth与color使用各自光学坐标，不能通过改frame名字强行把未对齐深度当成彩色视角。

本节点不自动完成深度/RGB配准、畸变矫正或手眼标定；backend当前从CameraInfo.k构造相机模型，需确认与输入图像相符。地图bounding_box沿用实验台范围，真实工作范围不同时应明确覆盖相应workspace参数。

## 9. 工具与测试

| 工具 | 功能及执行影响 |
| --- | --- |
| tools/check_wheel.py | 检查wheel提交和GPU可用性 |
| tools/check_backend.py | GPU合成数据验收、地图接口核对 |
| tools/watch_realtime.py | 查看metrics，不驱动机械臂 |
| tools/watch_esdf_hz.py | 订阅ESDF统计实际接收率，会增加输出需求 |
| tools/view_sweep.py | 依据motion_plan.json移动仿真机械臂 |
| tools/sim_test.py | 启动完整仿真验收并移动仿真机械臂，生成artifacts |
| tools/capture_rviz.py | X11下抓取RViz窗口，需可用DISPLAY |
| tools/write_validation.py | 整理既有测试结果，不替代实际验收 |
| tests/ | 消息、stride和话题选择回归测试 |
| src/ur7e_d435_gazebo/test/ | 场景几何、手眼与扫描检查 |

项目级回归测试：

```bash
source ./env.sh
/usr/bin/python3 -m unittest discover -s tests -v
```

完整仿真验收应先停止手工场景/Mapper，确认端口可用，在正常图形桌面运行：

```bash
source ./env.sh
/usr/bin/python3 tools/sim_test.py
```

不要照抄别的机器DISPLAY值，也不要在真实硬件会话中运行仿真运动工具。历史provenance验证记录不表示新路径或新机器已完成本次测试。

## 10. 数据管理、已知边界与常见问题

不上传`artifacts/`、`build*/`、`install*/`、`log/`、`.ros/`、缓存、虚拟环境、录制和下载的wheel。模型STL/DAE、必要JSON/YAML和vendor头文件属于源码/资源，应保留。

现有.gitignore仅覆盖部分生成目录。发布前应在Git变更列表确认没有测试数据；已经跟踪的文件不会因增加.gitignore自动消失。本README不更改Git忽略规则或远程地址。

| 现象 | 优先检查 |
| --- | --- |
| Bridge missing | 是否先执行build.sh，env.sh是否来自本工作区 |
| Wheel source commit differs | wheel与vendor提交不匹配，不绕过检查 |
| undefined symbol / CUDA错误 | Torch ABI、wheel、驱动、Toolkit和旧.so混用情况 |
| clearing requires positive radius | 关闭清图或提供正半径，尤其检查reference profile |
| Gazebo端口占用 | 先确认原gzserver所属会话；不要批量终止其他项目 |
| 只有RGB没有重建 | 深度/CameraInfo、图像时刻TF、use_sim_time和Mapper日志 |
| ESDF看起来覆盖整个视场 | observed含已观测自由空间，不能把全部点当障碍表面 |
| Mesh模糊或颜色扩展 | 体素、截断尺度、深度边界、内参、位姿/时间及遮挡投影 |
| 实际频率不足上限 | 输入率、过期/限频计数、地图规模、GPU导出和CPU打包耗时 |

当前`docs/`仅保留空目录，旧工程的历史测试报告与公共系统安装文档未包含在此副本中。`tools/ur_calibration_compat/`同样只有空目录骨架，不能作为可用工厂标定包。不要依赖这些空目录完成部署；后续可单独补充必要文档和经过审查的工具。

## 11. 来源与许可

新增适配代码及第三方内容的边界见[THIRD_PARTY.md](THIRD_PARTY.md)、[LICENSE](LICENSE)与[场景资源来源](src/ur7e_d435_gazebo/THIRD_PARTY.md)。保留原模型、插件和vendor文件的版权声明。

`provenance/`中的旧路径描述来源，不表示运行时必须存在旧workspaces。此项目不随源码分发官方核心运行库，迁移时应通过匹配的官方wheel安装，不能删除vendor后凭同名库替代。
