# UR7e + D435 外侧眼在手上 Gazebo 场景

ROS 2 Humble / Gazebo Classic 11 功能包。启动 UR7e、按实测外参放置的 D435 外形和传感器、
两面墙和一个球。默认保留实测外参，并在启动时检查相机与机械臂的几何相交。

当前默认使用 2026-09-07 ChArUco 实测标定，初始姿态相机与机械臂无相交，最小间隙约 12.14 mm。
导入说明、单位转换及验证见 [HANDEYE_20260907_IMPORT.md](docs/HANDEYE_20260907_IMPORT.md)。
旧外参嵌入问题的历史分析见 [HANDEYE_DIAGNOSIS.md](docs/HANDEYE_DIAGNOSIS.md)。

## 构建与启动

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./run_ur7e_d435_scene.sh
```

可追加 `gazebo_gui:=false`。先关闭其他使用同一 ROS 域和 Gazebo 端口的仿真实例。
启动会检查 `GAZEBO_MASTER_URI`（默认端口 11345）；占用时会直接报错，不会打开一个无法生成模型的空场景。
只关 Gazebo 窗口可能留下 `gzserver`，应在原启动终端按 Ctrl+C 退出整个场景。
可用 `ss -ltnp 'sport = :11345'` 查占用进程；确认是残留仿真实例后再用 `kill -INT <PID>` 正常停止。
如果服务器启动后异常退出，启动器会同时停止其他场景节点，并输出 Gazebo 详细日志。
两个脚本只加载系统 ROS 和当前工作空间，不需要 source 旧机械臂、相机、STORM 工作空间，
也不需要 `source_realtime_nvblox.sh` 的 Torch/cuVSLAM 环境。

标准 ROS 入口（新终端）：

```bash
source /opt/ros/humble/setup.bash
source /home/czz/perception_eye_in_hand/install/local_setup.bash
ros2 launch ur7e_d435_gazebo scene.launch.py
```

## 依赖

功能包内包含相机插件源码、当前 UR7e 模型、所有引用网格、相机内部描述、外参、支架、
障碍物、控制器配置和生成脚本。运行路径通过 ament 包索引查找，不引用旧工作空间文件。
第三方来源和许可证见 `THIRD_PARTY.md` 与 `licenses/`。

ROS/Gazebo 系统依赖完整声明在 `package.xml`；新机器安装依赖可运行：

```bash
cd /home/czz/perception_eye_in_hand
rosdep install --from-paths src/ur7e_d435_gazebo --ignore-src --rosdistro humble -r -y
```

构建需要系统 C++ 编译器、CMake 和 colcon。相机渲染需要可用的图形环境；关闭 GUI 仍会渲染深度。
本包不会启动 nvblox 或 MPPI，不会修改既有 realtime_nvblox 包。

## 配置与输出

- `config/ur7e_d435_handeye_20260907.yaml`：默认 `tool0 → camera_color_optical_frame`，
  平移 `[-0.029901376, -0.069185919, -0.012263443]` m，旋转由原始标定矩阵转换。
  启动时检查标定类型及父子坐标系名称；这些名称必须与原始标定配置一致，不能仅改名称而不转换外参。
- `config/ur7e_d435_side_sim_mount.yaml`：保留此前配置，可通过 `calibration_file:=...` 显式对照。
- `urdf/d435_handeye_mount.urdf.xacro`：从彩色光学中心反推相机安装孔和外壳位置。
  当前不加载旧的仿真外侧支架；真实支架的间隙需要由正确的标定坐标系和外参体现。
- `config/initial_positions.yaml`：六关节初始位置。
- `config/collision_world_gazebo_tall.yml`：两面墙和球；生成时沿用 `sage_clean_*` 模型名。
- `config/controllers.yaml`：100 Hz 的 controller_manager、关节状态和位置控制器。
- `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、`/camera/depth/color/points`：1280×720。
- `/joint_states`、`/forward_position_controller/commands`：保留原控制接口。

修改安装配置后重启即可（symlink-install）。不保证任意机械臂动作都无自碰撞；
原 MPPI 碰撞球尚未包含新增相机和支架。深度未对齐到 RGB，未模拟 IMU。

## 启动验证

仿真运行后，在加载系统 ROS 和本工作空间的终端执行：

```bash
ros2 run ur7e_d435_gazebo verify_scene.py --output /tmp/ur7e_d435_scene_check.json
```

检查实际图像/点云的连续时间戳、TF、六关节初始状态、两个 active 控制器、三个障碍物和机械臂，
以及所有网格引用都位于当前功能包，并检查相机显示网格和碰撞盒与机械臂几何的距离/相交。
该检查应在启动 MPPI 或发出关节运动命令之前运行。
当前默认 ChArUco 外参的初始安装几何检查通过；旧的相交外参仍会返回 `result=FAIL_GEOMETRY` 和退出码 1。
若启动时指定了 `calibration_file:=...`，验证时也应使用 `--calibration ...`。

以下为 2026-09-15 旧仿真安装配置的历史记录，不代表当前实测外参通过几何验证。
当时已完成实际构建和 GUI 启动，运行检查通过。
Gazebo 加载的是本包构建的相机插件；新旧完整展开 URDF 在归一化资源路径后相同，
25 个 link、24 个 joint 和 15 处网格引用一致，网格内容 SHA256 一致。
初始姿态、相机外参和障碍物配置也与原场景逐字节一致。
本次运行证据保存在工作空间 `log/`：

- `ur7e_d435_scene_validation.json`：实际模型、控制器、TF、相机消息与关节状态。
- `ur7e_d435_scene_equivalence.json`：新旧模型一致性检查结果。
- `ur7e_d435_package_scene.png`：Gazebo 界面截图。

## 实验台复杂场景

新增 `ros2 launch ur7e_d435_gazebo lab_scene.launch.py`，包含工作台、双试管架、试剂瓶、托盘和尺度块；原场景保留。完整编译、建图及查看说明见工作区 `docs/LAB_WORKBENCH_SCENE_20261110.md`。
