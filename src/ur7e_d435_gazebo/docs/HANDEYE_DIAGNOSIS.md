# 相机嵌入末端：模型、证据与处理流程

> 本文记录 2026-09-22 旧外参的诊断。2026-09-23 已导入用户提供的另一组 ChArUco 实测标定并作为默认，
> 初始姿态无相交、最小间隙 12.14 mm。当前配置和命令说明以 [HANDEYE_20260907_IMPORT.md](HANDEYE_20260907_IMPORT.md) 为准。

## 结论（2026-09-22）

默认保留截图中的实测外参，不根据照片猜测额外平移，也不单独移动外壳。
截图已经确认 `eye_in_hand`、`base_link`、`tool0`、`camera_color_optical_frame`、`camera_marker`。
本机 easy_handeye2 的采样器读取 `base_link <- tool0` 和 `camera_color_optical_frame <- camera_marker`；
后端返回相机在末端中的位姿，发布器直接以 `tool0` 为父、彩色 optical frame 为子。
因此这里不应将标定结果求逆，也不应额外交换四元数 xyzw。

当前数字与当前模型几何不相容。相交已通过 Gazebo MeshManager 加载网格、FCL 三角网格查询确认，
不是截图遮挡造成的错觉。原始采样文件不存在，无法从单张截图重算正确外参。

## 机械臂模型如何生成

`ur7e_base.urdf.xacro` 是展开后的 UR 模型：每个关节 origin 和关节角确定 link 位姿，
每个 visual/collision 的 origin 再把网格放到对应 link 中。

`T_world_mesh = T_world_link * T_link_visual_origin * T_mesh_internal`

末端 wrist3 的 mesh origin 是 `xyz="0 0 -0.0989" rpy="pi/2 0 0"`。
这不是应删除的“多余 98.9 mm”：它将网格自身的建模原点对齐法兰坐标系。
仓库 wrist3 的显示 DAE、碰撞 STL 与本机 `/opt/ros/humble/share/ur_description/meshes/ur5e/` 文件逐字节一致，
UR7e 的已安装 visual_parameters.yaml 本来就引用这些 UR5e 网格并使用 -0.0989 偏移。

`wrist_3_link -> flange -> tool0` 的两次旋转相抵、平移为零；当前模型中 wrist_3_link 与 tool0 重合。
flange 与 tool0 原点相同，但方向不同。不能把相同数值任意改挂在 flange 下。

末端显示网格在 tool0 中的包围盒（mm）约为：

| 轴 | 最小 | 最大 |
|---|---:|---:|
| X | -37.55 | 37.55 |
| Y | -43.52 | 38.05 |
| Z | -45.75 | 0.04 |

## 相机模型如何生成

手眼外参约束的是彩色光学中心，不是相机安装孔或外壳中心。
当前链为 `tool0 -> color_optical -> bottom_screw -> camera_link`。
其内部关系由仓库 D435 模型给出：

- 安装孔到 camera_link：平移 `(0.0106, 0.0175, 0.0125)` m。
- camera_link 到 color_frame：平移 `(0, 0.015, 0)` m。
- color_frame 到 color_optical：RPY `(-pi/2, 0, -pi/2)`。
- 合成后取逆：color_optical 到安装孔，平移 `(0.0325, 0.0125, -0.0106)` m，RPY `(pi/2, -pi/2, 0)`。

这条链的矩阵闭合误差已在 1e-12 容差内验证。此前 0.012525 已改为 0.0125，只有 0.025 mm 差异。

相机 DAE 尺寸约 89.914 × 25 × 25.055 mm；它的原点在前面板附近，并非外壳几何中心。
显示 origin 把前面板放在 camera_link 的 X=4.3 mm 处，外壳向后延伸约 25.05 mm。
原 collision box 却以 X=0 为中心，前后偏了 8.225 mm。
现在碰撞盒及不使用 mesh 时的外壳 box 都改为 X=-8.225 mm，对齐显示网格的名义外形。
这修复碰撞几何，但不会移动 optical frame，也不会消除原始外参对应的真实网格相交。

按实测外参生成的相机显示网格包围盒（mm）约为：

| 轴 | 最小 | 最大 |
|---|---:|---:|
| X | -72.96 | 3.75 |
| Y | -69.72 | 4.29 |
| Z | -24.19 | 1.69 |

包围盒重叠只是初筛；FCL 进一步确认相机显示网格和碰撞盒均与 wrist3 的显示/碰撞网格相交。
详见 `measured_mount_geometry.json`。检查覆盖初始姿态下 28 对相机/机械臂几何组合，不是全工作空间碰撞证明。

## 原始标定工作流中发现的问题

本机 `/home/czz/realsense_ws/src/aruco_ros/aruco_ros/launch/single.launch.py` 将
`/camera/color/image_raw` 当作 `image_is_rectified=True` 输入。
同一源码中的 `rosCameraInfo2ArucoCamParams` 在该模式下使用 P，并将畸变系数设为零。
如果原始彩色图像的畸变非零，这个输入约定不正确；实际影响还要看相机 CameraInfo，不能仅凭此认定全部偏差的来源。

不能只把它改成 False：该工作区的未校正分支还存在 K 矩阵索引错误，以及只接受 4 个畸变系数的问题。
本库的修复入口先利用 OpenCV 对原始图像做去畸变，发布匹配的新 K/P 和零 D，再交给检测器的 rectified 分支。
它不修改外部工作区，也不需要安装 image_proc。当前支持 D435 彩色 pinhole 模型、单位 rectification rotation。
原始时间戳、frame_id 和图像尺寸会保留，Image 与 CameraInfo 的 frame/尺寸不一致时不处理。
入口同时订阅 `/aruco_single/pose`，激活该版本 aruco_ros 的按需检测；仅有 TF 监听者不会触发它的图像处理。

本机保存文件 `~/.ros2/easy_handeye2/calibrations/my_eih_calib.calib` 与截图平移仅差 0.645 mm，
不是明显嵌入的解释，默认没有擅自替换截图中的外参。

## 可执行的处理流程

### 1. 保留实测结果启动和诊断

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./run_ur7e_d435_scene.sh
```

先关闭旧场景。启动会明确报告相交，但按用户要求保留实测结果并继续启动。
在另一个已 source 本工作区的终端执行：

```bash
ros2 run ur7e_d435_gazebo verify_scene.py --output /tmp/ur7e_d435_scene_check.json
```

当前实测结果预期 `runtime_result=PASS`、`result=FAIL_GEOMETRY`、退出码 1。
运行正常不再被报告为安装几何通过。

### 2. 核查真实机器人和标记

在真实机器人环境，而不是仿真中读取：

```bash
ros2 run tf2_ros tf2_echo wrist_3_link tool0
ros2 run tf2_ros tf2_echo flange tool0
ros2 topic echo /camera/color/camera_info --once
```

对比实际机器人 URDF、机型、机器人自身运动学标定配置。
核对是否另有发布者重新定义 tool0，或把示教器中非零 TCP 当作 tool0 发布。
示教器 TCP 本身不必然改变 ROS tool0，须以实际 TF 链为准。
量取 ArUco 黑色方形外边长（不包含白边），与检测器 marker_size 对照。
拍照无法代替这些实测值。

### 3. 使用修正后的标记输入重新采样

真实相机和机器人应已启动；停止旧 aruco_single，避免重复发布 camera_marker。
此过程不要同时运行使用相同 TF 名称的 Gazebo。

```bash
source /opt/ros/humble/setup.bash
source /home/czz/realsense_ws/install/local_setup.bash
source /home/czz/perception_eye_in_hand/install/local_setup.bash
# 只有实测黑边边长确为 0.10 m 时才使用下面的数值：
ros2 launch ur7e_d435_gazebo calibration_marker.launch.py marker_size:=0.10 marker_id:=582
```

另一个相同环境的终端启动原 easy_handeye2：

```bash
ros2 launch easy_handeye2 handeye_calibrate.launch.py
```

保持标记板固定；采集不同方向的旋转和不同位置，每次等机械臂及标记检测稳定后取样。
避免全部样本只做平移或绕同一轴旋转。留出独立姿态作验证，不仅比较参与拟合的样本。
相机/支架和标记板不能在过程中松动。

### 4. 保存样本，核对残差和算法一致性

保存标定结果不等于保存原始采样。标定服务运行时另行保存样本：

```bash
ros2 service call /easy_handeye2/calibration/save_samples easy_handeye2_msgs/srv/SaveSamples '{}'
ros2 run ur7e_d435_gazebo evaluate_handeye.py \
  /home/czz/.ros2/easy_handeye2/samples/my_eih_calib.samples \
  /home/czz/.ros2/easy_handeye2/calibrations/my_eih_calib.calib \
  --output /tmp/handeye_residuals.json
```

工具检查每个样本的 `T_base_tool * T_tool_camera * T_camera_marker` 是否为同一个固定标记位姿，
输出平移/旋转残差、各样本误差、旋转激励奇异值，以及五种算法的结果。
本机 easy_handeye2 版本可能保存空的样本 frame 元数据；工具会明确标注采用了给定标定文件的坐标系假设，
此时需要核对原始采样设置。若保存的 frame 非空却与标定文件不一致，工具会拒绝比较。
残差较小不是单独的合格证明：还应比较独立验证姿态、相机安装孔的实测位置和网格间隙。
不要自动选择训练残差最小的算法并当成真值。

### 5. 使用新标定结果验证

```bash
./run_ur7e_d435_scene.sh calibration_file:=/home/czz/.ros2/easy_handeye2/calibrations/my_eih_calib.calib
# 另一终端：
ros2 run ur7e_d435_gazebo verify_scene.py \
  --calibration /home/czz/.ros2/easy_handeye2/calibrations/my_eih_calib.calib
```

外壳、相机传感器、光学 TF 和点云坐标始终随同一外参一起变化。
若正确标定后仍与实际支架几何不符，应核对真实相机内部 TF、机型和末端工具定义，不能仅调整 visual origin。

### 可选的无碰撞仿真对照

```bash
./run_ur7e_d435_scene.sh calibration_file:=/home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/config/ur7e_d435_sim_clearance.yaml
```

这是单独的设计位姿，不是修正后的实测外参。初始姿态全部检查组合无相交，最小间隙约 34.99 mm；
见 `simulation_mount_geometry.json`。用户要求保留实测默认，因此没有自动切换到该配置。

## 验证代码

```bash
source /opt/ros/humble/setup.bash
source /home/czz/perception_eye_in_hand/install/local_setup.bash
python3 -m unittest discover -s /home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/test -v
```

覆盖外参及内部链、碰撞盒位置、实测相交检出、仿真配置间隙、去畸变坐标闭合、合成手眼样本求解及错误外参检出。
真实相机上的重新采样和独立姿态验证仍需现场执行。

2026-09-22 已通过 5 项回归测试，并用独立 ROS 域运行实际新标定入口：合成 582 号标记，
设定边长 0.10 m、理论深度 0.30 m，经去畸变与 ArUco 检测输出深度 0.30021 m。
默认场景的实际消息/TF/控制器运行验证通过，相交诊断如预期返回 FAIL_GEOMETRY，
记录见 `measured_mount_runtime.json`。这不代替真实相机的标定质量验证。
