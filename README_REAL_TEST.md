# 实机运行README：UR7e + D435i手动拖动重建

后续记录：[恢复后的30秒实机测试结果与分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_TEST_RESULTS_AND_ANALYSIS_20261111.md)。原故障诊断保留当时状态，恢复后结果单独归档。


日期：2026-11-11。使用`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`，不是旁边的`nvblox_in_hand`，不使用workspaces环境。本文生成运行指令，没有代替用户启动硬件或发送机械臂运动。

## 1. 本轮配置

- UR7e控制柜：192.168.56.100；PC回连：192.168.56.38（执行路由检查确认）。
- 工厂校准：`/home/czz/ur7e_real_test/ur7e_factory_calibration.yaml`，本机文件已确认存在。
- D435i序列号：349622072343，沿用上次设备。
- 使用ROS域135；与前面仿真域164隔离，所有实机终端必须一致。
- 深度和彩色640×480×30；原始深度不对齐到Color，使用各自CameraInfo及光学TF。
- 保持当前5mm体素、截断4体素=2cm、3D ESDF、stride2、融合上限40Hz、ESDF更新/显示上限33.4Hz、Mesh5Hz。
- 使用真实时间，不需要/clock。机械臂由操作员使用现场已有的Freedrive方式手动拖动，不发送轨迹命令，不运行仿真扫描脚本。

若机器人驱动、相机、静态TF已在同一135域正常运行，保留它们，不重复启动同类节点。手眼参数仅适用于上次同一相机及未改变的固定安装；重新拆装后不能仅凭同一机械臂就沿用标定。

## 2. 每个新终端先执行

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
```

所有后续终端均先执行此环境块。不要再source workspaces或其他感知包的install/setup.bash。不需要修改Gazebo端口，因为本次不启动Gazebo。

已经编译过当前代码，无需为实机输入重新编译；若自行修改过源文件且未构建，可在硬件启动前执行`./build.sh`。

## 3. 终端A：网络与机械臂驱动

```bash
ip route get 192.168.56.100
ping -c 3 192.168.56.100
ls -lh /home/czz/ur7e_real_test/ur7e_factory_calibration.yaml
```

路由输出应经连接控制柜的网卡，`src`应为192.168.56.38；如果不同，后面reverse_ip与示教器External Control主机地址应使用实际PC地址，不能直接照抄38。ROS_LOCALHOST_ONLY只限制DDS，不阻止驱动TCP连接控制柜。

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=192.168.56.100 \
  reverse_ip:=192.168.56.38 \
  kinematics_params_file:=/home/czz/ur7e_real_test/ur7e_factory_calibration.yaml \
  use_fake_hardware:=false \
  headless_mode:=false \
  activate_joint_controller:=false \
  launch_rviz:=false
```

示教器沿用上次External Control配置，根据既有现场操作完成驱动连接。按[UR官方启动流程](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/startup.html)核对校准成功；本机参数按已安装Humble launch核对，不照搬新版`use_mock_hardware`参数。

`activate_joint_controller=false`是不激活轨迹控制器，并不会自动进入Freedrive。拖动仍由操作员按该设备已有的示教器/按钮流程操作；Freedrive期间必须保持关节反馈及TF持续更新。如果既有程序与Freedrive操作互相影响，先按现场流程处理，不通过发送ROS轨迹替代手动拖动。

## 4. 终端B：D435i真实相机

先执行环境块。确保没有其他程序占用同一相机，再运行：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  serial_no:=_349622072343 \
  enable_depth:=true \
  enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  rgb_camera.color_format:=RGB8 \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  enable_sync:=true \
  publish_tf:=true
```

不另外开启相机SDK点云。禁用infra/IMU不影响D435i自身生成深度；本轮位姿来自机械臂，不运行视觉SLAM。

若再次出现`librealsense2.so... not found`，是驱动/SDK加载失败，后续建图不能弥补；保留错误日志核对当前环境依赖，不要同时切换多个环境尝试。

## 5. 终端C：安装外参TF

先执行环境块。仅在没有现成tool0→camera_link安装变换时启动下列节点。相机驱动继续负责camera_link到Depth/Color光学frame的内部关系。

```bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id tool0 \
  --child-frame-id camera_link \
  --x -0.0406379061113624 \
  --y -0.05902311421350448 \
  --z -0.012384254831755863 \
  --qx 0.0025872583947125077 \
  --qy -0.002933202909281628 \
  --qz 0.9251005977793049 \
  --qw -0.3797019705019513
```

该终端持续运行，不能在验证前Ctrl+C。此组是上次已组合的tool0→camera_link变换；手眼原始标定针对Color，不能把同一组数值改名直接当tool0→Depth。不要再发布一条tool0→camera_color_optical_frame与相机内部TF争用child。

## 6. 终端D：确认真实输入与TF

每条持续运行命令分别执行，Ctrl+C后换下一条：

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 topic hz /joint_states --wall-time

ros2 topic echo /camera/camera/depth/camera_info --once --qos-reliability best_effort
ros2 topic echo /camera/camera/color/camera_info --once --qos-reliability best_effort
ros2 topic hz /camera/camera/depth/image_rect_raw --wall-time
ros2 topic hz /camera/camera/color/image_raw --wall-time

ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

手动小幅拖动时，两条base→camera位姿应随关节变化。Color与Depth的内外参分别使用，这是相机模型要求；不是又将手眼标定目标更改成Depth。

## 7. 终端E：启动当前5mm建图与RViz

先执行环境块，使用显式参数避免继承仿真输入：

```bash
./run.sh \
  start_scene:=false \
  launch_camera:=false \
  use_sim_time:=false \
  run_rviz:=true \
  rviz_mode:=diagnostic \
  input_qos:=sensor_data \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  depth_info_topic:=/camera/camera/depth/camera_info \
  color_topic:=/camera/camera/color/image_raw \
  color_info_topic:=/camera/camera/color/camera_info \
  global_frame:=base_link \
  voxel_size:=0.005 \
  truncation_vox:=4.0 \
  esdf_mode:=3d \
  esdf_view_mode:=observed \
  esdf_view_stride:=2 \
  enable_depth:=true \
  enable_color:=true \
  enable_esdf:=true \
  enable_esdf_view:=true \
  enable_mesh:=true \
  depth_hz:=40.0 \
  color_hz:=40.0 \
  esdf_hz:=33.4 \
  esdf_view_hz:=33.4 \
  mesh_hz:=5.0 \
  min_depth_m:=0.2 \
  max_range_m:=2.05 \
  decay_hz:=0.0 \
  clearing_hz:=0.0 \
  workspace_bounds_type:=bounding_box \
  workspace_min_x:=-0.1 \
  workspace_max_x:=1.1 \
  workspace_min_y:=-0.6 \
  workspace_max_y:=0.6 \
  workspace_min_z:=-0.05 \
  workspace_max_z:=1.05
```

工作区是**真实base_link坐标下的范围**，不是自动覆盖相机视场。上述数值沿用当前有限工作区以限制5mm资源量，不能保证现场工作台恰好落在其中。先确认目标处于该范围；如果真实物体位于负x或更低的z，应按实际位置更改相应边界再启动，不要盲目改成unbounded。

此命令不加载旧`docs/examples/real_ur7e_d435i.yaml`，因为它指定3cm体素并默认关闭颜色。当前命令明确启用颜色、5mm和真实话题。启动代码会给RViz中的原仿真图像话题做真实话题重映射，不需要修改源RViz文件。

RViz Fixed Frame选base_link：

| 查看内容 | 话题/显示类型 |
|---|---|
| 彩色Mesh | /ur7e_nvblox/mesh，MarkerArray |
| 3D ESDF | /ur7e_nvblox/esdf_points，PointCloud2 |
| RGB | /camera/camera/color/image_raw，Image |
| 原生TSDF层 | /ur7e_nvblox/tsdf_layer，PointCloud2 |
| 机械臂相机轨迹 | /ur7e_nvblox/camera_path，Path |

先静止5～10秒建立初始地图，再手动平缓改变视角。看Mesh时暂时取消ESDF显示，反之亦然；无需运行任何view_sweep/orbit_scan脚本。UR驱动模型没有相机外观并不代表相机TF缺失；不要另起Gazebo机器人模型去补外观。

## 8. 终端F：实时性检查

先执行环境块：

```bash
ros2 param get /ur7e_nvblox use_sim_time
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox depth_topic
ros2 param get /ur7e_nvblox color_topic
python3 tools/watch_realtime.py
```

应为false、0.005、真实双层camera前缀。depth_fuse/color_fuse是融合调用，esdf_update是更新调用，esdf_pub是发布调用。33.4Hz是上限，不是实际达到的保证。

需要实际点云接收频率，Ctrl+C停止轻量查看后运行：

```bash
python3 tools/watch_esdf_hz.py
```

此工具RELIABLE接收完整原始消息，比本机固定BEST_EFFORT的普通topic hz更适合当前大点云。它会增加一个地图订阅和通信开销；看内部指标与测实际接收可分开进行。

## 9. 保存Mesh、ESDF原生地图和测试配置

保持Mapper运行，先停止手动移动，让机械臂静止，再在终端F执行：

```bash
TEST_OUT="/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_manual_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUT"

ros2 service call /ur7e_nvblox/save_ply ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/mesh.ply'}"

ros2 service call /ur7e_nvblox/save_map ur7e_nvblox_interfaces/srv/FilePath \
  "{file_path: '$TEST_OUT/map.blox'}"

ros2 param dump /ur7e_nvblox > "$TEST_OUT/params.yaml"
ros2 topic echo /ur7e_nvblox/metrics --once > "$TEST_OUT/metrics.yaml"
echo "$TEST_OUT"
```

确认服务返回success=true。mesh.ply为可用CloudCompare打开的三角网格；map.blox保留原生TSDF/Color/ESDF地图层，不是独立ESDF PLY。两次保存并非原子同帧快照。

若还要保留实际发布的ESDF点云消息，可在同一终端继续录制约10秒，然后Ctrl+C停止录制：

```bash
ros2 bag record -o "$TEST_OUT/esdf_bag" \
  /ur7e_nvblox/esdf_points /tf /tf_static
```

该bag保存实际显示采样点云，非所有原生体素；体素完整数据仍在map.blox。大点云录制也会增加负载，不把录制期间频率当作无录制性能上限。

保存成功后再Ctrl+C退出建图；保持相机、驱动、静态TF运行或按现场流程逐一关闭。本命令不自动保存地图，不能先结束Mapper再调用保存服务。

## 10. 本轮测试的解释边界

当前仿真中已发现颜色跨表面污染、局部几何细节误差和少量ESDF距离高估，这些问题并未在本轮指令中修复。实机运行是观察与评估，不代表精度已验收；手动拖动也不能消除投影或遮挡判断错误。当前ESDF不用于本次自动运动决策。

若RGB能显示而地图没有更新，依次检查：真实双层话题→CameraInfo→base到两路相机TF→use_sim_time=false→融合计数→工作区是否覆盖目标。不要用改frame字符串、最新TF或启动第二套Mapper来掩盖输入问题。


## 11. clearing参数错误与正常退出

如果启动后出现：

```text
ValueError: clearing requires positive radius
```

表示设置了正的`clearing_hz`，但`clearing_radius_m`仍为默认负值。Mapper参数检查失败退出，启动器随即关闭RViz；这不是机械臂或相机断线。

**本README的默认运行命令应保持：**

```bash
decay_hz:=0.0 clearing_hz:=0.0
```

如果确实需要按相机位置清除远处地图，才同时指定正半径，例如用以下参数替换原命令末尾的清图选项：

```bash
decay_hz:=0.0 clearing_hz:=1.0 clearing_radius_m:=1.0
```

表示每秒检查一次、清理相机半径约1m之外的地图块。边界按块处理；这不是孤立点过滤，也不是专门的运动残影清除。半径小可能丢失已重建区域，不要用它直接修复颜色串色或ESDF突出点。

停止测试时，先让机械臂静止并确认第9节保存成功，再在建图终端Ctrl+C。退出建图不会自动保存地图，也不等于替操作员关闭机器人或退出手拖模式。

## 12. 每次启动前后最短检查

每个终端独立检查：

```bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
```

本轮应分别为135、1、rmw_cyclonedds_cpp。尤其不要漏掉手眼静态TF终端；已经启动的进程不会因其他终端重新export而改变ROS域。

Mapper启动后检查：

```bash
ros2 param get /ur7e_nvblox use_sim_time
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox clearing_hz
python3 tools/watch_realtime.py
```

默认应为false、0.005、0.0，且depth_fuse及esdf_update持续增长。RGB可见、机器人模型可见或打印Mapper ready，都不能单独证明已开始融合。
