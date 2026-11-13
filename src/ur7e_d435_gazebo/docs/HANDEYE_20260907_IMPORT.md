# 2026-09-07 ChArUco 标定导入与安装验证

2026-09-23 更新：默认使用用户新提供的实测标定，不使用人为设计的无碰撞位姿。

## 两份文件的含义

- `legacy_calibration.yaml` 的 `gripper_T_camera` 平移为毫米。
- `calibration_20260907.json` 的同名矩阵平移为米，旋转保留更多有效数字。
- 将 YAML 平移乘 0.001 后，两份矩阵在导出精度内一致。
- 用户确认 gripper 就是 tool0，因此不补工具长度、不求逆。
- 根据彩色 ChArUco 重投影标定的定义，将 camera 解释为彩色光学坐标系；源文件本身没有 ROS camera frame 名称，这项映射在生成配置中明确记录。

JSON 是生成时的数值来源，YAML 用作交叉校验。旋转矩阵只做消除小数导出舍入误差的正交化，
再转成 xyzw 四元数，没有施加物理旋转补偿。

导入结果，`tool0 -> camera_color_optical_frame`：

```yaml
translation: {x: -0.029901376, y: -0.069185919, z: -0.012263443}
rotation: {x: -0.27259021854810417, y: -0.6513429981602794, z: 0.6549447692495959, w: 0.26924750831171806}
```

JSON 记录 22 张 ChArUco 图像，棋盘为 DICT_4X4_1000、7×5、方格 25 mm、标记 18 mm。
实际外参优化的角点重投影 RMSE 为 **2.1793760746 px**。
**0.1669671071 px 是未部署的自由内参拟合结果，不能作为这组外参的精度。**
文件中没有逐帧机器人位姿、角点观测和图像，因此本次没有重新优化或独立复算该 RMSE。

记录的 1920×1080 彩色内参畸变系数全为零，所以不能再将“忽略非零畸变”当作这组结果的已知误差来源。
原来的 ArUco 单标记重标定入口与这里的 ChArUco 棋盘不是同一套采样流程，不能照搬 marker_id=582 复现本次标定。

## 修改范围

1. 原始两份文件原样保存到 `config/handeye_20260907/`，不执行其中的部署或控制设置。
2. 新增 `scripts/import_handeye_20260907.py`，校验单位、两个矩阵、旋转合法性及内参一致性。
3. 生成 `config/ur7e_d435_handeye_20260907.yaml`，统一使用米和 xyzw，保留来源和精度记录。
4. launch、直接 xacro 展开、运行验证脚本的默认路径统一改为新配置。
5. 原有 `ur7e_d435_side_sim_mount.yaml` 保持用户现有内容，不覆盖；可显式切换作对照。
6. 机械臂模型、相机外壳、内部 TF 及传感器安装关系不作额外位移，整台相机随新外参移动。

1920×1080 彩色 K 只保存在 `reference_color_intrinsics` 中，不直接应用到当前 1280×720 的仿真图像，
也不应用到深度相机。仿真相机内参仍由既有仿真模型生成；本次解决的是外参安装位置，
尚未将仿真成像模型改成这份真实彩色 CameraInfo。
`base_T_table` 和玻璃对齐控制参数保留在原始文件中，不用于相机挂载或改变场景中的桌面/障碍物。

## 几何结果

Gazebo MeshManager + FCL 检查相机实际 DAE 与碰撞盒，对比机械臂显示/碰撞几何：

- 初始关节姿态下，28 对几何组合全部无相交。
- 最小间隙 **12.1419 mm**。
- 证据：`handeye_20260907_geometry.json`。
- 隔离 ROS 域中的实际启动验证：`result=PASS`、`runtime_result=PASS`，图像、点云、TF 和控制器正常；
  证据：`handeye_20260907_runtime.json`。九项回归测试全部通过。

这一间隙由实测外参直接得到，没有附加“防穿模”平移。
它表明该标定结果解决了当前模型中的相机/机械臂相交，不代表全部运动姿态均无碰撞，
也不代表已经验证相机在真实空间中的毫米级定位精度。实际支架没有被完整建模。

## 运行与复现

默认启动：

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./run_ur7e_d435_scene.sh
```

关闭旧场景后再启动。加载当前工作区的另一个终端中运行：

```bash
source /opt/ros/humble/setup.bash
source /home/czz/perception_eye_in_hand/install/local_setup.bash
ros2 run ur7e_d435_gazebo verify_scene.py --output /tmp/handeye_20260907_runtime.json
```

重新生成配置：

```bash
ros2 run ur7e_d435_gazebo import_handeye_20260907.py \
  /home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/config/handeye_20260907/legacy_calibration.yaml \
  /home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/config/handeye_20260907/calibration_20260907.json \
  /home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/config/ur7e_d435_handeye_20260907.yaml
```

对照旧外参：

```bash
./run_ur7e_d435_scene.sh calibration_file:=/home/czz/perception_eye_in_hand/src/ur7e_d435_gazebo/config/ur7e_d435_side_sim_mount.yaml
```
