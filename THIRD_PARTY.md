# 来源与版本

项目新增Python/C++/CUDA适配代码采用Apache-2.0。官方文件、依赖和资产保留各自许可证/版权标头，不能将项目许可证解释为替换第三方条款。

- `vendor/nvblox/include`与`vendor/nvblox_torch/cpp/include`：来自NVIDIA `nvidia-isaac/nvblox`提交`e4a2b00f45ecf989682d72a33a2c29defe39fd2d`，与本机wheel的`__git_sha__`一致。仅保留编译头文件，未复制整套示例或模型。原LICENSE见`vendor/LICENSE.md`；头文件原声明保留。
- `vendor/stdgpu`：从本机已安装的stdgpu编译头文件复制，保留原版权标头，用于解释Python Mapper头文件中的GPU哈希类型。文件在本工程内，不从原workspaces运行时读取，也不链接其stdgpu/nvblox二进制。
- `src/ur7e_d435_gazebo`：复制自相邻新工程的机器人与D435场景包，模型、插件、标定与资产来源见其`THIRD_PARTY.md`及licenses目录。未改机械臂场景与手眼标定。
- `tools/motion_plan.json`与仿真观察路径：沿用此前已检查的高位横向路径，完整参数随工程保存，动作工具固定隔离域并检查仿真/控制器反馈。
- Python后端依赖安装后的wheel自带core与Torch绑定；原库不被修改，桥接仅增加独立Torch算子。

已验证平台为Ubuntu22.04、ROS2 Humble、Python3.10、CUDA Toolkit12.6、PyTorch2.9.1+cu128、nvblox_torch0.0.10.dev1+cu12ubuntu22及RTX4060 Laptop（架构89）。build时检查wheel源码提交和PyTorch CUDA可用性；换wheel需同步适配/验证桥接头文件，不能绕过版本检查混用二进制。

本工程无需安装cuVSLAM、实机RealSense SDK（只做Gazebo时）或人物识别模型。wheel本身的框架类型/依赖不等于本项目启动了这些模型。
