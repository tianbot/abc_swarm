# 动捕实验入口

文件保持一层，按效果选择；正式三车入口统一以 `demo_mocap_` 开头。

| 实验 | 文件 |
| --- | --- |
| 手动头车，三角形跟随 | `demo_mocap_three_mini_leader_follower_triangle.launch` |
| 手动头车，逐车跟随 | `demo_mocap_three_mini_leader_follower_snake.launch` |
| 三车点镇定 | `demo_mocap_three_mini_point_stabilization.launch` |
| 三车 ORCA 入位 | `demo_mocap_three_mini_orca_point_stabilization.launch` |
| 三车同圆运动 | `demo_mocap_three_mini_circle_tracking.launch` |
| 三车三角形逐边运动 | `demo_mocap_three_mini_triangle_tracking.launch` |
| 三车 PID 对照 | `demo_mocap_three_mini_pid_tracking.launch` |
| 单车预设路径 | `demo_mocap_path_tracking.launch` |
| 预设路径 + 三角形 leader-follower | `demo_mocap_formation_leader_follower_path_tracking.launch` |
| 预设路径 + 逐车 leader-follower | `demo_mocap_snake_leader_follower_path_tracking.launch` |

这些新实验用一个文件组合驱动、VRPN、算法和交互，反馈有效后自动运行；三角/蛇形跟随默认先自动入位，随后按manual/auto控制头车。后台已运行时显式关闭重复启动。各文件的中文注释说明后台、算法、遥控和显示模块；输入输出和阶段说明见[运行接口](../../docs/demo-runtime.md)。正式操作见[现场用法](../../docs/mocap-field-use.md)。

`mocap_backend.launch` 是可独立保持的公共后台，供 GUI 或多个顺序运行的实验复用。

原 `demo_mc_tf_tt_follow_mini.launch`、`demo_mc_tf_tt_circle_mini.launch` 移至此处，改为 `demo_mocap_tf_tt_*`；内容、参数、设备准备流程保持原版。这两个是原有空地协同参考，不属于新三车统一运行协议，也不自动管理飞行器起降。

这两个原课程文件仍受冻结兼容检查保护，本轮中文注释补在新增入口和公共片段中；原课程说明集中在本页，避免修改视频对应资源。
