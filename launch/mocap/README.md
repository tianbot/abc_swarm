# 动捕实验入口

文件保持一层，按效果选择；正式三车入口统一以 `demo_mocap_` 开头。

| 实验 | 文件 |
| --- | --- |
| 手动头车，三角形跟随 | `demo_mocap_three_mini_leader_follower_triangle.launch` |
| 手动头车，逐车跟随 | `demo_mocap_three_mini_leader_follower_snake.launch` |
| 三车 PID 对照 | `demo_mocap_three_mini_pid_tracking.launch` |
| 单车预设路径 | `demo_mocap_path_tracking.launch` |
| 预设路径 + 三角形 leader-follower | `demo_mocap_formation_leader_follower_path_tracking.launch` |
| 预设路径 + 逐车 leader-follower | `demo_mocap_snake_leader_follower_path_tracking.launch` |

这些新实验用一个文件组合驱动、VRPN、算法和交互，默认禁用运动。后台已运行时显式关闭重复启动。详见 [操作与摆位](../../docs/mocap-demos.md) 和 [GUI 接口](../../docs/demo-runtime.md)。

`mocap_backend.launch` 是可独立保持的公共后台，供 GUI 或多个顺序运行的实验复用。

原 `demo_mc_tf_tt_follow_mini.launch`、`demo_mc_tf_tt_circle_mini.launch` 移至此处，改为 `demo_mocap_tf_tt_*`；内容、参数、设备准备流程保持原版。这两个是原有空地协同参考，不属于新三车统一运行协议，也不自动管理飞行器起降。
