# 三车动捕实验

首次现场复验直接按 [现场测试清单](field-test.md) 操作；下文保留完整实验说明。

本轮从原 main 摘取有用的新功能，不转换原有无动捕实车 demo，也不增加 Gazebo 实验。Mini 三车仍为 `tianbot_mini_01/02/03`，动捕刚体默认同名，服务器和端口统一在 `config/mocap.yaml`。NOKOV 是动捕合作伙伴，当前通过 VRPN 接入。

## 从哪里找

- 原视频脚本仍在 `nodes/`，原九个文件不动；四个新可执行节点也在同一目录。原课程 launch 保持原位。
- 新动捕入口都在 `launch/mocap/` 一层，名称全部为 `demo_mocap_*`。
- 不再单独设置 `scripts/`；leader-follower、PID 对照、预设路径跟踪、统一运行节点都从 `nodes/` 查找。
- 少量控制计算在 `src/abc_swarm/` 一层，供实车和未来的仿真复用，不带 sim/mocap 后缀。
- 新公共组合在 `launch/common/`；原 `launch/includes/` 没有改动。

参考原版 `demo_formation_leader_follower_path_tracking.launch`：完整实验应组合设备、定位、编队、头车路径和显示。该原文件实际使用雷达地图和 AMCL，因此保留原名、位置和内容，不把它改名为仿真。本轮复用这种组合方式，定位替换为动捕；头车路径跟踪和后车跟随仍保持原有解耦。

## 准备与摆位

终端加载 `/opt/ros/noetic/setup.bash` 和 `~/tianbot_mini_ws/devel/setup.bash`。新增脚本后按根 README 编译本包；所需外部驱动快照在 `dependencies.repos`，RMTT 依赖只供原空地实验参考。

动捕应持续发布 world 下刚体，原点对应底盘控制参考点，x 向前、y 向左、z 向上。leader-follower 还需前车带有效时间戳的实际 odom 速度。若车名与刚体不同，用 `robot_name_N`、`rigid_body_N` 分别指定。

三角形：三车同向，02 在 01 后方 0.5 m、左侧 0.5 m，03 在后方 0.5 m、右侧 0.5 m。拓扑为 01→02、01→03。

逐车跟随：三车同向纵列，相邻车距 0.5 m。拓扑为 01→02→03，03 的速度前馈来自 02。转弯可能切弯，不保证严格复走头车历史路径。

## 手动领航

选择一个入口，切换前先停止并退出旧实验：

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch
```

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_snake.launch
```

默认同时启动 socket 驱动、VRPN、控制器和键盘终端；反馈有效后自动运行，后车可立即向编队位置收敛。已有后台时传 `start_driver:=false start_vrpn:=false`。只保留一套控制同组车辆的算法。

启动前摆好车并确认活动空间；启动后观察统一状态：

```bash
rostopic echo /abc_swarm_demo/status
```

反馈就绪后自动运行：


键盘 i 前进，u/o 缓弯，k 停止。遥控请求初始 0.15 m/s、0.15 rad/s；统一入口超过 0.3 秒没有新指令时停车。旧键盘版本需要持续按键。GUI 接管时关闭默认键盘，并向 `/abc_swarm_demo/teleop_cmd` 发布，不能另开节点直接争用头车 cmd_vel。

停止整套实验：在 roslaunch 终端按 Ctrl+C，并确认三车静止。


键盘 k 只停止头车请求，后车仍可收敛；退出实验进程停止整套控制。PID 对照入口 `demo_mocap_three_mini_pid_tracking.launch` 采用相同启动和操作接口，但算法是目标点 PID，便于比较。

## 圆形与圆角矩形路径

首次验证先使用单车：

```bash
roslaunch abc_swarm demo_mocap_path_tracking.launch shape:=circle
```

三角形：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch shape:=circle
```

逐车跟随：

```bash
roslaunch abc_swarm demo_mocap_snake_leader_follower_path_tracking.launch shape:=rounded_rectangle
```

默认启动驱动、VRPN、算法和 RViz，不启动键盘。头车首次有效位姿确定路径起点和切线方向，路径预览后固定在 world，不随头车移动。默认逆时针，圆在车头左侧；`clockwise:=true` 改为右侧。

默认圆半径 0.8 m；圆角矩形外轮廓宽 2.5 m、高 2.0 m、圆角半径 0.8 m；头车 0.15 m/s、一圈。可通过 `radius`、`width`、`height`、`speed`、`laps`、`clockwise`、`spacing` 调整。`front_distance` 同时传给跟随器和收敛判断，默认 0.1 m。路径几何、速度和圈数由 launch 参数负责；前视距离、容差等算法参数在 `config/path_tracking.yaml`，不再重复声明同名默认值。矩形宽高必须至少为圆角半径两倍。后车和车体尺寸会超出头车路径，先确认整个队形扫过范围；外侧车需要更大速度余量。

RViz 使用 world，显示 `/abc_swarm_path/path`、`actual_path`、`target` 和 TF。启动前确认完整编队活动空间和摆位；反馈有效后自动运行，RViz 显示运行路径。

正常一圈后头车停下，等待后车前置控制点误差小于 0.1 m，或达到 30 秒上限，再共同停止。`laps:=0` 连续运行，需要人工停止。新一轮可以重启 launch，也可在 completed/fault 后重置：

```bash
rosservice call /abc_swarm_demo/reset
```

重置接受后按实时头车位置重建路径并自动运行。运行期间不能重置；路径故障后不会自动恢复旧轨迹，需重启 launch 或重置。

## 边界

控制器没有避障。leader-follower 控制车体前方 0.1 m 的点，该点误差与车体参考点误差不同，不单独约束最终朝向。TF 时间检查也不能识别服务器用新时间戳反复发送冻结姿态；底盘急停、模式和断网停车仍须现场确认。

这一分支完成静态与合成反馈验证后，仍需现场复验。过去三车手动领航的测试不能代替新分支和自动路径实验的实际验收。统一参数、服务和 GUI 生命周期见 [运行接口](demo-runtime.md)。


当前默认直线速度为 0.15 m/s：手动领航的键盘请求、运行节点及后车限幅共同使用 max_linear；路径 speed 默认为 0.15，后车限幅同为 0.15。点镇定详见[任务选择](point-stabilization.md)。0.15 是请求/上限，转弯与接近目标会减速；跟随算法还会按误差调整速度，不能保证每车每时刻都以同一速度运动。外侧车辆受相同限幅时，提速转弯会增大跟随误差，需现场验收。
