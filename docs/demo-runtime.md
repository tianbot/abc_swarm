# 动捕 demo 的启动与 GUI 接口

适用于 `launch/mocap/` 六个新增 Mini 实验。原视频入口保持原操作，本轮不增加 Gazebo 实验。

## 启动即运行

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch
```

一个文件组合驱动、VRPN、算法及键盘/RViz。**反馈有效后自动运行，不需要调用启用 service。后车可能在头车静止时立即向编队位置运动。** 请在启动前准备场地和摆位；当前没有避障，也没有独立的自动入队规划阶段。

- 手动实验：头车由键盘控制，02/03 自动跟踪编队目标；k 或键盘超时只停止头车请求，后车仍可收敛。
- 自动路径：初次有效位姿确定路径起点，反馈及路径准备完成后自动跟踪。RViz 是运行时显示，不是启动前的确认门。先单车、低速验证，场地必须提前准备好。
- 停止整套实验：在其 roslaunch 终端按 Ctrl+C，确认三车实际静止。切换实验时先退出旧进程，再启动新进程。

2026-10-24 移除了此前未经确认加入的 `/abc_swarm_demo/set_enabled` 服务和 `enabled:=` 启动选项。旧脚本或生成目录若仍包含该步骤，应重新生成。`/abc_swarm_demo/enabled` 现在仅为节点之间的内部反馈有效标志，由运行节点维护；不要手动写它。

## 后台和场地

| 参数 | 用途 |
| --- | --- |
| site_config | 场地 YAML，默认 config/mocap.yaml |
| vrpn_server_ip / vrpn_port | 覆盖场地服务器/端口 |
| start_driver / start_vrpn | 默认 true；复用已有后台时 false |
| start_teleop | 手动实验默认 true；GUI 遥控时 false |
| use_rviz | 手动默认 false，自动默认 true |

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch start_driver:=false start_vrpn:=false
```

VRPN 服务器优先级：命令行 > ABC_SWARM_VRPN_SERVER 环境变量 > 场地配置；端口为命令行 > 场地配置。

```bash
roslaunch abc_swarm mocap_backend.launch site_config:=/absolute/path/site.yaml
```

后台可以常驻，GUI 启停实验进程时传 start_driver/start_vrpn 为 false。同一组车辆只运行一套实验，不能让多个控制器争用 cmd_vel。

## 反馈和恢复

```bash
rostopic echo /abc_swarm_demo/status
```

waiting 表示等待必需反馈；running 表示反馈有效且控制正在运行，并不表示车轮一定在转；completed 表示路径结束；fault 表示路径故障锁定。

- odom 默认有效期 0.5 秒，同时检查消息时间戳和本机接收时间。允许时间戳小幅超前 0.1 秒，兼容现场 rosserial 同步抖动，仍拒绝过旧数据、长时间没有接收及非有限速度。
- 手动实验：反馈失效时共同停止；反馈恢复后自动恢复跟随。头车旧键盘请求不会重放，需要新输入。
- 自动路径：反馈失效或路径异常时停车并锁定，避免自行恢复旧轨迹；完成后也不会重复跑圈。重新测试可退出并重新启动 launch；或者在 completed/fault 时使用可选的路径重置。

```bash
rosservice call /abc_swarm_demo/reset
```

重置接受后按实时头车位置重建路径并自动运行，不需要再启用；不要在车辆运行时请求重置。手动实验不支持路径重置。

## GUI 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| /abc_swarm_demo/status | std_msgs/String，JSON，latched | 状态和原因，变化时发布，不是心跳 |
| /abc_swarm_demo/teleop_cmd | geometry_msgs/Twist | 手动实验头车请求 |
| /abc_swarm_demo/reset | std_srvs/Trigger | 路径完成/故障后的可选重试 |

JSON schema_version=2，保留 state、reason、enabled、ready、mode、vrpn_server、vrpn_port。enabled 表示自动反馈门是否打开，不再表示操作员启用状态。GUI 用实验进程的启动/退出控制运行，不把 ROS service 调用作为开车前的必经步骤。

键盘和 GUI 都通过 teleop_cmd 进入头车统一输出；GUI 使用 start_teleop:=false。输入超时 0.3 秒停止头车请求，默认限制 0.15 m/s、0.6 rad/s。路径实验的头车最终输出仍只由路径跟踪器发布。

## 后续功能边界

目标链条是：实时定位 → 分配每车目标/入队位置 → 点镇定或路径跟踪给出期望运动 → 避障修正 → 底盘控制。现有 LF 已能向相对编队目标收敛，但不等同于无碰撞的入队规划。

点镇定入队和 ORCA 是后续独立实验/算法，不在本次实现中。引入时要明确机器人外形、定位输入、邻车状态、静态障碍来源及差速底盘约束；不能直接把全向速度输出当作 Mini 的 v/omega。旧课程和现有 LF 核心算法继续保留。
