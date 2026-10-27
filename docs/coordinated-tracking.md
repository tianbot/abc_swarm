# 三车动捕：同圆绕行与正三角形逐边运动

对应 [issue #8](https://github.com/tianbot/abc_swarm/issues/8)。仅面向三台 Mini 的 VRPN 动捕实验。两个入口共用 Python 算法和现有点镇定节点，由 launch 配置任务，不改旧视频与旧跟随命令。

已确认的行为：同一个圆上三车约120°间隔、同向绕行；正三角形三个顶点每轮各走一条直边，到齐停车后原地转向，再一起走下一条边。

## 自动执行顺序

1. 等待三台动捕位姿有效，冻结整个图形。
2. 点镇定到三台各自起点及出发朝向。所有车都到位并稳定默认0.5秒后，才同时开始路径跟踪。
3. 圆形每车独立跟踪自己的同圆路径；三角形每条边末端和转向末端都有明确的全体到齐条件。
4. 默认一圈后共同停车并保持 completed。没有手动启用 service；第二轮重启 launch。

启动后可能立即开始到起点的运动，RViz 是运行时显示，不是启动前的确认门。启动前退出同组车辆的旧实验，准备完整车体活动空间和清楚的起点通道；本实验不提供墙、桌或人的障碍感知。

复用2026-10-25三台 Mini 的现场标定及摆位时，直接使用[现场使用步骤](mocap-field-use.md)。日期标定配置仅显式加载；通用入口默认使用原始刚体和动捕差分速度，不依赖场地补偿。

## 圆形 demo

加载 Noetic 和工作空间 setup，新功能按根 README 编译后使用：

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch
```

默认直径2.0m、速度上限0.15m/s、逆时针、一圈。三车位于同一个圆的三个等分起点，并朝切线方向出发。

直径1.5m：

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch diameter:=1.5
```

顺时针两圈：

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch diameter:=2.0 clockwise:=true laps:=2
```

各车有自己的同圆闭合路径，直接发布各自Pure Pursuit结果。初始位置相隔120°，运行中不按最慢车进度缩放速度；底盘执行差异可能使间隔变化，不提供跟踪段全程避障。接近结束的减速来自Pure Pursuit本身，到各自起点后再做最终位姿校正。

## 正三角形 demo

```bash
roslaunch abc_swarm demo_mocap_three_mini_triangle_tracking.launch
```

默认**正三角形边长2.0m，重心固定在world原点**。01/02/03 各占一个顶点并朝向自己的下一条边；每轮同时向下一顶点移动。所有车到齐并稳定后，原地转约120°，全体朝向对齐后再出发。

边长1.5m：

```bash
roslaunch abc_swarm demo_mocap_three_mini_triangle_tracking.launch side_length:=1.5
```

三轮直边运动构成每车一整圈，默认 laps=1。顶点不会被圆角化；转向阶段线速度指令为零。实际位置、航向与直线跟踪存在容差，不能把期望直线理解成实际误差绝对为零。

这个正三角形路径与原“头车领航两台后车”的三角编队是两个实验，原跟随入口继续保留。

## 图形放在动捕场地哪里

圆形默认reference=initial_leader，仍按01当时位姿冻结第一起点和切线；逆时针圆心在其车头左侧一个半径处。圆形的origin是第一起点，不是圆心。

三角形默认reference=world，origin_x/origin_y是三角形重心，默认(0,0)；origin_yaw旋转整个图形，默认0。默认边长2m、逆时针时：

| 车 | 顶点 [x,y] m | 出发朝向 |
| --- | --- | --- |
| 01 | [-1.000,-0.577] | 0°，向02 |
| 02 | [1.000,-0.577] | 120°，向03 |
| 03 | [0.000,1.155] | -120°，向01 |

顺时针时图形沿X轴镜像，车辆仍按01→02→03的角色顺序走下一顶点。若需平移中心或旋转三角形，修改origin_x/origin_y/origin_yaw。reference=initial_leader是可选方式：把启动01位姿作为重心参考，不再作为第一个顶点。没有改动动捕world坐标系本身。

图形冻结后，反馈恢复或车辆移动不会重新锚定。圆形origin和三角形重心在reference=initial_leader时必须为零；自定义非零位置请用reference=world。尺寸默认2m，支持1.5m及以上；活动空间还需容纳车体和入位路线。

## 后台、机器人映射和 ORCA

两个 demo 默认启动原 Mini socket 驱动、VRPN 与 RViz，不启动键盘。后台已有时追加：

```bash
roslaunch abc_swarm demo_mocap_three_mini_triangle_tracking.launch side_length:=1.5 start_driver:=false start_vrpn:=false
```

VRPN 地址仍由 config/mocap.yaml、site_config 或 vrpn_server_ip/vrpn_port 覆盖。机器人和刚体仍用 robot_name_N、rigid_body_N，默认 tianbot_mini_01/02/03。GUI 可传 use_rviz:=false 并复用已运行后台。

默认 avoidance=none，起点通道由现场准备。可选沿用已有 ORCA **仅用于起点点镇定**：

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch diameter:=1.5 avoidance:=orca
```

默认以动捕差分获取速度；选择velocity_source:=odom时还需三车实际odom，全部模式需要有效TF，包络/速度条件见 [ORCA 说明](orca.md)。从进入路径跟踪开始不执行 ORCA 绕行，保持本次指定几何；不能把本开关当成全程障碍保护。配置包络过大、无法容纳该图形的正常间距时会在启动前拒绝；实际误差/场地仍须验收。

## 状态、路径和停止

```bash
rostopic echo /abc_swarm_demo/status
```

mode=coordinated_tracking，task=circle/triangle。state 的 waiting/running/completed/fault 语义沿用已有接口，stage 表示具体阶段：

| stage | 行为 |
| --- | --- |
| assembling | 三车到起点及出发朝向，等待全体稳定 |
| tracking_circle | 各车独立同圆跟踪，直接发布各自控制结果 |
| tracking_edge | 三角形当前直边，端点全体等待 |
| turning | 三角形顶点原地转向，线速度为零 |
| finishing | 圆形结束后的起点位置和最终朝向校正 |
| completed | 完成并持续输出零 |

common_progress/progress_spread 单位m；三角形的 edge_rounds_completed 表示已完成几轮边，三轮为一圈。robots 记录每车阶段、目标误差、路径误差和本周期命令。avoidance_scope=assembly 与 avoidance_active 说明 ORCA 是否正在作用，避免 GUI 把启用选项当作全程生效。

每车路径为 `/abc_swarm_demo/paths/robot_name_N` 的 nav_msgs/Path，当前阶段目标为 `/abc_swarm_demo/goals` 的 PoseArray。默认 RViz 配置显示三条路径、目标和动捕 TF；自定义车名通过 remap 映射路径话题。同圆路径本身重合，颜色对应三台路径索引；起点看目标 PoseArray，车辆位置看 TF 名称。

- 任一必需 TF 失效，全组停车；所选速度反馈失效也会共同停车。恢复后继续原图形和原阶段，不计入失效期间的进展超时。
- 当前位置离路径过远、转向时偏离顶点、长时间没有协调进展会 fault 停车，检查后重启，不会自行重建任务。
- 圆形/三角形都可使用 laps:=0 连续运行，必须在现场监控并手工退出；AI 交付只接受明确的正整数圈数。
- 停止整套实验在 roslaunch 终端 Ctrl+C，确认三车实际静止。不要另开控制这些 cmd_vel 的节点。

## 参数归属

| 位置 | 参数 |
| --- | --- |
| launch | 任务形状、diameter/side_length、speed、laps、clockwise、reference/origin、设备和后台 |
| config/coordinated_tracking.yaml | lookahead、settle_time、max_error、progress_timeout、vertex_tolerance |
| control_config（默认 point_control.yaml） | 点镇定/转向增益、位置/朝向容差、角速度限制 |
| config/orca.yaml | 可选起点 ORCA 的车体包络及求解参数 |

feedback_config（默认feedback.yaml）提供TF/odom有效期及动捕速度窗口。launch的speed仅设置路径跟踪速度，不覆盖点镇定的max_linear。点镇定已有增益动态调参仍可用；几何和协调参数在启动时加载，更换任务须退出旧 launch。

实现是 `src/abc_swarm/coordinated_loop.py`，复用 point_control.py 的点镇定及 pure_pursuit.py 的圆路径跟踪，增加开直线段跟踪与全体屏障。ROS IO 仍用 nodes/point_stabilization.py；每车最终 cmd_vel 始终只有一个发布者，没有新增可执行节点。

## AI 客户任务

两个示例为 ai/coordinated-circle.example.json 与 ai/coordinated-triangle.example.json。experiment 分别是 coordinated_circle/coordinated_triangle；trajectory 明确尺寸、速度、圈数、方向和参考位姿，orca=null 表示无起点避障，或填写完整 ORCA 参数。初始位姿记录不当作起点控制目标。

```bash
python3 ai/generate_launch.py ai/coordinated-circle.example.json --output /tmp/customer-circle-demo
```

```bash
python3 ai/generate_launch.py /tmp/customer-circle-demo/experiment.json --output /tmp/customer-circle-demo --check
```

triangle 示例使用同一生成流程。生成工具只组合固定入口，不运行车辆。没有对障碍感知、任意拥挤起点安全入队或精确相位作未验证承诺。

## 实车验收记录

原三角形/蛇形跟随测试不替代这两个新任务的验收：

| 项目 | 结果 |
| --- | --- |
| 1.5m/2m 图形参考、实际活动范围及路径显示 | 未测 |
| 自动到起点、出发朝向与全体等待 | 未测 |
| 初始同圆间隔、独立跟踪、结束停车 | 未测 |
| 三条直边、顶点到齐、原地转向后同时出发 | 未测 |
| 必需反馈中断/恢复、故障与退出实际停车 | 未测 |

记录 launch 命令、提交、场地、各车起始位置、实际轨迹/相位误差及停车表现。合成模型只用于自动测试，不是新增用户仿真实验，也不能代替 mocap 真车验收。
