# 三车动捕点镇定：到位置，再对齐朝向

对应 [issue #6](https://github.com/tianbot/abc_swarm/issues/6)。这是在三角形/蛇形跟随之后增加的独立实验，目标是看清“当前位置 → 指定位置与最终航向”的控制过程。ROS1 Noetic 专用；原视频命令和已测的跟随入口保持不变。

## 本次做什么

三台 Mini 各有一个目标位姿，依次转向目标、向前接近、最后原地对齐目标朝向。不是头车遥控领航，也不会自动切入跟随实验。节点不需要 odom 速度前馈，只使用有效动捕 TF。

**launch 启动后，全部定位有效即自动运动，不需要调用启用 service。** 本页默认的普通模式没有避障、目标自动分配或无碰撞入队规划；ORCA 模式见 [独立说明](orca.md)。首次只做摆位附近的小幅修正，为每台车准备互不冲突的通道，不要用交叉/密集起点测试“自动集结”。

## 先试默认三角形目标

先退出正在运行的三角形/蛇形或其他控制同组车辆的实验，加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/tianbot_mini_ws/devel/setup.bash
```

新增节点后先按根 README 编译本包并重新 source。VRPN 场地文件仍为 config/mocap.yaml；默认车名及刚体为 tianbot_mini_01、02、03。

先按原三角形摆位，各车同向。让 02 或 03 偏离自己的位置约 0.1–0.2 m、朝向略偏，保持通道空旷。启动：

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch
```

如果驱动与 VRPN 后台已经运行，用下面这一条替代，避免重复后台：

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch start_driver:=false start_vrpn:=false
```

默认 config/point_stabilization.yaml 为 initial_leader：在**三台定位全部有效**时，按 01 的实测位姿一次性冻结目标。01 保持捕获的位置和朝向；02 目标在其后方 0.5 m、左侧 0.5 m；03 在后方 0.5 m、右侧 0.5 m。目标冻结在 world，不随 01 后续移动；如果 01 偏离，也会返回自己的目标。

启动前的客户起点记录不用于改写 TF；默认参考点来自实际动捕。刚体 x 向车头、y 向左，原点对应底盘控制参考点。

## 用参数选择任务

点镇定的“任务”就是每台车的目标位置和最终朝向。可以直接选择预设：

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch formation:=triangle
```

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch formation:=snake
```

两种预设都在全部定位有效时冻结 01 的位姿；01 保持该位置，02/03 到位后停车，各车最终朝向与冻结的 01 朝向相同。

| formation | 01 目标 | 02 相对目标 | 03 相对目标 |
| --- | --- | --- | --- |
| triangle | 原位置/朝向 | 后 spacing、左 spacing | 后 spacing、右 spacing |
| snake | 原位置/朝向 | 后 spacing | 后 2×spacing |
| config（默认） | 由目标文件决定 | 由目标文件决定 | 由目标文件决定 |

`spacing:=0.5` 设置预设间距。预设显式覆盖文件中的 target_reference/target_poses，固定使用 initial_leader；用自定义 world 目标时选择 `formation:=config goal_config:=...`。默认不写 formation 就读取默认目标文件，效果是三角形；自定义文件仍按原来的方式生效，不会被默认预设覆盖。

点镇定蛇形是“排成一列并停住”，不等于 01→02→03 的持续跟随。它完成后不会自动切换到跟随实验。自定义 world 文件可以给 01 也指定新位置，使三台车都移动。

直线速度默认上限为 0.15 m/s，可通过max_linear显式覆盖；不传时使用control_config的max_linear。它不是全程恒速：接近目标会按距离减速，转向/最终对齐航向时直线速度为零。没有改角速度限制。

## 看目标、误差和结束

```bash
rostopic echo /abc_swarm_demo/status
```

状态是 JSON：mode=point_stabilization；waiting 表示某台定位缺失/过期；running 表示自动执行；completed表示所有位置和航向进入容差，且实测速度满足admission配置的停车门槛并保持稳定窗口。robots 字段包含每车 goal、phase、position_error 和 yaw_error。

phase 分为 aligning、approaching、orienting、arrived。默认最大前进速度 0.15 m/s、角速度 0.5 rad/s，位置容差 0.10 m、航向容差 0.08 rad，使用前进与原地转向，不倒车。

精定位使用明确的两个位置切换门槛：最终位置容差`position_tolerance=0.10m`，切换余量`position_hysteresis=0.07m`，均在point_control.yaml。先进入3cm后才开始对齐最终朝向；此后只要位置不超出10cm就保持朝向控制，超出10cm再返回位置调整。最终完成要求位置≤10cm、航向误差≤0.08rad并实测停车稳定；这是面向Mini教学实验明确放宽的位置验收值。设position_hysteresis=0可比较原来的单门槛行为。

`align_angle`默认π/4，朝向目标位置的偏角超过它时先原地转向。线速度是距离比例项（先限制最大值，再乘朝向余弦），角速度是当前阶段角误差的比例项并限制最大值。源码point_velocity接收previous_phase，AdmissionController保存每车阶段；raw_cmd与最终命令使用同一阶段。没有速度渐变。

直接查看计算链路：`/abc_swarm_demo/raw_cmd/机器人名`是未限幅比例需求，`point_cmd/机器人名`是点镇定控制结果，`preferred_cmd/机器人名`是规划器给ORCA的期望；ORCA结果直接发布到`/机器人名/cmd_vel`。阶段不使用的中间话题不发布，status中相应字段为null，避免把旧值当作当前值。

```bash
rostopic echo /abc_swarm_demo/effective_config
```

此话题列出实际生效的控制参数、标定、输入速度来源、动捕速度差分窗口和配置文件来源。动态修改比例增益时会同步更新。status中的`processing`指出当前计算步骤，`feedback_valid=false`表示本条状态没有新的有效反馈，位置误差按最后快照保留。新任务的`ready_criterion=pose_and_measured_stop`，同时判断位姿和实测停车。

```bash
rostopic echo -n 1 /abc_swarm_demo/goals
```

目标用 geometry_msgs/PoseArray 发布，frame_id=world，顺序对应 launch 中 01/02/03。RViz 可添加 PoseArray，topic 选 /abc_swarm_demo/goals；use_rviz:=true 可打开已有显示配置，但需要手工添加这个目标显示。

任意必需 TF 过期超过 0.5 秒，**整组三车输出零速度**；定位恢复后继续原目标，不重新捕获起点。completed 后持续停车，外力移动车辆也不会自动启动第二轮；新一轮请重启 launch。

停止整个实验在 roslaunch 终端按 Ctrl+C，并确认车辆实际静止。不要同时开其他 cmd_vel 控制节点。本轮没有新增启用/重置 service。

## 指定 world 下各自的目标

复制 config/point_stabilization.yaml 到自己的场地文件，修改这两项。例如下列只是文件格式示例，坐标须换成现场确认的空旷目标：

```yaml
target_reference: world
target_poses:
  - [1.0, 1.0, 0.0]
  - [0.5, 1.5, 0.0]
  - [0.5, 0.5, 1.5708]
max_linear: 0.04
```

数组顺序一对一对应 robot_name_1/2/3；位置单位 m，航向 rad。world 模式不会叠加启动头车坐标。目标文件只包含目标；算法参数修改point_control.yaml，并通过control_config加载；反馈参数通过feedback_config加载。

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch goal_config:=/absolute/path/targets.yaml
```

设备名称和刚体仍可通过 robot_name_N、rigid_body_N 分别指定。目标数量必须与车辆数量相等；重复名称/刚体、非有限目标及非法限幅会在节点初始化阶段报错，不开始输出运动。

## AI 客户交付

示例为 ai/point-stabilization.example.json。targets 是**控制目标**，与 robots[].initial_pose 起点记录分开；reference=world 或 initial_leader，poses 数量必须为三。没有头车跟随角色，数组顺序仅用于设备与目标映射。

```bash
python3 ai/generate_launch.py ai/point-stabilization.example.json --output /tmp/customer-point-demo
```

```bash
python3 ai/generate_launch.py /tmp/customer-point-demo/experiment.json --output /tmp/customer-point-demo --check
```

在 Noetic 环境可先做只解析检查：

```bash
roslaunch --nodes /tmp/customer-point-demo/experiment.launch start_driver:=false start_vrpn:=false use_rviz:=false
```

生成目录中的运行单有准确的启动和停止步骤；AI 不会代替客户启动实验。新实验登记在能力表及测试中，当前生成契约为v3，旧配置迁移见ai/README.md。

## 实现与后续

- src/abc_swarm/point_control.py：点镇定计算和坐标转换，不依赖 ROS。
- nodes/point_stabilization.py：整组定位检查、冻结目标、逐车限幅输出及到达停车。
- launch/mocap/demo_mocap_three_mini_point_stabilization.launch：三台 Mini、VRPN、目标配置的组合。
- config/point_stabilization.yaml：目标；config/point_control.yaml：算法；config/feedback.yaml：输入有效期和速度测量窗口。

这里的无 ROS 计算只为共享算法和测试，并不新增纯 Python 教学课程。ORCA 已作为同一节点的可选模式接入，使用车体包络及邻车实测速度；障碍输入、自动目标分配及未知环境的全局规划留待后续；登记车辆之间的入位规划已提供。ORCA 的参考入口是[官方项目与差速机器人论文](https://gamma-web.iacs.umd.edu/ORCA/)。普通到位控制本身没有避障，必须明确选择 avoidance=orca 并满足其反馈及几何条件。

## 现场验收（由用户填写）

2026-10-24 用户报告简单三角形、蛇形跟随已测完；这是之前两个入口的结果，不替代本实验验收。自动路径与本点镇定新功能没有据此记为通过。

| 项目 | 结果 |
| --- | --- |
| 默认三角形目标冻结正确 | 未测 |
| 02/03 小范围位置修正及最终朝向 | 未测 |
| 指定 world 目标，设备与目标一一对应 | 未测 |
| 定位丢失时三车停止，恢复后继续原目标 | 未测 |
| 全部到位持续停车、退出实际停车 | 未测 |

记录启动命令、提交号、场地配置、实际初始位置和状态 reason。ROS1 Noetic 已结束上游常规支持，本包当前沿现有设备/课程环境维护，本轮不迁移 ROS2。
