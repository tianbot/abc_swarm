# 动捕点镇定中加入 ORCA 车车避让

对应 [issue #7](https://github.com/tianbot/abc_swarm/issues/7)。本轮仅面向 **三台 Tianbot Mini + VRPN 动捕**，在已有点镇定过程中增加 ORCA；不增加其他实验环境。

## 做什么、在哪一步工作

每车仍去自己的目标位置，最后对齐目标朝向。点镇定给出期望运动，ORCA 根据三台车当前状态调整途中运动，控制器再输出差速底盘 v/omega。接近目标且捕获区域与车体包络可分离后，切换纯点镇定精确入位；越界或间距不足则恢复ORCA。完成仍检查原位置/航向容差及实测停车，不扩大最终容差。独立点镇定到位后整组停车，不自动切入跟随。

两个leader–follower入口现在默认复用这套入队，再自动交给manual/auto跟随；完整阶段和GUI接口见[编队启动流程](formation-workflow.md)。

普通点镇定默认avoidance=none，不加入ORCA。ORCA 复用同一个 nodes/point_stabilization.py；新增入口只是明确选择避障模式，没有复制整套节点。

**启动后反馈齐全即运动，ORCA 模式可能短距离倒车。** 没有人工启用 service。它只避让本实验列出的三台车：墙、桌子、人、未登记车辆都不在这个模型里。仅有动捕不等于自动获得静态障碍几何。

## 启动

先编译 abc_swarm 并 source `/opt/ros/noetic/setup.bash` 与 `~/tianbot_mini_ws/devel/setup.bash`。退出控制同组车辆的旧跟随/点镇定实验，确认车体半径、参考点和场地后再启动。

在原点镇定命令上明确加 avoidance：

```bash
roslaunch abc_swarm demo_mocap_three_mini_point_stabilization.launch avoidance:=orca formation:=triangle
```

也提供名字直接可辨认的等价入口：

```bash
roslaunch abc_swarm demo_mocap_three_mini_orca_point_stabilization.launch formation:=triangle
```

已有底盘与 VRPN 后台时，用下列命令替代，避免重复启动：

```bash
roslaunch abc_swarm demo_mocap_three_mini_orca_point_stabilization.launch formation:=triangle start_driver:=false start_vrpn:=false
```

formation=triangle/snake 时按启动头车实际位姿冻结队形目标；formation=config（默认）读取目标文件。预设三角形中的 01 保持原位置，另外两台自动到位，但 ORCA 可能为了避让让某台车暂时偏离其目标。自定义 world 目标可让三台都移动：

```bash
roslaunch abc_swarm demo_mocap_three_mini_orca_point_stabilization.launch formation:=config goal_config:=/absolute/path/targets.yaml orca_config:=/absolute/path/orca.yaml
```

目标配置格式沿用[点镇定说明](point-stabilization.md)，不是持续蛇形跟随。第一轮先做三角形摆位附近的小幅调整，避免包络重叠；确认基本定位/速度反馈与实际停车后，再由现场操作人员安排宽敞场地中的对向、交叉或换位实验。不要把文档中的坐标示例当成当前场地目标直接执行。

## 参数和反馈

默认velocity_source=mocap，由动捕控制位姿差分得到实测速度，不依赖底盘odom。窗口在feedback.yaml的mocap_velocity配置。显式选择velocity_source:=odom时，odom_timeout/odom_future_tolerance也从feedback.yaml加载；需要延长容忍期请编辑自己的feedback_config，而不是ORCA几何文件。较大窗口会使用更旧的反馈，不能据此保证避碰。AI对应字段是mocap_control.feedback。

`config/orca.yaml` 是一层配置文件，启动时加载：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| robot_radius | 0.075 m | 以动捕控制参考点为中心、覆盖实际车体的半径候选 |
| safety_margin | 0.01 m | 每车额外包络余量 |
| front_distance | 0.04 m | 差速前置控制点距离 |
| time_horizon | 3.0 s | 两两速度障碍预测时域 |
| right_bias | 0.01 m/s | 小幅靠右的期望速度偏置，用于打破对向对称；不放松约束 |
| velocity_buffer | 0.002 m/s | 将避碰半平面稍向内收紧，给离散更新留数值余量；不是实测误差保证 |

2026-10-26 操作者确认Mini车体可用直径15cm圆包络，默认车体半径改为7.5cm。旧值13cm来自导航footprint，明显大于本次设备实物；不再用它代表本实验Mini。半径以差速轴心/控制中心为参考，仍须覆盖标记架、载荷和参考点偏移，不能将车体半径调到实际尺寸以下来绕过检查。

前置点有效半径 R=robot_radius+front_distance+safety_margin，当前为0.125m；有效圆心在每车前方4cm，两个有效圆心距离需超过25cm。这个门槛来自差速外包圆，不表示车体直径25cm；不同车头朝向下也不等于实际轴心间距门槛。目标检查额外保留2×velocity_buffer×time_horizon，默认1.2cm。精定位捕获区和入队交接仍检查自身条件，不因包络缩小而跳过。

前置距离从8cm减为4cm，降低虚拟外包圆的膨胀；它也决定可实现的横向前置点速度，不能只为压缩空间而无限减小。3cm在当前交叉入队模型出现停滞，4cm通过该模型；这不是实车调参验收。当前几何配置与AI示例同步，运行中的节点不会自动热加载这些更改。

所有三台需要 world→rigid_body_N 的有效 TF。显式选择`velocity_source:=odom`时，使用对应 `/robot_name_N/odom` 的实际 linear.x/angular.z；`velocity_source:=mocap` 使用控制中心位姿约0.2秒窗口估计实测速度，约0.08秒样本积累后开始，不依赖底盘 odom。反馈缺失或时钟/时间窗失效时全组停车，恢复后继续。ORCA 使用实测速度，不把 cmd_vel 当作反馈；车辆仍需连接底盘驱动才能接收命令。无避障点镇定同样检查所选速度反馈，以确认实际停车。最大车体直线速度指令为 ±0.15 m/s，角速度默认 ±0.5 rad/s，接近目标、转弯和避让时会更低。

动捕原点不是差速轴心，或刚体朝向不对应车头时，通过控制/目标 YAML 的 `mocap_calibration` 显式配置 yaw_offset 与 marker_offset。节点只转换内部控制位姿，不覆盖原始 TF。当前现场专用配置和命令见[现场用法](mocap-field-use.md)，不要把其中02的补偿复制给新设备。所有新入口均通过control_config加载标定和点镇定参数；goal_config只加载目标。

精定位按显式position_hysteresis切换：默认3cm内进入最终朝向控制，超过10cm才返回位置调整，最终位置验收是10cm；详见点镇定说明。没有额外速度渐变。入位稳定时长、实测停车门槛及无进展超时统一在admission.yaml。raw_cmd、point_cmd、preferred_cmd和各车cmd_vel可对照各处理步骤。

切换与稳定窗口由 `config/admission.yaml` 配置，可通过 `admission_config` 替换；各参数归属见[完整流程](formation-workflow.md#参数归属)。

几何/时域参数重启后生效；已有点镇定的 k_distance/k_heading 动态调参继续可用，修改增益不会改变目标或重新启动 completed 的任务。

## 观察和停止

```bash
rostopic echo /abc_swarm_demo/status
```

模式仍为 point_stabilization，新增 avoidance=orca。robots 保留每车目标、误差及阶段；admission_stage区分assembling_orca、fine_positioning、ready。avoidance_active只在实际ORCA阶段为true。

| state | 行为 |
| --- | --- |
| waiting | 任一反馈缺失/过期，或ORCA速度交集暂时为空，全组输出零；条件恢复后继续原冻结目标 |
| running | 正在ORCA入位，或受几何监控的纯点镇定 |
| completed | 三台位置和最终航向均到达，持续停车，不自动开始第二轮 |
| fault | 实际车体包络过近、目标冲突或长期无进展；整组停车并锁定，检查后重启 launch |

虚拟前置圆包络重叠时不再直接锁定fault：参与车辆通过径向分离约束，按现有time_horizon逐步拉开，同时继续满足差速v/omega上限。分离速度不可行时全组零输出等待重算，不放松约束；长期无进展仍fault。实际控制中心距离不大于2×(robot_radius+safety_margin)时仍锁定停车（当前默认17cm）。status新增recovery_active，各参与车辆phase为recovering；恢复后继续原冻结目标，不重新分配目标，也不增加service。

反馈中断时间不计入进展超时。ORCA 是局部避让，不能保证所有拥挤/对称配置都能到达目标；fault 是未完成，不能当成避障成功。不要用重启反复强推几何不可行的任务。

```bash
rostopic echo -n 1 /abc_swarm_demo/goals
```

目标仍是 world 中的 PoseArray，按设备数组顺序映射。退出实验用 roslaunch 终端 Ctrl+C 并确认实际静止。同一车辆只允许一套算法发布最终 cmd_vel。

## AI 客户配置

使用 `ai/orca.example.json`，experiment=point_stabilization_orca。targets 与初始摆位记录仍分开；额外的 orca 对象明确填写上表全部参数。示例半径和位姿只是候选，AI 必须让客户核对车体/刚体映射及物理包络，不能推断不存在的障碍信息。

```bash
python3 ai/generate_launch.py ai/orca.example.json --output /tmp/customer-orca-demo
```

```bash
python3 ai/generate_launch.py /tmp/customer-orca-demo/experiment.json --output /tmp/customer-orca-demo --check
```

```bash
roslaunch --nodes /tmp/customer-orca-demo/experiment.launch start_driver:=false start_vrpn:=false use_rviz:=false
```

生成器会拒绝未知/缺少参数、非法数值和终点有效包络重叠。它不能确认实际起点或真实车体尺寸；节点还会根据实测状态检查。生成 launch 在反馈齐全后自动运行，不由 AI 代启动。

## 实现依据和局限

两两 ORCA 半平面参考 [RVO2 Agent.cc 固定提交](https://github.com/snape/RVO2/blob/75822bba14462c2e872ca96423825017ecd24764/src/Agent.cc)，Python 实现在 `src/abc_swarm/orca.py`，保留 Apache-2.0 归属，许可证和参考文件哈希在 `licenses/orca/`。没有引入 C++ 编译依赖；仅实现本轮需要的参与车辆两两约束，不包含 RVO2 的静态障碍部分。

差速适配参考 [ORCA-DD](https://gamma-web.iacs.umd.edu/ORCA-DD/) 的有效圆心和半径思想。前置点速度满足 u=v·heading+d·omega·left；本实现把车体 v/omega 上限变成半平面，直接求与 ORCA 半平面的交集，再还原车体命令。不会在求解后随意裁剪速度并继续假设约束成立；交集为空则整组零输出并等待下一次测量重试；不放松约束，长期无进展仍锁定故障。

重叠恢复属于本仓库的差速工程适配：标准RVO2对重叠圆使用单步时间尺度的分离分支；此处针对仍未侵入实际车体包络的虚拟圆重叠，采用径向分离半平面和现有预测时域，避免要求一周期完成分离。目标包络冲突仍是配置错误，不进入恢复。这一分离分支本身不负责路线规划；上层入位规划另行提供途经点与通行顺序。

这是工程首版，不声称逐项复现论文全部假设或证明。尚未辨识执行延迟、打滑、刹停距离、轮速组合限幅和动捕误差；离散周期、执行器对命令的裁剪也影响真实效果。计算模型中的几何余量不替代实车验收。当前ORCA入位已增加登记车辆间的可见性图绕行、逐车通行和临时停车点，详见[入位规划](formation-workflow.md#入位路线与通行顺序)。没有静态障碍感知、场地边界规划、自动目标分配或所有配置的可达性保证。

## 测试与现场验收

本仓库的数值模型和私有 ROS 反馈只用于测试算法及接口，不作为新增的纯 Python 教学实验。用户操作仍是 mocap launch。新 ORCA 实车验收由用户完成：

| 项目 | 实车结果 |
| --- | --- |
| 实际车体/标记架半径与刚体参考点核对 | 未测 |
| 空场小范围三角形到位与停车 | 未测 |
| 宽敞通道中的对向避让 | 未测 |
| 交叉、三车换位 | 未测 |
| TF/odom 中断共同停车及恢复 | 未测 |
| 故障锁定、退出实际停车 | 未测 |

已有三角形、蛇形跟随实测结果不代表 ORCA 通过。记录场地、提交、配置、初始/目标位姿、最小实际间距及状态原因。
