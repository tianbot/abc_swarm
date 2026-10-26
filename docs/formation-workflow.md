# 编队实验的完整启动流程与 GUI 接口

仅用于当前 mocap Mini 实验；不增加Gazebo课程或飞行器适配。两个leader–follower入口默认startup=true，manual/auto决定入队后由谁控制头车。

## 阶段与输出归属

| stage | 行为 | ORCA | 遥控请求 |
| --- | --- | --- | --- |
| assembling_orca | 冻结启动目标，规划途经点与通行顺序，ORCA护航入队 | 启用 | 丢弃 |
| fine_positioning | 到各自目标，调整最终航向 | 关闭，保留几何监控 | 丢弃 |
| ready | 全体位置/航向/实测速度稳定后的零速交接 | 关闭 | 丢弃 |
| following | manual转发新遥控；auto头车走路径；后车按原跟随边工作 | 不提供全程ORCA | 仅manual接收 |
| settling | auto路径结束后冻结最终队形，再收敛与稳定停车 | 按入队条件选择 | 丢弃 |
| completed | 全组保持零速 | 关闭 | 丢弃 |

状态state另外区分running/waiting/fault/completed。反馈缺失时保留冻结任务，输出零速；恢复后重新积累速度/稳定窗口。虚拟前置包络重叠时计算受限分离动作（recovery_active=true、参与车辆phase=recovering），恢复后继续原任务；短暂无可行ORCA速度时整组停车等待重新测量，不放松半平面约束；实际车体包络过近/目标冲突等几何错误直接fault，长期无进展也fault。退出或故障后重启任务，不增加人工启用service。

默认完整流程由现有 `nodes/point_stabilization.py` 管理定位、标定、速度反馈和每车唯一cmd_vel。算法在 `src/abc_swarm/admission.py` 和 `formation_control.py`，复用原ORCA、点镇定、leader–follower控制律与Pure Pursuit。不再同时启动旧跟随节点、头车路径节点或demo_runtime发布最终速度。startup=false保留旧式入口，便于对照；原视频48个资源不变。

## 入位路线与通行顺序

开启ORCA的入位默认启用规划：把其他登记车辆当作膨胀圆，构造可见性图并用Dijkstra寻找分段路线；一个时刻只给一台车非零期望运动，其余车辆让行。最终目标被别的车占用时，尝试先去临时停车点腾出位置。ORCA仍校正所有车辆速度，必要时让行车辆也可能移动；实际车体保护与输出限幅保留。目标及角色不变，规划不自动交换02/03分工。

实现为平层 `src/abc_swarm/admission_planner.py`，不增加ROS节点或依赖。参考[PythonRobotics可见性图规划说明](https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/visibility_road_map_planner/visibility_road_map_planner.html)，采用独立的圆障碍几何实现，没有复制其绘图/依赖代码。几何来自orca_config，规划余量参考admission.clearance及最终位置容差；节点参数planning默认true，也可在control_config（独立点镇定为goal_config）中显式设false作无规划对照。

无进展依据是实测位姿沿途经点的推进及精定位误差的改善，不能因正在绕行、暂时远离终点就误报失败；原地等待或反复重规划不算进展。反馈中断后保留最终目标、根据最新位姿重新规划。路线走完仍须检查最终位置、朝向和实测停车；fault保留最后一次误差快照，不能把路线完成当成到位。

这是空场内登记车辆之间的入位规划，未建模墙壁、场地边界、人或未知障碍。临时停车点可能离当前车位约三倍规划包络半径，路线可能超出起终点包围区域，启动前须人工确认周围空场。它不是完整多机器人路径规划或任意目标必达保证，不修复刚体朝向/参考点、底盘跟踪、网络延迟或PID问题；跟随/协调运动阶段也没有增加全程避障。

## 何时退出 ORCA

接近目标不等于可以直接关闭避让。本实现采取工程保守条件：所有位置误差低于handoff_distance；以各目标为中心、覆盖当前误差/最终容差/小量余量的捕获圆，与车体半径及clearance一起计算后两两分离；当前间距也合格；这些条件持续handoff_time后才切换纯点镇定。

纯点镇定期间继续检查实际位姿是否仍在捕获区域、车辆间距是否合格，越界则重新进入避让。最终完成仍要求位置和航向在control_config原定容差内，且实测速度稳定。它不是忽略邻车，也不是对通信延迟、轮胎滑移或未知障碍的安全证明。[ORCA原论文](https://gamma-web.iacs.umd.edu/ORCA/)给出的避碰条件适用于其模型；本仓库的切换监控属于额外工程策略，不宣称继承未验证的全程保证。

## 参数归属

`config/admission.yaml`在config保持一层，包含handoff_distance、clearance、position_margin、handoff_time、settle_time和progress_timeout。handoff_distance只决定何时评估切换，最终精度仍由control_config中的position_tolerance、yaw_tolerance决定。默认最终容差4cm/0.08rad；当日现场配置显式使用8cm，不应把切换到点镇定等同于毫米级精度。

车体包络与速度障碍参数在orca_config；标记原点/航向和点镇定增益在control_config。三角形/蛇形的角色、跟随边和偏移由launch生成；算法遍历车辆列表和跟随边，没有为型号复制实现。内部计算覆盖更长链条，公开launch、AI白名单及硬件验收当前仍限定三台Mini，不能据此宣称任意数量已交付。

独立点镇定的target_poses不用于跟随任务；跟随启动目标由01初始实测位姿和跟随边计算。admission.progress_timeout负责完整跟随任务的入队/自动行进无进展检查；独立点镇定保留orca.stall_timeout，协调路径保留coordinated.progress_timeout。协调路径的admission.settle_time负责入位稳定，coordinated.settle_time负责顶点/路径阶段屏障，两者可以分别配置。

following阶段使用原跟随控制律和实测前车速度。当外侧后车需要更高速度时，头车按最大前置控制点误差缩放v/omega，以保留转弯曲率；tracking_window控制减速窗口，误差达到窗口时头车暂停等待。manual也遵守这个队形协调上限。它仍是编队跟随，不是三车120°同圆。

## GUI 复用接口

| 接口 | 含义 |
| --- | --- |
| launch mode=manual/auto | 启动时选择；切换先退出旧任务再启动 |
| start_driver/start_vrpn/use_rviz/start_teleop | 后台与UI归属，可全部复用已有进程 |
| status.admission_planning | state、active_robot和剩余world平面waypoints；未启用ORCA时为null |
| /abc_swarm_demo/status | schema_version=2，阶段变化立即发，数值更新约5Hz |
| status.control_mode / stage / admission_stage | 任务模式、整体阶段、入队子阶段 |
| status.ready / admission_complete | 是否已经到齐、是否完成过启动入队 |
| status.avoidance_active / recovery_active / formation_error / leader_scale | 是否执行ORCA、是否分离恢复、跟随误差、头车减速比例 |
| status.robots | 每车名称、原始刚体、当前目标、位置/航向误差、命令和阶段 |
| /abc_swarm_demo/startup_goals | 一次冻结的启动PoseArray，latched，顺序对应robots |
| /abc_swarm_demo/goals / control_poses | 当前阶段目标与校准后的水平控制位姿 |
| /abc_swarm_demo/paths/头车名称 | auto路径预览，latched |
| /abc_swarm_demo/teleop_cmd | Twist，仅following且manual时接收新请求；超时0.3秒停止头车请求 |

GUI以status的stage/ready展示进度，不写enabled参数控制启停。enabled是运行状态指示，不是人工arming接口。全组停止退出任务进程；只发零遥控会停头车，后车仍可能继续收敛。未实现GUI页面、静态障碍感知、全程移动ORCA或自动目标分配。

四项命令和当前标定见[现场使用](mocap-field-use.md)。
