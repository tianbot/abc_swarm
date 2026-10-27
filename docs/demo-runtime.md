# 动捕任务的启动与GUI接口

常用四项实验以[现场使用](mocap-field-use.md)为准。ROS1 Noetic专用，旧课程入口保持原用法。

## 输入到输出

三角/蛇形跟随在startup=true/false和manual/auto下始终使用同一个`point_stabilization.py`任务节点，ROS名称`/abc_swarm_demo`。startup只表示是否先入位。

manual的`/abc_swarm_demo/teleop_cmd`含底盘v/omega请求。反馈有效、处于following且未超时，原样转发到头车cmd_vel；不按跟随误差缩放、不再限幅。跟随公式计算后车指令，算法本身的输出界限在leader_follower.yaml；键盘初始值独立。

启动入位是显式流程：规划期望→ORCA→点镇定精定位→全体稳定→跟随。`planning:=false`可直接比较ORCA，不使用分段路线规划。入位期间忽略遥控，不回放旧输入。前车静止不触发后车换算法。

## 生命周期

| 状态/阶段 | 含义 |
| --- | --- |
| waiting | 缺少所需新反馈，或ORCA暂时没有可行解，最终指令为零，reason说明原因 |
| assembling_orca / fine_positioning | 启动入位，当前中间速度可观察 |
| ready / following | 完成入位交接，或显式跳过入位后运行；admission_skipped区分两者 |
| settling / completed | 自动任务结束收敛和稳定停车；不自动开始下一轮 |
| fault | 路径/几何/无进展故障锁定，最终指令为零，保留最后误差快照；重启任务 |

默认反馈有效即自动运动。退出launch终止控制，人工确认车辆实际静止。键盘k或超时只停止头车请求，后车仍使用所选跟随算法。没有手动enable服务。

反馈恢复后继续冻结目标，不重新选择起点；人工输入需要新请求。无进展超时、位姿和停车门槛从入位/路径配置读取；command_state和reason解释被暂停的条件。

## 接口

| 接口 | 内容 |
| --- | --- |
| status | JSON schema_version=3，node_type区分任务/运行管理，task与stage标明算法和阶段 |
| effective_config | latched实际参数与来源，含标定表、速度来源、测量窗口 |
| teleop_cmd | 人工输入Twist，只取平面差速v/omega；有效请求逐值转发 |
| raw_cmd/车名 | 当前控制原始结果，点镇定阶段为未限幅P需求 |
| point_cmd/车名 | 点镇定有界输出 |
| preferred_cmd/车名 | 规划后给ORCA的期望速度 |
| /车名/cmd_vel | 实际发布给驱动的指令；实测速度来自反馈，不把两者混称 |
| control_poses / goals / startup_goals | 控制参考点、当前目标、冻结启动目标，按robots顺序排列 |

中间话题仅在相应计算步骤存在时发布，status对应字段否则为null。processing列出处理步骤；feedback_valid=false时误差为最后快照。新任务的ready_criterion为pose_and_measured_stop。没有任何节点同时争用最终速度话题。

## 后台和参数

start_driver/start_vrpn控制是否由本launch启动后台；复用时设false。site_config提供VRPN设置，显式地址/端口覆盖场地值；环境变量ABC_SWARM_VRPN_SERVER也可提供地址。feedback_config统一提供输入有效期；控制参数分属point_control和leader_follower配置。use_sim_time默认不写全局参数，只有显式设置才覆盖现有master。

## 独立路径与PID对照入口

`demo_mocap_path_tracking.launch`及两个独立路径+跟随组合，保留独立`preset_path_tracker.py`和后车节点，供观察解耦的算法组合。路径状态为`/abc_swarm_path/status`，算法参数为其effective_config；`/abc_swarm_demo`此时是path_runtime，status明确node_type。只有这些路径运行管理链路提供reset服务；新四任务直接重启开始下一轮。

PID对照的运行管理是manual_relay，原样转发头车请求。其键盘初始请求独立为teleop_linear=0.15m/s、teleop_angular=0.15rad/s，不由PID输出上限覆盖。PID结果包含配置的输出界限及抗积分饱和，不再附加yaw替换/死区。`/mini_2_tracker/pid_cmd`与02最终指令一致；动态PID增益和界限看其effective_config。

两个从原课程搬来的`demo_mocap_tf_tt_*`保持原内容，不属于上述运行协议，也不会自动管理飞行器起降。
