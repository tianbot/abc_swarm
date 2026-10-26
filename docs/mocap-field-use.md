# 三台 Mini 动捕实验：现场使用

ROS1 Noetic；本页整理2026-10-25现场使用的两个独立实验。现在直接使用仓库 launch，不再依赖 `/tmp` 测试脚本或额外的临时 TF 进程。

## 准备一次后台

三台 Mini 进入集群模式：按住配网/WIFI键，点击 Reset，随后松开配网键，状态灯变蓝。车辆与电脑连接同一局域网，rmip 指向运行底盘驱动的电脑，端口11411。蓝灯后仍需核对实际连接。

每个终端加载环境。下面 `.106` 是当日控制电脑地址，换电脑/网络后改成实际地址：

```bash
source /opt/ros/noetic/setup.bash
source ~/tianbot_mini_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.2.106
```

后台没有启动时，在一个终端运行：

```bash
roslaunch "$(rospack find abc_swarm)/launch/common/mocap_backend.launch.xml" vrpn_server_ip:=192.168.2.101
```

默认机器人与刚体对应 `tianbot_mini_01/02/03`，动捕 server 为 `.101:3883`。底盘 rmip `.106` 与动捕 server `.101` 用途不同。退出旧运动算法，每次只运行一个实验。

## 本次现场配置

另一个终端加载相同环境后，选择配置：

```bash
export ABC_SWARM_FIELD_CONTROL="$(rospack find abc_swarm)/config/mocap_field_control.yaml"
```

这个配置是**当日三台车的专用配置**，显式保存8cm最终位置容差、02平面航向补偿以及三车标记原点到差速控制中心的偏移。它不是新客户的通用默认值，未默认加载，也不会覆盖原始动捕 TF。更换标记架、重新建立刚体、修改刚体坐标或更换车辆后需重新核对；已在动捕软件修正坐标时，去掉相应补偿，避免重复校正。

节点内部使用校正后的水平控制位姿进行目标定位、速度估计和控制；`/abc_swarm_demo/control_poses` 按机器人顺序发布这些位姿，RViz可添加 PoseArray 查看。原 `rigid_body_N` 仍指向原始动捕刚体，不要再启动昨天的临时 `_field_control` TF 适配进程。

## 三车同圆，一圈

直径1.5m，圆心固定在 world `(0.5,0.5)`，第一起点 `(0.5,-0.25)`，逆时针：

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch diameter:=1.5 speed:=0.15 laps:=1 reference:=world origin_x:=0.5 origin_y:=-0.25 origin_yaw:=0.0 control_config:="$ABC_SWARM_FIELD_CONTROL" velocity_source:=mocap avoidance:=orca start_driver:=false start_vrpn:=false
```

## 三车正三角形，一圈

边长1.5m，三个顶点约为 `(-0.25,-0.25)`、`(1.25,-0.25)`、`(0.5,1.049)`：

```bash
roslaunch abc_swarm demo_mocap_three_mini_triangle_tracking.launch side_length:=1.5 speed:=0.15 laps:=1 reference:=world origin_x:=-0.25 origin_y:=-0.25 origin_yaw:=0.0 control_config:="$ABC_SWARM_FIELD_CONTROL" velocity_source:=mocap avoidance:=orca start_driver:=false start_vrpn:=false
```

两条命令都在定位有效后自动入位、调整出发朝向，然后执行任务，不需要启用 service。三角形每轮各走一条直边，到齐停车、原地转向，再一起走下一边；三条边为一圈。`speed=0.15` 是指令上限，转弯、靠近目标和等慢车时会降速。ORCA只用于入位，路径段没有全程避障。

以上坐标描述控制中心参考线，车辆外形会超出图形边界。换场地先调整 world 原点与尺寸；需要按01启动位姿放置图形时，使用 `reference:=initial_leader origin_x:=0 origin_y:=0 origin_yaw:=0`。圆直径用 diameter，三角边长用 side_length；两者支持1.5m及以上，原入口默认仍为2m。

## 观察、停止和重来

```bash
rostopic echo /abc_swarm_demo/status
```

`stage` 顺序为 assembling → tracking_circle/ tracking_edge → turning/finishing → completed。状态中应看到 `velocity_source: mocap`、`mocap_calibrated: true`。mocap 速度模式使用约0.2秒控制中心位姿窗口估计实测速度，不等待底盘 odom；odom 仍可单独用于驱动诊断。动捕过期仍全组输出零，恢复后继续冻结的任务。接收不到速度指令的车辆本身仍需要有效的驱动连接，动捕速度模式不会替代这一连接。

完成后持续停车。在实验终端 Ctrl+C 退出，确认三车静止；后台保留。下一轮重新启动同一命令。出现 fault 先检查 reason，再退出/摆位/重启。不要同时运行旧测试脚本、另一套轨迹或直接发布 cmd_vel 的遥控。

## 本轮收口的验证范围

- 1.5m圆：现场录包出现 completed；最大路径误差约6.7cm，RMS约1.3～1.5cm，最终位置误差约6.9～7.5cm，符合当轮8cm容差。该轮使用临时02航向补偿，尚未包含后来整理的完整控制中心校准和动捕速度模式。
- 1.5m三角：用户反馈已运行；自动录包中进入过 tracking_edge，最后一份自动记录因无进展退出，未记录三边完整 completed，不记为完整一圈验收通过。
- 2m圆/三角、顺时针实车、收口后正式配置：本轮不追加实车测试，待下次现场验收。
- 临时补偿和速度模式已收进共享节点及显式配置，离线/私有ROS验证另记录在开发说明，不把它当作新增实车结果。

当日圆形录包的轨迹图（浅色虚线为入位，彩色实线为运行，黑色虚线为参考圆）：

![三台Mini的1.5m圆现场轨迹](mocap-circle-20261025.png)

通用任务参数见[协调轨迹说明](coordinated-tracking.md)，车车避让见[ORCA说明](orca.md)，详细现场记录见[测试清单](field-test.md)。
