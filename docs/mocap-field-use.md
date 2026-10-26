# 三台 Mini 动捕实验

先按本页运行四项实验。所有新任务在反馈有效后自动执行；启动前留出入位和路径活动空间。同一组车辆一次只运行一个实验。

通用Mini教学配置不要求加载场地补偿文件：位置完成容差10cm，3cm内开始最终朝向控制；航向容差0.08rad（约4.6°），仍检查实测停车。三角路径顶点偏离保护为15cm。日期命名的标定配置仅作可选现场实验，加载它会使用该文件自己的容差。

## 一次准备后台

每个终端加载工作区。本机IP按实际网卡设置；下面的`.106`是本次现场地址。

```bash
source /opt/ros/noetic/setup.bash
source ~/tianbot_mini_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.2.106
export ROS_MASTER_URI=http://localhost:11311
```

车辆进入集群模式：按住配网/WIFI键，点击Reset，松开配网键，灯变蓝。rmip指向驱动电脑的11411端口。蓝灯只表示模式，仍需能收到反馈。

后台没开时运行：

```bash
roslaunch abc_swarm mocap_backend.launch
```

动捕地址和端口在`config/mocap.yaml`。已有后台时不再启动一份；下面的实验命令均复用后台。

## 标记原点与现场校准

默认车名和刚体为`tianbot_mini_01/02/03`，参考系world。机器人名决定速度话题，刚体名决定TF。控制点应在轮轴中心，x向前、y向左。没有额外校准表时直接使用刚体原点；`mocap_calibrated:false`本身不是错误。

当前02有标记点偏离轮轴的测量记录。使用本次现场拟合时，在实验命令末尾加：

```bash
control_config:="$(rospack find abc_swarm)/config/mocap_field_control_20261026.yaml"
```

这是命令参数，不是单独执行的命令。该文件只对02应用前向约3.35cm、左向约0.21cm的位置修正，yaw_offset为0；01/03为恒等转换。它是特定标记架的测量，不是客户通用参数。重新定义刚体原点后要重新核对。原始TF不变，实际控制位姿看`/abc_swarm_demo/control_poses`；加载情况看`effective_config`中的完整校准表。

## 1. 三角形 leader-follower

02的目标在01左后方各0.5m，03在右后方各0.5m；两后车都跟01。这里不是等边三角形。

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch start_driver:=false start_vrpn:=false
```

先自动入位，状态进入following后在键盘窗口遥控。键盘显示的请求就是teleop_cmd；反馈有效、请求未超时，头车逐值转发，不二次截断或按后车误差减速。默认初始请求为0.15m/s、0.6rad/s；e/c改变转速10%。实测速度仍由底盘或动捕反馈决定。

## 2. 蛇形 leader-follower

跟随边为01→02→03，相邻目标纵向偏移0.5m；03的前车速度来自02实测反馈。

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_snake.launch start_driver:=false start_vrpn:=false
```

前车停下后，后车继续使用同一前置点跟随公式，不自动切换点镇定。该控制律可能切弯，不保证复走前车历史轨迹或单独收敛车体朝向。键盘k只停止头车请求；退出整套实验用launch终端Ctrl+C。

这两个入口可加`mode:=auto`让头车按圆形路径领航。auto头车固定路径参数speed默认0.08m/s，跟随算法的线速度上限默认0.15m/s；外侧后车需要更高速度，参数应由人配置，程序不暗中修改头车速度。`startup:=false`只跳过启动入位，仍用同一节点、标定和速度来源。

## 3. 三车同圆

默认直径2m、初始120°间隔，各车独立跟踪同圆路径，一圈结束。起点/切线默认由01启动位姿确定；origin表示第一台的起点，不是圆心。

```bash
roslaunch abc_swarm demo_mocap_three_mini_circle_tracking.launch start_driver:=false start_vrpn:=false
```

运行中不按最慢车进度缩放速度，物理执行差异可能使间隔变化。跟踪段没有全程避障。若要入位时车车互避，加`avoidance:=orca`；改尺寸只需加`diameter:=1.5`。

## 4. 三车正三角形逐边运动

默认边长2m，三角形重心为world原点：01(-1,-0.577)、02(1,-0.577)、03(0,1.155)，单位m。每台先到各自顶点，沿边到下一顶点；全体到齐、停车、原地转向，再走下一边。三条边为一圈。

```bash
roslaunch abc_swarm demo_mocap_three_mini_triangle_tracking.launch start_driver:=false start_vrpn:=false
```

改边长用`side_length:=1.5`。三车路径速度speed默认0.15m/s；转弯/终点减速来自路径算法本身，没有进度缩放。ORCA同样仅在显式选择时用于启动入位。

## 参数去哪儿改

| 内容 | 位置 |
| --- | --- |
| VRPN地址、端口 | config/mocap.yaml |
| TF/odom有效期、动捕速度窗口 | config/feedback.yaml |
| 点镇定P、速度上限、位置/航向容差 | config/point_control.yaml；或显式control_config |
| 独立点镇定目标 | config/point_stabilization.yaml |
| 跟随P、前置距离、跟随速度上限 | config/leader_follower.yaml |
| 入位交接、停车门槛、途经点容差、无进展超时 | config/admission.yaml |
| ORCA车体包络和时间范围 | config/orca.yaml |
| 三车路径算法参数、顶点位置门槛 | config/coordinated_tracking.yaml |
| 键盘初始请求 | launch的teleop_linear/teleop_angular |

可选CLI覆盖默认留空，未填写就用YAML。`max_linear/max_angular`只覆盖跟随算法；`point_max_angular`只覆盖点镇定；`speed`是auto头车或三车路径速度；`teleop_linear/teleop_angular`只设置键盘请求。不存在把一项速度静默传给所有控制器的默认行为。

## 直接观察

```bash
rostopic echo /abc_swarm_demo/status
```

```bash
rostopic echo /abc_swarm_demo/effective_config
```

`effective_config`列实际生效值和文件来源，调P后同步更新。status的stage说明任务阶段，command_state说明遥控转发/超时，processing说明计算步骤。raw_cmd/车名、point_cmd/车名、preferred_cmd/车名用于观察中间结果，最终指令是各车`/cmd_vel`。未执行的中间步骤为null，不把上一次值当作当前值。

所有新任务用位姿和选定实测速率判断稳定到位。默认速度来源是动捕差分，可明确改为odom。反馈中断立即零输出；恢复后重新积累测量窗口，人工旧请求不回放。fault锁定后重启实验；没有手动启用service，也没有通用于所有实验的reset。

GUI可用`start_teleop:=false use_rviz:=false`复用上述接口。完整阶段说明见[流程与接口](formation-workflow.md)。
