# 新 demo 的统一启动与 GUI 接口

适用于 `launch/mocap/` 的六个新增 Mini 动捕实验；原有空地参考和原视频入口保持原操作。本轮不增加 Gazebo 入口。

## 一个文件启动完整实验

例如三车三角形手动领航：

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch
```

默认启动 Mini socket 驱动、VRPN 客户端、跟随算法、统一运行节点及键盘遥控终端。自动路径入口默认启动驱动、VRPN、算法、运行节点和 RViz，不启动遥控。所有新实验默认禁用运动，反馈就绪后再开始。

| 参数 | 用途 |
| --- | --- |
| `site_config` | 场地配置文件，默认 `config/mocap.yaml` |
| `vrpn_server_ip`、`vrpn_port` | 覆盖配置文件中的服务器和端口 |
| `start_driver`、`start_vrpn` | 默认 true；已有对应后台时设 false |
| `start_teleop` | 手动领航默认 true，打开键盘终端；自动路径不提供此参数 |
| `teleop_terminal` | 手动领航默认 `xterm -e`，可替换终端启动命令 |
| `use_rviz` | 手动领航默认 false，自动路径默认 true |

只运行一个控制同组车辆的实验。驱动或 VRPN 已运行时：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch start_driver:=false start_vrpn:=false
```

## 地址只配置一次

`config/mocap.yaml` 保存当前场地的 server、port 和 VRPN 客户端设置。需要部署到其他场地时，用独立文件，不必修改每个 demo：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch site_config:=/absolute/path/site.yaml
```

临时覆盖：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch vrpn_server_ip:=192.168.2.101 vrpn_port:=3883
```

服务器优先级：命令行 `vrpn_server_ip` > 环境变量 `ABC_SWARM_VRPN_SERVER` > 场地文件。端口优先级：命令行 `vrpn_port` > 场地文件。数值仅出现在场地配置中，不在每个 launch 中重复。原监控脚本未指定 `ping_target` 时读取实际 `/vrpn_client_node/server`，不再默认 ping 另一个地址；旧频率估算仍未作为正式就绪判断使用。

## 准备、开始、停止

查看统一状态：

```bash
rostopic echo /abc_swarm_demo/status
```

准备好再开始：

```bash
rosservice call /abc_swarm_demo/set_enabled "data: true"
```

停止头车遥控输出和后车算法：

```bash
rosservice call /abc_swarm_demo/set_enabled "data: false"
```

路径实验停止后重置：

```bash
rosservice call /abc_swarm_demo/reset
```

等待路径重新生成及 ready 后再开始。手动领航实验没有路径，reset 返回不支持；重新开始使用 set_enabled。

原 `/abc_swarm_demo/enabled` 参数继续兼容，但直接设 true 也会受反馈检查约束。GUI 应使用服务，以便获得成功/拒绝及原因。路径故障锁定后需通过统一 reset 清除，不能反复设置启用参数强行恢复。

## 键盘和 GUI 遥控共用入口

手动领航时，键盘和 GUI 都向 `/abc_swarm_demo/teleop_cmd` 发布 `geometry_msgs/Twist`；统一运行节点向头车最终 `cmd_vel` 发布，是该话题唯一的实验控制输出。GUI 接管时关闭键盘：`start_teleop:=false`。不要另开键盘直接向头车 cmd_vel 发速度。

启用前的输入不会重放；禁用、输入超过 0.3 秒未更新、位姿或所需里程计失效时输出零。GUI 遥控应持续更新，例如 20 Hz；键盘节点支持重复发布时采用 repeat_rate 和 key_timeout，旧版本则需要持续按方向键。默认头车上限 0.1 m/s、0.6 rad/s，键盘初始请求为 0.05 m/s、0.15 rad/s。

自动路径时头车 cmd_vel 仍只由路径跟踪器发布，运行节点不发布第二路头车速度；统一停止通过启用参数让跟踪器和后车停止。

就绪表示所需 TF、里程计及路径准备状态有效，不代表电机、电源、急停或遥控器模式已经经过设备诊断。真实停车延迟与遮挡冻结姿态仍需现场验收。

## GUI 最小契约

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/abc_swarm_demo/set_enabled` | `std_srvs/SetBool` | 开始/停止，返回是否接受及原因 |
| `/abc_swarm_demo/reset` | `std_srvs/Trigger` | 停止后重置路径，响应表示已接受请求 |
| `/abc_swarm_demo/status` | `std_msgs/String`，JSON，latched | 状态快照，变化时发布，不是心跳 |
| `/abc_swarm_demo/teleop_cmd` | `geometry_msgs/Twist` | 手动领航速度请求 |

JSON `schema_version=1`，字段包括 `state`、`reason`、`enabled`、`ready`、`mode`、`vrpn_server`、`vrpn_port`。state 为 waiting、ready、running、completed 或 fault。GUI 要结合节点/进程存活判断连接，不能把 latched 消息当作实时心跳。服务调用成功也不替代随后反馈状态的监视。

GUI 可以保持公共后台：

```bash
roslaunch abc_swarm mocap_backend.launch site_config:=/absolute/path/site.yaml
```

然后启动所选实验并传 `start_driver:=false start_vrpn:=false`，手动实验再传 `start_teleop:=false use_rviz:=false`，自动实验传 `use_rviz:=false`。切换前先停止、退出旧实验进程，再启动新实验；后台不需要重启。场地/VRPN 配置变化时停止实验后重启后台。算法参数当前主要在启动时加载，GUI 修改配置后重启实验，不承诺所有参数都可在线修改。

GUI 负责 launch 进程生命周期；本轮提供可复用组件和稳定 ROS 接口，尚未实现 GUI 或集中式进程管理器。
