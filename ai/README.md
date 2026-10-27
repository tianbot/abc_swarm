# 用AI配置客户动捕实验

这是离线、ROS1 Noetic专用的实验编译工具。人或AI填写实验单，固定生成器组合白名单launch；运行不依赖模型服务。设备型号、数量、角色、刚体、定位、后台归属先确认，起点未知保留null。没有实现的设备/数量/算法应明确报告，不修改白名单冒充支持。

## 工作流

1. 读取根AGENTS.md、capabilities.json、当前示例和将使用的launch。
2. 根据客户描述填写experiment.json；只允许契约字段，不接受任意节点、文件路径、shell命令或ROS substitution。
3. 生成到新目录并执行只读检查。生成器拒绝覆盖旧交付。

```bash
python3 ai/generate_launch.py ai/experiment.example.json --output /tmp/customer-demo
```

```bash
python3 ai/generate_launch.py /tmp/customer-demo/experiment.json --output /tmp/customer-demo --check
```

4. 有Noetic环境时按运行单做`roslaunch --nodes`解析；它不启动进程。启动与运动由现场操作人执行。
5. 分别报告生成检查、ROS解析、合成回归及实车验收，不能相互替代。

输出experiment.launch、实验单、中文运行单和composition.sha256.json。哈希覆盖编译器、能力表、组合launch/配置，以及新增节点与共享控制代码；改了算法也会使旧交付检查失败。不要改哈希掩盖版本差异，审阅后从实验单重新生成。

## 契约v3与迁移

contract_version必须为3。v1/v2会明确拒绝，要求阅读本页后迁移；不会把旧实验单静默换成新控制行为。旧视频48个资源及2022年前命令不受影响。

v3的具体变化：manual请求原样转发；跟随阶段不因前车停车切换控制器；删除队形/进度速度缩放及默认点镇定渐变。startup=false只跳过入位，同一节点仍支持同样的标定和反馈来源。path.speed是头车路径速度，不再改三车共用速度上限。

旧ORCA对象的odom_timeout/odom_future_tolerance迁入feedback.yaml，或mocap_control.feedback；stall_timeout改为admission.settings.progress_timeout。ORCA对象现在只含六个几何/求解字段。显式admission.settings使用当前admission.yaml的九个字段，包括停车门槛和途经点容差。新字段请从示例复制，不自动猜测旧现场值。

## 支持的实验

| experiment | 数量与任务 |
| --- | --- |
| manual_triangle / manual_snake | 三台Mini，手动领航；02/03跟01或01→02→03 |
| path_triangle / path_snake | 同上，头车独立预设路径领航 |
| path_single | 单台Mini的独立预设路径 |
| point_stabilization / point_stabilization_orca | 三台各自位置/最终航向，普通或ORCA入位 |
| coordinated_circle / coordinated_triangle | 三台同圆独立跟踪，或三角形逐边到齐转向 |

仅支持tianbot_mini、mocap_vrpn、world。任意数量、混合型号、RMTT/Gazebo新实验不在此交付白名单中；原课程仿真实验继续按视频使用。

## 主要字段

- name与ROS/刚体名称为字母开头的扁平标识符；名称与刚体分别不能重复，一一映射。
- robots顺序就是角色顺序；initial_pose为人工摆位记录，null表示未知，不能当作实测TF。跟随启动目标由实测01和跟随边计算；启用入位会自动移动到这些目标。
- spacing为前车车体系中的偏移：三角后方/左右各spacing，蛇形纵向每边spacing。它不是等边三角形边长。
- path仅用于头车路径，shape为circle/rounded_rectangle，几何为正，圆角矩形边长不小于两倍半径；speed为(0,0.15]m/s，laps为正整数。clockwise明确方向。手动任务path为null。
- trajectory用于三车圆/三角，尺寸至少1.5m，speed不超过0.15m/s；圆形origin是第一台起点而非圆心；三角形改用center=[x,y,yaw]明确表示重心和整体转角，示例采用world/[0,0,0]。旧三角JSON的origin须按新的重心含义人工更新，不自动把旧顶点坐标当重心。reference为world或initial_leader。初始120°不代表运行中自动锁相。
- targets用于点镇定，reference与poses逐车明确给出，是控制目标，不是initial_pose。
- startup中的start_driver/start_vrpn/use_rviz/start_teleop是真布尔值，控制后台/UI归属；不是运动使能。反馈有效后自动运行，无额外enable服务。
- admission可显式开启/关闭跟随前的入位；enabled=true还需要settings和orca，enabled=false只跳过入位。
- planning可选布尔值；true需要ORCA入位，false便于观察不带路线规划的ORCA。不能在不使用ORCA的任务里填true。

## 人工请求与反馈可选设置

manual任务可提供teleop；未填写时生成器会明确写入能力表默认值。它是输入源实际发布值，运行节点不二次截断。初始请求交付范围为linear≤0.15m/s、angular≤3.14rad/s；这是生成约束，不是运行中悄悄限幅。

```json
"teleop": {"linear": 0.09, "angular": 0.7}
```

支持mocap_control的任务可显式选择速度来源、位置容差和逐车校准；calibration为null表示直接使用刚体原点，不表示物理校准已验收。

```json
"mocap_control": {
  "velocity_source": "mocap",
  "position_tolerance": 0.04,
  "calibration": null,
  "velocity_window": {"window": 0.2, "min_interval": 0.08},
  "feedback": {"tf_timeout": 0.5, "odom_timeout": 1.5, "odom_future_tolerance": 0.1}
}
```

velocity_window和feedback可省略，使用包内配置；测量窗口必须满足min_interval≤window≤0.5秒。odom有效期只在选择odom速度来源时影响对应输入。标定对象必须覆盖每台车，字段为yaw_offset与marker_offset[前,左]，单位rad/m；更换标记架必须重核。

## 可观察性和运行边界

新任务发布status(schema_version=3)、effective_config、control_poses、goals和中间速度话题。manual到following且反馈有效、未超时时请求逐值转发；反馈中断或fault立即停车并报告原因。键盘k只停止头车请求，后车继续同一跟随算法；全组退出用Ctrl+C。

ORCA只处理登记车辆入位互避。路线规划可能安排临时让行，01也可能移动；没有墙壁/场地边界模型。路径跟踪段不提供全程避障。不要将生成成功写成到位、避碰或设备已验收。

旧独立path_single链路的reset与新任务不是同一接口。运行单默认用退出重启开始下一轮，避免泛用服务。

维护新能力时同步更新能力表、生成器、正反例和Noetic展开测试；不能仅改白名单。原课程兼容规则、生成只读检查及真实硬件验收边界继续生效。

显式mocap_control可选position_hysteresis，满足0≤余量<position_tolerance，运行单显示实际值。未填写时保留已有客户显式容差的1cm切换余量；不指定mocap_control时沿用通用Mini教学配置（10cm位置容差、7cm切换余量，即3cm进入最终朝向控制）。
