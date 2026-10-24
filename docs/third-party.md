# 随包提供的第三方代码

## simple-pid 2.0.0

当前使用的 PID 实现直接随仓库提供，不再要求客户 pip 安装。

- 上游：[m-lundberg/simple-pid v2.0.0](https://github.com/m-lundberg/simple-pid/tree/v2.0.0)。
- 本次从环境中已使用的 simple-pid 2.0.0 发行包复制 __init__.py 与 pid.py，算法代码未修改。
- 文件位于 src/simple_pid，一层包目录；保留 `from simple_pid import PID`，原视频脚本不改。
- 原始许可证为 MIT，完整文本保留在 src/simple_pid/LICENSE.md；版本、来源和复制文件 SHA-256 记录在 UPSTREAM.json。
- catkin 同时安装 abc_swarm 与 simple_pid 两个 Python 包。install 中的许可证/来源记录位于 share/abc_swarm/licenses/simple_pid。
- requirements.txt 保留为兼容入口，只含注释；CI 不再 pip 安装 simple-pid。ROS、NumPy、SciPy 等既有依赖仍来自系统环境。

本轮不卸载系统已安装的 simple-pid，不修改全局 Python 环境。编译并 source 本工作空间后，验证 simple_pid.__file__ 来确认实际导入工作空间版本；不要只凭能 import 就认定已移除外部依赖。

测试用 Python -S 禁用 site-packages，并仅加入本仓库 src，验证原导入方式、限幅、增益更新、积分限幅及 reset。构建/安装还需验证导入来源，避免机器上恰好安装的外部版本掩盖问题。


catkin 同时提供多个 Python 包时，其开发环境转发模块可能把 src 加到搜索路径前面；abc_swarm 的包初始化使用标准 pkgutil.extend_path，保留生成的 cfg 子包可见性。已补“先导入随包 PID，再导入动态配置”的独立进程回归，不修改上游 PID 算法或原教学脚本。

本轮显式更新 setup.py/CMake 安装规则及 CI：移除 pip 安装，增加随包 PID 的独立导入测试、安装来源与许可证检查。冻结课程检查器、基线和例外未改；实际工作空间只重新编译，没有启动或重启设备，也未卸载全局 Python 包。

验证结果：56 项本地测试、Noetic 构建/安装及 48 个旧资源兼容检查通过。禁用 site-packages、仅提供 src、devel 或 install 路径时 PID 均可导入；实际工作空间的四个 PID 使用节点已做非 main 导入检查。远端 CI 未推送运行。


## ORCA 两两约束参考

`src/abc_swarm/orca.py` 的两两半平面构造译自固定提交的 RVO2 Agent.cc，其他小规模凸投影、差速限制和停机处理为本地实现。Apache-2.0 许可证及参考文件 SHA-256 位于 licenses/orca，catkin 安装到 share/abc_swarm/licenses/orca；没有额外 pip/native 库依赖。细节见 [ORCA 指南](orca.md)。
