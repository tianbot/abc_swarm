# 文档入口与迁移建议

`docs/source/*.rst`、原图片和旧 Read the Docs 配置均保留原位、原内容，作为原课程和历史网站依据。根目录继续使用原 `README.rst`，不增加第二个 README 入口。

开发原则记录在根 [AGENTS.md](../AGENTS.md)：节点统一在 nodes，Python 算法可复用，设备和场景通过参数与 launch 组合。

当前用户指南：

- [控制参数核对、首轮调参与 ORCA 路线](control-tuning.md)

- [新实验：三车点镇定到位与最终朝向](point-stabilization.md)

- [AI 配置客户实验：设备清单生成 launch](../ai/README.md)

- **[三车现场测试清单：明天直接从这里开始](field-test.md)**

- [旧 main 兼容检查、CI 和你需要做的事](course-compatibility.md)

- [动捕实验、摆位与启动](mocap-demos.md)
- [统一启动及 GUI 接口](demo-runtime.md)
- [从 main 重组的取舍与验证](main-reorg.md)

建议新增和后续更新集中发布到 docs.tianbot.com，保持已有课程 URL，不再同时维护两套新增教学内容。原 RST 暂留归档，不整批转换、删除或改动旧站。本轮提供 [可迁入 Tianbot Docs 的页面草稿](website/mocap-experiments.md)，没有修改外部网站仓库、导航或部署。

迁移时先在 Tianbot Docs 现有 swarm 章节旁增加“动捕三车实验”，链接原视频课程；经现场验收后补充效果图/录像和设备清单。旧实验页面继续兼容原命令；新页面使用 demo_mocap 命名。完成网站预览、链接与命令核对后再决定发布，不能把本仓库草稿当成已上线文档。

原站配置指向 `docs/conf.py`，而本仓库实际配置在 `docs/source/conf.py`；原 toctree 也有缺失页面。这些是原 main 已存在的问题，当前保持原历史材料，不把旧站构建成功列为本轮交付。后续若继续维护旧站，应单独修复，避免夹带进动捕代码迁移。
