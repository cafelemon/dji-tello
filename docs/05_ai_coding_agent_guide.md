# 05 AI Coding 工作协议

1. 以 `docs/01_prd.md`、`docs/03_architecture.md` 和 `docs/07_decisions.md` 为实现事实源。
2. 修改公共话题、消息、服务、状态或默认安全参数前，先更新决策记录。
3. 保持 Transport、Vision、FlightManager 的权限边界；Vision 不得直接发布最终控制。
4. 先为 PID、解析器、协议或状态迁移增加失败测试，再修改实现。
5. 不提交 `build/`、`install/`、`log/`、模型、Engine、rosbag 或第三方源码。
6. 不写入用户绝对路径、密钥、Wi-Fi 凭据或设备序列号。
7. 每次交付记录变更文件、命令、结果、受阻项和剩余风险。
8. 普通项目文档只陈述设计、用法和已取得的软件证据；硬件测试信息统一维护在 `08_hardware_test.md`。
