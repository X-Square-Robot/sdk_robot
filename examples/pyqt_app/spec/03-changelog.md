# 03-changelog.md — 变更日志

## 2026-05-13 — I003 Robot Info & UI 改进

- 新增 `RobotInfoPage` 页面，通过 grpcurl 查询/设置 Master 臂信息
- 支持 GetControlMode / GetJointStates / GetEndPose / GetGripperPosition 查询
- 支持 GetJointStatesStream / GetEndPoseStream / GetGripperStateStream 流式监听
- 支持 SetControlMode（ENDPOSE_TELEOP=13 / JOINT_TELEOP=14）
- 修复导航栏按钮文字截断（按钮改为"进入"，侧边栏 +20px）
- Deploy 页面容器名改为可编辑下拉框，支持 SSH 刷新 `docker ps -a` 列表
- Deploy 预设与 Remote Ops 预设 IP/标签对齐

## 2026-05-13 — I002 部署工具

- 新增 `deploy_ops/` 包（models, widgets, controller）
- 新增 `DeployPage` 页面，支持分阶段部署流程
- Step 1: Rsync 本地编译产物到机器人 Host 临时目录
- Step 2: SSH 远程执行容器替换（stop → rm → cp → restart）
- Step 3: 查看容器日志确认运行状态
- 支持部署预设配置（多机器人 IP）
- 注册到远程运维（ops）分组

## 2026-05-13 — I001 Codex 架构重构

- 建立 spec/ 文档体系（项目简介、技术约束、路线图、变更日志）
- 建立分层架构：pages / controllers / services / models / core / remote_ops
- 实现 BasePage 统一管理 TaskWorker 生命周期
- 实现 5 个业务页面：连接检测、机器人控制、单臂控制、主从对齐、远程运维
- 实现全局 Server 输入框共享机制
- 实现页面分组切换（系统控制 / 远程运维）
- 实现 TOPP-RA 平滑轨迹执行（关节空间 + 末端位姿）
- 实现 SSH 远程命令执行和容器管理
- 全局异常 hook 和 Fusion 风格 QSS
