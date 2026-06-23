# I001 — Codex 架构重构

## 本次目标

将 pyqt_app 从单文件/松散脚本重构为分层 MVC + services 架构的 PyQt6 GUI 工作台。

## 本次范围

- 建立清晰的分层目录结构（pages / controllers / services / models / core / remote_ops）
- 实现 BasePage 基类，统一管理 TaskWorker 生命周期
- 实现 5 个业务页面：CheckConnect、RobotControl、ArmControl、AlignMasterSlave、RemoteOps
- 实现全局 Server 输入框共享机制
- 实现页面分组切换（系统控制 / 远程运维）
- 实现 TOPP-RA 平滑轨迹执行（关节空间 + 末端位姿 Slerp）
- 实现 SSH 远程命令执行和 Docker 容器管理
- 全局异常 hook 和 Fusion 风格 QSS 样式

## 本次不做什么

- 不写单元测试（项目本身就是测试工具）
- 不写 examples（项目本身在 examples/ 目录下）
- 不做配置持久化（预设数据写死在代码中）
- 不做多窗口或 i18n

---

## 实现前 Plan

1. 建立 models/ (PageSpec, ServiceResult, ArmControlRequest)
2. 建立 workers.py (TaskWorker)
3. 建立 pages/base_page.py (统一生命周期)
4. 建立 services/ (SDK 调用封装)
5. 建立 controllers/ (薄控制器)
6. 建立 ui/ (AppWindow + navigation + styles)
7. 建立 core/ + remote_ops/ (SSH 功能)
8. 逐页实现 + 连接信号
9. 全局异常 hook + QSS

## 变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `main.py` | 新增 | QApplication 入口 + 全局异常 hook |
| `__main__.py` | 新增 | `python -m pyqt_app` 入口 |
| `__init__.py` | 新增 | 包初始化 |
| `workers.py` | 新增 | TaskWorker — QThread 包装 |
| `ui/window.py` | 新增 | AppWindow — 主窗口完整布局 |
| `ui/navigation.py` | 新增 | PAGE_SPECS 页面注册表 |
| `ui/styles.py` | 新增 | build_stylesheet() 全局 QSS |
| `pages/base_page.py` | 新增 | BasePage — run/stop/worker 管理 |
| `pages/check_connect_page.py` | 新增 | 连接检测页面 |
| `pages/robot_control_page.py` | 新增 | 机器人控制页面 |
| `pages/arm_control_page.py` | 新增 | 单臂控制页面 |
| `pages/align_master_slave_page.py` | 新增 | 主从对齐页面 |
| `pages/remote_ops_page.py` | 新增 | 远程运维页面 |
| `controllers/lifecycle.py` | 新增 | shutdown_pages() 关闭清理 |
| `controllers/robot_pages.py` | 新增 | 4 个系统控制页面控制器 |
| `services/common.py` | 新增 | LogFn, CancelFn 类型别名 |
| `services/connectivity.py` | 新增 | run_check_connect() |
| `services/robot_control.py` | 新增 | run_robot_control() |
| `services/arm_control.py` | 新增 | run_arm_control() |
| `services/arm_motion.py` | 新增 | TOPP-RA 轨迹 + stream |
| `services/align_master_slave.py` | 新增 | 主从对齐两步流程 |
| `models/page_spec.py` | 新增 | PageSpec dataclass |
| `models/service.py` | 新增 | ServiceResult dataclass |
| `models/requests.py` | 新增 | ArmControlRequest dataclass |
| `core/ssh_client.py` | 新增 | SshClient — subprocess SSH |
| `core/command_worker.py` | 新增 | SshCommandWorker — QThread SSH |
| `remote_ops/models.py` | 新增 | QuickCommand, SshPreset |
| `remote_ops/widgets.py` | 新增 | SSH/Docker/Command 面板 |
| `remote_ops/controller.py` | 新增 | RemoteOpsController |

---

## 完成总结 (Completion Summary)

已完成 Codex 架构重构，建立了清晰的分层结构：

- **pages/** — 5 个页面，全部继承 BasePage
- **controllers/** — 薄控制器层，连接 UI 信号到 services
- **services/** — 纯 SDK 调用函数，返回 ServiceResult
- **models/** — dataclass 数据结构
- **core/** — SSH Client + Worker
- **remote_ops/** — RemoteOps 自包含 MVC
- **ui/** — AppWindow 组装所有 UI 组件 + QSS 样式

核心设计：BasePage → TaskWorker → ServiceResult 信号链；全局 Server 共享；页面分组切换。

## 验证结果 (Verification)

```bash
# 语法检查
python -m py_compile examples/pyqt_app/main.py
python -m py_compile examples/pyqt_app/workers.py

# 模块导入检查
cd /home/shangyizhou/code/x2robot
python -c "from examples.pyqt_app.main import main; print('import OK')"
```

✅ 所有模块可正常导入。

## 偏差和风险

- 无偏差，按 plan 完整执行
- RemoteOps 页面使用独立的 SshCommandWorker（非 TaskWorker），因为 SSH 需要流式输出能力，TaskWorker 的 ServiceResult 模式不适合长连接场景。这是有意为之的设计差异
