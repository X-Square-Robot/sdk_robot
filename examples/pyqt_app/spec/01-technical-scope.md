# 01-technical-scope.md — 技术约束

## 目录结构

```text
pyqt_app/
  main.py              # QApplication 入口 + 全局异常 hook
  __main__.py          # python -m 入口
  __init__.py           # 空
  workers.py            # TaskWorker (QThread 包装)

  ui/                   # 主窗口 UI 组装
    window.py           # AppWindow — QMainWindow, header/sidebar/content/log/status
    navigation.py       # PAGE_SPECS 列表 + PageSpec 导入
    styles.py           # build_stylesheet() 全局 QSS

  pages/                # 页面层 (View)
    base_page.py        # BasePage — run/stop 按钮, TaskWorker 生命周期管理
    check_connect_page.py
    robot_control_page.py
    arm_control_page.py
    align_master_slave_page.py
    remote_ops_page.py

  controllers/          # 薄控制器层
    lifecycle.py        # shutdown_pages() — 窗口关闭时清理所有页面 worker
    robot_pages.py      # CheckConnect/RobotControl/ArmControl/AlignMasterSlave 控制器

  services/             # 业务逻辑 (SDK 调用)
    common.py           # LogFn, CancelFn 类型别名
    connectivity.py     # run_check_connect()
    robot_control.py    # run_robot_control()
    arm_control.py      # run_arm_control()
    arm_motion.py       # move_by_end_pose(), move_by_joint_positions(), stream_*, TOPP-RA
    align_master_slave.py  # run_move_slave_arm(), run_align_master_to_slave()

  models/               # 数据结构
    page_spec.py        # PageSpec dataclass
    service.py          # ServiceResult dataclass
    requests.py         # ArmControlRequest dataclass

  core/                 # SSH 基础设施
    ssh_client.py       # SshClient — subprocess 封装, exec/exec_streaming
    command_worker.py   # SshCommandWorker — QThread SSH 执行

  remote_ops/           # RemoteOps 专属 MVC
    models.py           # QuickCommand, SshPreset, QUICK_COMMANDS, SSH_PRESETS
    widgets.py          # SshBarWidget, ContainerPanelWidget, CommandPanelWidget
    controller.py       # RemoteOpsController
```

## 模块边界

- **pages/** — 纯 View，只做 UI 组装和信号连接，不直接调 SDK
- **controllers/** — 薄层，从 page 收集参数 → 调 services → 返回 ServiceResult
- **services/** — 纯函数，接收参数 + log/cancel 回调 → 返回 ServiceResult，不依赖 Qt
- **models/** — 纯 dataclass，无逻辑
- **core/** — SSH 基础设施，不依赖 SDK，不依赖 UI 页面
- **remote_ops/** — 自包含 MVC，不跨层引用其他 controller/page

## 核心设计模式

### BasePage 生命周期

```
BasePage.start_worker(task) → TaskWorker(task).start()
  → started_task signal → set_running_state(True)
  → task(log, cancelled) 在后台线程执行
  → task_succeeded(ServiceResult) → append_log + _handle_success
  → task_failed(str) → append_log + QMessageBox
  → task_finished → set_running_state(False), worker = None
```

### 全局 Server 共享

- sidebar 顶部 `server_input` 被所有页面通过 `get_server()` 读取
- 默认值 `localhost:50051`

### 页面分组切换

- header 中"系统控制 / 远程运维"两个 group button
- sidebar 按 `PageSpec.group` 过滤显示
- `AppWindow.set_active_group(group)` → 显示对应 nav row → 切换到该组第一个页面

### TaskWorker

- 继承 `QThread`，通过 pyqtSignal 将 `ServiceResult` 传回 UI 线程
- `request_stop()` → `requestInterruption()`，由 `CancelFn` 在 worker 中检查

## 命名规范

- 页面类: `XxxPage`，继承 `BasePage`
- 控制器类: `XxxController`
- 服务函数: `run_xxx()`，返回 `ServiceResult`
- 模型: 用 `@dataclass(slots=True)` dataclass
- 文件: snake_case

## 依赖

- PyQt6 — UI 框架
- x2robot — 机器人 SDK (gRPC)
- toppra — 轨迹规划
- scipy — Slerp 插值
- numpy — 数值计算
- grpc (grpcio) — gRPC 错误处理

## 错误处理

- services 中 gRPC 错误通过 `grpc.RpcError` 捕获，返回 `ServiceResult(False, ...)`
- UI 层通过 `_install_excepthook` 全局捕获未处理异常
- Worker 中异常 → `task_failed` signal → QMessageBox + 日志
