# I003 — Robot Info & UI 改进

## 本次目标

新增 Robot Info 页面支持 gRPC 查询/设置 Master 臂信息，修复导航栏文字截断，改进 Deploy 页面容器选择。

## 本次范围

- 新增 `RobotInfoPage`（system 分组），通过 grpcurl 执行 gRPC 操作
- 修复导航栏按钮文字截断问题（按钮文本 → "进入"）
- Deploy 页面容器名改为可编辑下拉框，支持 SSH 刷新容器列表
- Deploy 预设与 Remote Ops 预设对齐

## 本次不做什么

- 不更新 SDK stubs（GetControlMode/SetControlMode 等新 RPC 暂不加入 SDK）
- 不修改 Remote Ops 页面

---

## 实现前 Plan

Robot Info 页面通过 subprocess 调用 grpcurl，避免 SDK stubs 不完整的问题。页面提供左右臂切换、4 个 unary 查询、3 个 stream 监听、2 个 control mode 设置按钮。导航栏截断问题通过缩短按钮文本解决。Deploy 容器名从 QLineEdit 改为 QComboBox + 刷新按钮。

## 变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `services/robot_info.py` | 新增 | grpcurl unary/stream/set 执行函数 |
| `pages/robot_info_page.py` | 新增 | RobotInfoPage(BasePage) |
| `pages/__init__.py` | 修改 | 导出 RobotInfoPage |
| `ui/navigation.py` | 修改 | 追加 PAGE_SPEC + RobotInfoPage |
| `ui/window.py` | 修改 | 按钮文本改为"进入"，侧边栏 max_width 360→380 |
| `deploy_ops/models.py` | 修改 | 预设 IP/标签对齐 SSH_PRESETS |
| `deploy_ops/widgets.py` | 修改 | 容器名 QComboBox + 刷新按钮 |
| `deploy_ops/controller.py` | 修改 | refresh_containers() 方法，container_input→combo |
| `pages/deploy_page.py` | 修改 | 连接 refresh_button 信号 |

---

## 完成总结 (Completion Summary)

- Robot Info 页面：8 个查询/设置按钮，支持左右臂切换，通过 grpcurl + subprocess 执行
- 导航栏修复：按钮文本从 spec.label 改为 "进入"，侧边栏宽度 +20px
- Deploy 改进：容器名可编辑下拉框，SSH 后点击刷新获取 `docker ps -a` 容器列表

## 验证结果 (Verification)

```bash
cd /home/shangyizhou/code/x2robot
uv run python -c "
from examples.pyqt_app.pages import RobotInfoPage; print('RobotInfoPage OK')
from examples.pyqt_app.services.robot_info import run_unary_query; print('service OK')
from examples.pyqt_app.ui.navigation import PAGE_SPECS; print([s.key for s in PAGE_SPECS])
"
```

## 偏差和风险

无偏离 plan 的情况。grpcurl 方式依赖本地安装的 grpcurl 命令。
