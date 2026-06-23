# I002 — 部署工具

## 本次目标

新增 Deploy 页面，支持将本地编译产物通过 rsync 同步到机器人并替换容器内文件，分阶段按钮执行。

## 本次范围

- 新增 `deploy_ops/` 包（models, widgets, controller）
- 新增 `pages/deploy_page.py`（DeployPage）
- 注册到 ops 分组导航

## 本次不做什么

- 不做自动检测本地目录变化
- 不做部署历史记录
- 不做多机器人并行部署

---

## 实现前 Plan

遵循既有 MVC 分层，在 `deploy_ops/` 中自包含所有部署逻辑。页面 UI 分为 4 块：
SSH 连接栏 → 部署源（本地目录 + 子目录勾选）→ 部署目标（容器配置）→ 分步按钮（Rsync / 容器替换 / 查看日志）。

## 变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `deploy_ops/__init__.py` | 新增 | 导出 DeployController |
| `deploy_ops/models.py` | 新增 | DeployPreset + DEPLOY_PRESETS |
| `deploy_ops/widgets.py` | 新增 | DeploySshBar, DeploySourcePanel, DeployTargetPanel, DeployStepsPanel |
| `deploy_ops/controller.py` | 新增 | DeployController 全部业务逻辑 |
| `pages/deploy_page.py` | 新增 | DeployPage(BasePage) |
| `pages/__init__.py` | 修改 | 导出 DeployPage |
| `ui/navigation.py` | 修改 | 追加 DeployPage 到 PAGE_SPECS |

---

## 完成总结 (Completion Summary)

新增 Deploy 页面，3 阶段部署流程：

1. Rsync 同步 — 本地 subprocess 执行 rsync，逐行输出日志
2. 容器替换 — 通过 SSH 执行 docker stop / rm / cp / restart 序列
3. 查看日志 — docker logs --tail 200

每步独立按钮，通过 BasePage.start_worker 在后台线程执行，互不阻塞。

## 验证结果 (Verification)

```bash
cd /home/shangyizhou/code/x2robot
python -c "from examples.pyqt_app.pages import DeployPage; print('OK')"
python -c "from examples.pyqt_app.deploy_ops import DeployController; print('OK')"
python -c "from examples.pyqt_app.ui.navigation import PAGE_SPECS; print([s.key for s in PAGE_SPECS])"
```

## 偏差和风险

无偏离 plan 的情况。rsync 依赖本地安装的 `rsync` 和 `sshpass` 命令。
