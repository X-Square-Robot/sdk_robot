# 02-roadmap.md — 路线图

## 迭代记录

| Iterator | 日期 | 描述 | 状态 |
|----------|------|------|------|
| I001 | 2026-05-13 | Codex 架构重构（当前版本） | ✅ done |
| I002 | 2026-05-13 | 部署工具 | ✅ done |
| I003 | 2026-05-13 | Robot Info & UI 改进 | ✅ done |
| I004 | 2026-05-14 | SDK demos 分层重构 | 📋 planned |

## 当前状态

项目已完成 Codex 重构，建立了清晰的 MVC + services 分层架构。5 个业务页面均可正常运行。
Robot Info 页面已从 grpcurl 迁移到 SDK stubs，SDK 从 proto 重新生成。

## 未来方向（待定）

- I004: SDK demos 分层 — 抽离 SDK 调用为独立的 `sdk_demos/` 函数库
- 用户根据实际使用反馈决定后续迭代
- 可能的增强方向：更多预设配置、状态持久化、多机器人管理
