# spec/README.md — AI 工作流规则

本文档是 AI 助手的入口文档。**AI 在每次开始迭代前必须阅读本文件。**

---

## AI 启动流程

1. 阅读 `spec/README.md`（本文件）
2. 阅读 `spec/00-project-brief.md` — 了解项目是什么
3. 阅读 `spec/01-technical-scope.md` — 了解技术约束
4. 阅读 `spec/02-roadmap.md` — 了解当前迭代位置
5. 阅读 `spec/03-changelog.md` — 了解最近变更
6. 找到当前 iterator 文档，开始工作

---

## AI 执行 iterator 规则

- 每个迭代对应一个 `spec/iterators/IXXX-<name>.md` 文件
- iterator 文档是**施工单**，代码放在项目对应目录（`pages/`、`services/` 等）
- iterator 文档不存放代码

## AI 完成 iterator 后

1. 更新 iterator 文档的「Completion Summary」和「Verification」章节
2. 在 `spec/03-changelog.md` 顶部追加一条新纪录
3. 如果 roadmap 有变化，更新 `spec/02-roadmap.md`

---

## 文档规则

- 本项目**不需要** tests / examples / benchmarks（项目本身就是测试工具）
- 不需要写 ADR，除非涉及跨模块的架构决策
- 所有 spec 文档使用中文编写
