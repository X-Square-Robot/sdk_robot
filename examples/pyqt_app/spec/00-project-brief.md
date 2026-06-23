# 00-project-brief.md — 项目简介

## 项目名称

X2 Examples GUI (pyqt_app)

## 一句话描述

基于 PyQt6 的桌面 GUI 工作台，用于控制和调试 X2Robot SDK 的双臂机器人。

## 核心功能

- **连接检测**：验证 SDK gRPC server 可达性
- **机器人控制**：homing / stop / recover 基础系统操作
- **单臂控制**：move（TOPP-RA 轨迹）/ stream（关节状态 & 末端位姿）两种模式
- **主从臂对齐**：两步流程——移动从臂到目标位姿 → 主臂对齐
- **远程运维**：SSH 连接远端主机，Docker 容器管理，命令执行

## 目标用户

机器人开发与调试工程师，用于日常调试、测试和运维 X2Robot 双臂平台。

## 技术栈

- **UI**: PyQt6 (Qt 6.x, Fusion style)
- **SDK**: x2robot (gRPC-based)
- **轨迹规划**: TOPP-RA + scipy (Slerp)
- **远程连接**: SSH (sshpass + subprocess)
- **运行时**: Python 3.12
