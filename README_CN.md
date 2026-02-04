# X2Robot SDK

<div align="center">
  <img src="docs/logo.png" alt="X2Robot Logo" width="200"/>
</div>

**Language / 语言**: [English](README.md) • [中文](README_CN.md)

## 简介

X2Robot SDK 提供了用于控制 X2 机器人的 Python 客户端库。

## 系统要求

### 硬件要求

- CPU: Intel i5 或同等性能
- 内存: 8GB 或更多
- 存储: 100GB 可用空间
- 网络: 稳定的网络连接

### 软件环境

| 组件 | 版本要求 | 说明 |
|------|---------|------|
| 操作系统 | Ubuntu 22.04/24.04 | 推荐使用 LTS 版本 |
| Python | 3.10+ | SDK 开发语言 |

### 环境搭建

Ubuntu 和 Python 安装说明请参考：

- Ubuntu 22.04 LTS 安装: [Ubuntu 官方安装指南](https://ubuntu.com/download)
- Python: [Python 官方网站](https://www.python.org/)

## 网络配置

### 通过网线连接机器人

1. 使用 USB 转网口模块和网线将 PC 连接到机器人

2. 配置 PC 的局域网 IP：

   - 打开 PC 设置 → 网络 → 有线配置
   - 设置 IP 为手动配置
   - 配置 IP 地址和子网掩码（例如：192.168.10.10/24）
   - 点击应用

3. 重启网络接口：

   - 关闭然后重新打开网络接口以应用更改

4. 验证连接：

   ```bash
   # 从 PC ping 机器人以确认连接，机器人IP已经预先配置好:192.168.10.1
   ping 192.168.10.1
   ```

## SDK 安装

### 安装依赖

Ubuntu 24.04 和 Ubuntu 22.04 的默认 Python 版本可能不同。请根据您的 Python 版本安装虚拟环境包

```bash
sudo apt-get update
sudo apt install python3-pip
sudo apt install ffmpeg
# Ubuntu 22.04 - 安装 Python 虚拟环境
sudo apt install python3.10-venv

# Ubuntu 24.04 - 安装 Python 虚拟环境
# sudo apt install python3.12-venv
```

### 配置 pip 镜像源（推荐）

由于某些依赖包下载速度较慢，建议配置 pip 镜像源以加速下载：

```bash
# 配置清华大学镜像源（推荐）
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 或者配置阿里云镜像源
# pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/

# 或者配置腾讯云镜像源
# pip config set global.index-url https://mirrors.cloud.tencent.com/pypi/simple/

# 验证配置是否生效
pip config list
```

**注意：** 配置镜像源后，pip 将从国内镜像站点下载包，可以显著提高下载速度。如果遇到特定包无法从镜像源下载，可以临时使用官方源：

```bash
pip install -i https://pypi.org/simple/ 包名
```

### 虚拟环境设置

```bash
# 创建虚拟环境
python3 -m venv .venv

# 激活虚拟环境
source .venv/bin/activate
```

### 下载并安装 SDK

下载最新的 SDK 包并安装：

```bash
# 安装 whl 包
pip install x2robot-1.0.0-py3-none-any.whl
```

## 快速开始

### 第一个程序 - 连接测试

创建 `check_connect.py`：

```python
from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import PingRequest


def main(
    server: Annotated[str, typer.Option(help="服务器地址，例如：localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    request = PingRequest(payload="Hello, X2Robot!")

    response = robot.benchmark.ping(request)
    if response.payload == "Pong to: Hello, X2Robot!":
        print("连接到 X2Robot SDK 服务器成功！")
        print(f"响应负载：[{response.payload}]")
    else:
        print("意外响应：", response.payload)


if __name__ == "__main__":
    typer.run(main)
```

运行测试程序：

```bash
python3 check_connect.py --server 192.168.10.1:50051

# 预期输出：
# 连接到 X2Robot SDK 服务器成功！
# 响应负载：[Pong to: Hello, X2Robot!]
```

### 运行示例

详细示例请参考 [示例文档](examples/README_CN.md)。

## API 文档

完整的 API 参考请查看 [API 文档](docs/API_Quanta_X1_CN.md)。

## 示例

代码示例请查看 [示例目录](examples/)。

## 常见问题

### 如何连接机器人？

```python
from x2robot import connect

robot = connect('x2://localhost:50051')
```
