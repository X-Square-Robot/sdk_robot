# X2Robot SDK

<div align="left">
  <img src="docs/logo.png" alt="X2Robot Logo" width="200"/>
</div>

**Language / 语言**: [English](README.md) • [中文](README_CN.md)

## 简介

X2Robot SDK 提供了用于控制 X2 机器人的 Python 库。

## 系统要求

### 硬件要求

- CPU: Intel i5 或同等性能
- 内存: 8GB 或更多
- 存储: 100GB 可用空间
- 网络: 稳定的网络连接

### 软件环境要求

| 组件 | 版本要求 | 说明 |
|------|---------|------|
| 操作系统 | Ubuntu 22.04/24.04 (x86_64) | 推荐使用 LTS 版本 |
| Python | 3.10+ | SDK 开发语言 |

## 快速开始

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

   ```bash
   # 查看网口
   ifconfig

   # 重启网口
   sudo ifconfig eth0 down
   sudo ifconfig eth0 up
   ```

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

从github的Release下载最新的 SDK 包并安装：

```bash
# 安装 whl 包
pip install x2robot-1.0.0-py3-none-any.whl
```

### 卸载SDK

更新SDK版本时需要将之前版本SDK卸载，注意卸载也需要在SDK虚拟环境中执行

```bash
# 卸载x2robot，假设你之前的版本是v0.1.8
pip uninstall x2robot-0.1.8-py3-none-any.whl
```

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
# Connection to X2Robot SDK server successful!
# Response payload:[Pong to: Hello, X2Robot!]
```

### 确认网络环境已关闭代理

在一个终端执行相机图像获取示例脚本：<https://github.com/X-Square-Robot/sdk_robot/blob/main/examples/camera.py>

```bash
python3 camera.py head rgb-stream --server 192.168.10.1:50051
```

在另一个终端执行

```bash
sudo netstat -anp | grep 50051
```

确认只有名为python3的进程有创建50051端口的tcp连接
![图片](docs/no_proxy.png)
如果有其他进程也创建了50051端口的连接，例如下面这种情况：
![图片](docs/has_proxy.png)
需要先关闭这个进程，确保网络代理是关闭的，否则容易导致使用过程中连接中断。

### 运行示例

详细示例请参考 [示例文档](examples/README_CN.md)。

## API 文档

完整的 API 参考请查看:

- 量子1号Pro: [API 文档](docs/API_Quanta_X1_CN.md)。
- 量子2号: [API 文档](docs/API_Quanta_X2_CN.md)。
- 桌面六轴臂系列产品：[API 文档](docs/API_Desktop_CN.md)。

## 示例

代码示例请查看 [示例目录](examples/)。

## 采集数据并转换成Lerobot格式的数据

用户可自定义数据流程：

- (1) 修改`examples\data_collection`目录下的数据采集代码 ，只采集所需字段；
- (2) 编辑 `tools/convert_to_lerobot.py`（如 `ROBOT_DATA_CONFIG`、`state_source_mapping`、`action_source_mapping`）以定义需要转换的数据。

请参考[数据采集示例](examples/data_collection_example.py)和[转换成lerobot数据的脚本](tools/convert_to_lerobot.py)

convert_to_lerobot.py使用请参考[lerobot数据格式转换脚本说明](tools/README_CN.md)

## 样例

我们提供了一个简单的样例，用于演示 SDK 推理流程与交互方式。
目前仅支持以下型号产品：

- 量子1号Pro：[SDK 推理](samples/quanta_x1/README_CN.md) 样例
- 桌面六轴臂系列：[SDK 推理](samples/desktop/README_CN.md) 样例

## 常见问题

### Q1：执行示例代码导包报错？

如下图所示：

![图片](docs/Q1.png)

这种情况一般是没有source 虚拟环境，用户请根据自己虚拟环境安装目录，执行以下命令：

```bash
source .venv/bin/activate
```

### Q2：运行回放脚本报错， 运行推理脚本报错

A:

1. 在采的数据没问题的情况下，运行回放脚本报错，可能是当前模式为数采模式，切换成空闲模式才能回放
2. 推理脚本的运行也需要切换到空闲模式下，才能运行成功，否则报错如下图所示：

![图片](docs/Q2.jpeg)
用主臂切换成空闲模式才能运行成功。

### Q3：建图后，给出目标点定位导航，机器人不移动，

A：
执行定位导航时，机器人不移动， 有时候会报错，如下图所示：
![图片](docs/Q3.jpeg)
机器人为了保证定位导航的准确性，要求建图过程中移动距离必须达到 3.5 m 或旋转角度达到 210 度才会认为地图有效！建议执行开始建图后，在空旷场地下前后充分移动， 再结束建图， 参考示例代码 chassis_control.py。
将control-mode切换成map，会执行先建图再用相对定位的方式导航到目标点

```bash
python3 chassis_control.py --server 192.168.10.11:50051 --control-mode map
```

### Q4：运行中过程中机器人双臂完全掉电

A:
目前的机制是，整机电量低于7%时，手臂会优先掉电，手臂关节电机全部自动松闸，建议控制的程序中添加实时监控电量，获取当前电量的接口用法参考示例代码system.py：

```python
result = robot.system.get_dynamic_info()
print(f"dynamic_info:{result.power_status.value}")
```

### Q5：运行报错 Connection reset by peer？

若使用 SDK 时出现如下图的连接异常
![图片](docs/Q5.jpeg)

A:
请排查本机代理设置，确认代理是关闭的。排查网络代理请参照[确认网络环境已关闭代理](#确认网络环境已关闭代理)。
部分企业终端安全软件（如 qzhddr 等）会将所有外网连接强制走代理。在高频、大流量的数据传输场景下，这类代理性能不足，容易导致连接超时或中断。在测试或使用 SDK 时暂时关闭代理，以排除网络环境对连接稳定性的影响。

### Q6：建图结束保存地图失败

建图结束，保存地图提示“map save failed”

![Image](docs/Q6.png)

A:
和Q3一样的原因，机器人为了保证定位导航的准确性，要求建图过程中移动距离必须达到 3.5 m 或旋转角度达到 210 度才会认为地图有效！建议执行开始建图后，在空旷场地下前后充分移动， 再结束建图。

### Q7: 执行convert_to_lerobot.py脚本报错“File exists： 'lerobot_data'”？

![Image](docs/file_exists_error.png)

A:
指定的输出目录lerobot_data已经存在，请指定不同的输出目录或者删掉/重命名之前的目录

### Q8：切换SDK工作模式时出现："Task operation failed"?

控制或者数据回放时出现下面的错误

![Image](docs/sdk_mode_failed.PNG)

A: 当前处在其他工作模式，需要在主端控制界面切换到空闲模式

## 开发注意事项

- 版本兼容：请使用匹配的固件和SDK版本
- 环境限制：请在满足要求的软件环境下运行SDK
- 频率限制：调用控制接口的频率不要超过200HZ
- 规格限制：调用控制接口的参数不得超过各关节的位置限制规格
- 异常处理：请正确处理接口异常，调用接口出现问题时请参考FAQ或者联系技术解决
