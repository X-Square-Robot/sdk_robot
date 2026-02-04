# X2Robot SDK

<div align="left">
  <img src="docs/logo.png" alt="X2Robot Logo" width="200"/>
</div>

**Language / 语言**: [English](README.md) • [中文](README_CN.md)

## Introduction

X2Robot SDK provides Python client libraries for controlling X2 robots.

## System Requirements

### Hardware Requirements

- CPU: Intel i5 or equivalent performance
- Memory: 8GB or more
- Storage: 100GB available space
- Network: Stable network connection

### Software Environment

| Component | Version Requirement | Description |
|-----------|---------------------|-------------|
| Operating System | Ubuntu 22.04/24.04 | LTS versions recommended |
| Python | 3.10+ | SDK development language |

### Environment Setup

For Ubuntu and Python installation instructions, please refer to:

- Ubuntu 22.04 LTS installation: [Official Ubuntu Installation Guide](https://ubuntu.com/download)
- Python: [Python Official Website](https://www.python.org/)

## Network Configuration

1. Connect your PC to the robot using a USB-to-Ethernet adapter module and an Ethernet cable

2. Configure Your PC's LAN IP:

   - Open PC Settings → Network → Wired configuration
   - Set IP to manual configuration
   - Configure IP address and subnet mask (e.g., 192.168.10.10/24)
   - Click Apply

3. Restart Network Interface:

   - Turn off and then turn on the network interface to apply changes

4. Verify Connectivity:

   ```bash
   # Ping robot from PC to confirm connection (robot IP: 192.168.10.1)
   ping 192.168.10.1
   ```

## SDK Installation

### Install Dependencies

The default Python version may differ between Ubuntu 24.04 and Ubuntu 22.04. Install virtual environment package according to your Python version

```bash
sudo apt-get update
sudo apt install python3-pip
sudo apt install ffmpeg
# Ubuntu 22.04 - Install Python virtual environment
sudo apt install python3.10-venv

# Ubuntu 24.04 - Install Python virtual environment
# sudo apt install python3.12-venv
```

### Virtual Environment Setup

```bash
# Create virtual environment
python3 -m venv .venv

# Activate virtual environment
source .venv/bin/activate
```

### Download and Install SDK

Download the latest SDK package and install:

```bash
# Install whl package
pip install x2robot-1.0.0-py3-none-any.whl
```

## Quick Start

### First Program - Connection Test

Create `check_connect.py`:

```python
from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import PingRequest


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    request = PingRequest(payload="Hello, X2Robot!")

    response = robot.benchmark.ping(request)
    if response.payload == "Pong to: Hello, X2Robot!":
        print("Connection to X2Robot SDK server successful!")
        print(f"Response payload:[{response.payload}]")
    else:
        print("Unexpected response:", response.payload)


if __name__ == "__main__":
    typer.run(main)
```

Run the test program:

```bash
python3 check_connect.py --server 192.168.10.1:50051

# Expected output:
# Connection to X2Robot SDK server successful!
# Response payload:[Pong to: Hello, X2Robot!]
```

### Run Examples

See [Examples README](examples/README.md) for detailed examples.

## API Documentation

See [API Documentation](docs/API_Quanta_X1.md) for complete API reference.

## Examples

See [Examples](examples/) for code examples.

## FAQ

### How to connect to robot?

```python
from x2robot import connect

robot = connect('x2://localhost:50051')
```
