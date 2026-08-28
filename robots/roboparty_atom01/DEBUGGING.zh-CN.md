# Atom01 调试参考

本页用于定位 Atom01 的 RynnRCP 安装、硬件连接、标定、Server、协议和策略问题。
首次安装和启动先阅读 [Atom01 README](README.zh-CN.md)。

> **调试安全**
>
> 配置程序、测试策略和 Server 都可能访问真机电机。连接前可靠支撑机器人，准备好
> 急停，并确认当前只有一个电机控制进程。先完成只读检查，再执行动作验证。

## 从哪里开始

按下列顺序检查，当前层通过后再进入下一层：

| 层级 | 通过标准 |
|---|---|
| 主控连接 | SSH 登录成功 |
| 安装环境 | `atom01_py`、`rynnrcp` 和机器人包导入成功 |
| 硬件接口 | `can0` 至 `can3` 为 `UP`，IMU 串口存在 |
| 配置标定 | 23 个关节零位已确认，配置文件已保存确认状态 |
| Server | 终端打印 gRPC 和 Debug UI 地址 |
| 协议 | Observation 持续更新，Health 无关键告警 |
| 策略 | dry-run 输出幅度和连续性符合预期 |

除 SSH 检查外，本文命令默认在 Atom01 主控的以下目录执行：

```bash
cd <RynnRCP仓库>/robots/roboparty_atom01
source venv/bin/activate
```

## 1. 主控连接

参考部署中，电脑连接 Wi-Fi 热点 `atom` 后，主控地址为 `192.168.12.1`：

```bash
ping 192.168.12.1
ssh -v orangepi@192.168.12.1
```

| 现象 | 检查 |
|---|---|
| `Operation timed out` | 电脑是否连接 `atom` 热点、主控是否上电、IP 是否仍为 `192.168.12.1` |
| `No route to host` | 电脑和主控是否处于同一网段 |
| `Connection refused` | 主控 SSH 服务是否启动，22 端口是否放行 |
| `Permission denied` | 主控用户名、系统密码或 SSH 公钥是否正确 |
| 主机指纹变化 | 向设备维护者确认主控是否重装，再更新对应的 `known_hosts` 记录 |

机器人接入其他局域网时，使用主控在该网络中的实际 IP。

## 2. 安装和编译

先确认系统资源和 Python 环境：

```bash
uname -m
cat /etc/os-release
df -h /
python -V
python -c "import atom01_py, rynnrcp, rynnrcp_robot_atom01; print('Imports OK')"
```

参考主控环境为 Ubuntu 22.04、`aarch64`。`Imports OK` 表示 Python 包和 C++ 绑定
可以加载。

| 现象 | 处理 |
|---|---|
| `apt-get` 失败 | 检查主控联网、APT 源和系统时间，再运行 `bash setup_atom01.sh` |
| Python 包下载失败 | 使用 `--pip-index-url URL` 指定主控可访问的镜像 |
| CMake 找不到 Eigen、fmt 或 spdlog | 运行 setup 安装 apt 编译依赖 |
| `atom01_py is not importable` | 激活本目录 `venv`，再重建环境 |
| 编译进程被终止 | 检查 `dmesg`、内存和磁盘空间，关闭占用资源的进程后重试 |

指定镜像：

```bash
bash setup_atom01.sh \
  --pip-index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

重建虚拟环境和 C++ 绑定：

```bash
bash setup_atom01.sh --recreate
source venv/bin/activate
python -c "import atom01_py, rynnrcp, rynnrcp_robot_atom01; print('Imports OK')"
```

## 3. CAN 和 IMU

Atom01 默认接口映射：

| 接口 | 设备 |
|---|---|
| `can0` | 左腿 |
| `can1` | 右腿和腰 |
| `can2` | 左臂 |
| `can3` | 右臂 |
| `/dev/ttyUSB0` | HIPNUC IMU，921600 baud |

检查接口：

```bash
ip -br link show type can
ip -details -statistics link show can0
ip -details -statistics link show can1
ip -details -statistics link show can2
ip -details -statistics link show can3
ls -l /dev/ttyUSB0
ulimit -r
```

预期 `can0` 至 `can3` 均为 `UP`，`/dev/ttyUSB0` 可访问。参考实时优先级上限为
`98`。

| 现象 | 检查 |
|---|---|
| CAN 接口缺失 | USB 转 CAN 连接、供电和 udev 规则 |
| CAN 接口为 `DOWN` | 主控的 CAN 初始化规则和波特率配置 |
| error/drop 持续增长 | 接线、终端电阻、波特率、供电和接口映射 |
| IMU 串口缺失 | USB 连接、udev 规则和实际设备名 |
| `Permission denied` | 当前用户是否具有 CAN 和串口访问权限 |

接口名与默认配置不一致时，更新主控 udev 绑定，或修改
`rynnrcp_robot_atom01/atom_control/config/robot.yaml` 中的 `motor_interface` 和
`imu_interface`，使其与真机一致。

## 4. 配置和按需零位标定

启动配置页面：

```bash
rynnrcp-atom01-configure --no-browser
```

使用终端打印的 `Atom01 configure UI LAN` 地址。连接默认热点时通常为：

```text
http://192.168.12.1:28421
```

端口被占用时，程序会选择下一个空闲端口，以终端输出为准。

配置页面可用于填写机器人信息和查看硬件诊断。零位标定仅在机器人首次组装完成或
机器人站立不稳定时执行；已经完成标定且站立稳定时，保留当前标定结果。

标定前确认：

1. 机器人已可靠支撑，急停可用。
2. RynnRCP Server 和其他电机控制进程已停止。
3. CAN 映射、IMU 串口和 23 个电机 ID 与真机一致。
4. 机器人姿态与页面中的零位参考图一致。

保存标定后检查确认状态：

```bash
grep -nE "confirmed|confirmed_at" \
  rynnrcp_robot_atom01/atom_control/config/robot.yaml
```

预期 `confirmed: true`，并记录 `confirmed_at`。关节读数、零位姿态或接口诊断异常时，
先在配置页修正，再启动 Server。

### 配置页无法访问

在主控检查监听端口：

```bash
ss -lntp | grep 284
```

然后：

- 使用配置程序终端打印的实际端口。
- 确认电脑仍与主控处于同一网络。
- 使用终端打印的 LAN 地址，而不是主控本机的 `127.0.0.1` 地址。
- 经过防火墙访问时，放行实际监听端口。

## 5. Server 启动

启动前再次确认机器人已可靠支撑并准备好急停：

```bash
rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

Server 会初始化电机并移动到默认姿态。首次启动保持前台运行，确认终端打印：

- gRPC 监听地址和端口；
- `Debug UI Local`；
- 一个或多个 `Debug UI LAN` 地址。

`atom01_server.yaml` 使用动态端口，访问时以本次终端输出为准。

### 启动失败

检查现有进程和监听端口：

```bash
ps -ef | grep -E "rynnrcp-server|atom01" | grep -v grep
ss -lntp
```

| 现象 | 处理 |
|---|---|
| `atom01_py is not importable` | 回到“安装和编译”检查虚拟环境和 C++ 绑定 |
| CAN 打开失败或设备不存在 | 回到“CAN 和 IMU”检查接口状态和配置路径 |
| 串口打开失败 | 检查 `/dev/ttyUSB0`、访问权限和 `imu_interface` |
| 标定告警 | 回到配置页检查零位并保存确认状态 |
| 端口被占用 | 使用终端打印的新端口 |
| CAN 被占用 | 正常停止现有电机控制进程，再重新启动 Server |

使用 `Ctrl+C` 停止 Server。等待电机进入安全状态后再断开主控电源。

## 6. Protocol Debug 和 Health

保持 Server 运行，在主控的另一个终端进入相同目录并激活相同环境：

```bash
rynnrcp-protocol-debug \
  --config rynnrcp_robot_atom01/config/atom01_server.yaml
```

先检查：

1. `observation.robot.joint_state` 持续更新，包含 23 个关节位置。
2. `observation.robot.imu` 持续更新。
3. `get_health` 的错误和告警。

| Health 告警 | 含义和动作 |
|---|---|
| `atom01.not_connected` | 控制器未建立；检查 `atom01_py`、CAN、IMU 和 Server 日志 |
| `atom01.last_error` | 查看告警详情，再检查对应底层接口 |
| `atom01.motors_not_enabled` | 检查 CAN 状态、电机供电和急停状态 |
| `atom01.zero_calibration_unconfirmed` | 回到配置页完成零位标定并保存 |

Observation 稳定且关键告警已解决后，再测试 Action。首次 Action 使用保持当前位置或
小幅关节变化，并持续观察 Health、关节状态和急停。

## 7. Policy 包加载

Server 只在启动时扫描：

```text
rynnrcp_robot_atom01/policies/*/policy.yaml
```

平台下发的 ZIP 解压后，目录应为：

```text
rynnrcp_robot_atom01/policies/<policy_id>/policy.yaml
rynnrcp_robot_atom01/policies/<policy_id>/policy.py
```

检查目录和描述文件：

```bash
find rynnrcp_robot_atom01/policies \
  -mindepth 2 -maxdepth 2 -name policy.yaml -print
python - <<'PY'
from pathlib import Path
import yaml

root = Path("rynnrcp_robot_atom01/policies")
for path in sorted(root.glob("*/policy.yaml")):
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    print(path.parent.name, "->", data.get("policy_id") or path.parent.name)
PY
```

| 现象 | 处理 |
|---|---|
| 解压后 `list_policies` 没有新项 | 停止当前 Policy，重启 Server，再刷新列表 |
| 找不到 `policy.yaml` | 调整 ZIP 目录层级，使描述文件位于 `policies/<policy_id>/` |
| `Duplicate policy_id` | 修改或移除重复包，确保每个 `policy_id` 唯一 |
| 加载 `policy.py` 失败 | 检查 `entrypoint`、Python 依赖和包内文件 |
| 模型文件找不到 | 检查 `policy.py` 使用的相对路径和 ZIP 是否包含模型 |
| Observation 或 Action 不存在 | 核对 `policy.yaml` 引用是否与 Atom01 Server 能力一致 |

## 8. Policy 生命周期

### 启动

`start_policy` 会加载 Policy、应用默认或指定的 `runtime_inputs`，随后立即开始循环：

1. 读取 Policy 声明的 Observation。
2. 执行 `Policy.step()`。
3. 将返回的 Action frames 下发到真机。

启动成功即代表真机 Action 已开始下发。先可靠支撑机器人并准备急停。

Policy 单步运行失败时，服务会把错误写入 `last_error`，保持 Policy 为 active，并按
100 ms 间隔继续重试。此时真机保持最后一次成功下发的目标。通过 `list_policies`
查看 `last_error`，解决问题后重新启动 Policy。

### 更新输入

`update_policy_inputs` 只更新当前 Policy 声明过的输入。`stand` 和 `walk` 的
`cmd_vel` 格式为 `[vx, vy, wz]`，500 ms 未更新后自动回到 `[0, 0, 0]`。

### 切换

启动另一个 Policy 时，服务先完成新 Policy 的加载和校验，再停止当前 Policy 并启动
新 Policy。如果新 Policy 加载失败，当前 Policy 继续运行。

成功切换时，旧 Action 停止，新 Policy 第一帧直接接管；服务不执行姿态插值。首次
切换先停止旧 Policy、对齐目标 Policy 起始姿态，再启动目标 Policy。

### 停止

`stop_policy` 会停止 Policy 循环并调用 `stop_action`，阻止继续发送 Action frames。
底层电机继续保持最后一次目标位置；停止 Policy 不会关闭 Server，也不会自动切换为
阻尼模式。

需要结束控制时，停止 Policy 后在 Server 终端按 `Ctrl+C`，让控制器执行关闭流程。

## 9. 策略 dry-run

策略测试程序会连接真机、初始化电机并进入阻尼模式。先让机器人保持可靠支撑，再执行
dry-run：

```bash
rynnrcp-atom01-test-policy \
  --policy stand \
  --steps 50
```

当前测试命令支持包内的 `stand`、`walk` 和 `sim2simdance`。平台下发的自定义
Policy 通过 Protocol Debug 检查和启动。

dry-run 运行推理并打印指标，但不下发策略输出：

- `max|q-home|`：当前姿态与默认姿态的最大偏差。
- `max|a-q|`：策略目标与当前关节的最大差值。
- `max|da|`：相邻策略输出的最大变化。

保存较长的检查记录：

```bash
rynnrcp-atom01-test-policy \
  --policy stand \
  --steps 200 \
  --output /tmp/atom01-stand-dry-run.jsonl
```

确认零位、关节顺序、当前姿态、输出幅度和连续性符合预期后，才向真机下发：

```bash
rynnrcp-atom01-test-policy \
  --policy stand \
  --steps 50 \
  --apply \
  --i-understand-risk
```

程序默认在任一关节目标与当前值相差超过 `0.6 rad` 时终止下发。触发保护时，检查
当前姿态、零位和关节映射，再重新执行 dry-run。

## 10. 收集排障信息

提交问题时附上以下信息，并移除设备凭据：

```bash
uname -a
cat /etc/os-release
python -V
python -c "import atom01_py, rynnrcp; print('Imports OK')"
ip -br link show type can
ls -l /dev/ttyUSB0
git rev-parse --short HEAD
git status --short
```

同时附上：

- `setup_atom01.sh` 或 Server 的完整报错；
- `get_health` 返回；
- 问题发生前执行的命令；
- 机器人是否已标定，以及当时处于支撑、阻尼还是动作状态。

凭据、设备密钥和访问令牌保留在主控本地。
