# SO100接入乐云平台最小接入示例
本目录提供 SO100 机械臂接入 RynnRCP 的最小可运行工程，包括：

- 配置文件：
  - config/so100_config.yaml：RcpCore 配置（Action/Sensor/DeviceMonitor 的 inputs/outputs）
  - config/rynnbot_config.yaml：乐云平台设备信息（三元组/URL）
- 端侧启动示例：
  - run_rcp_so100.py：启动 RcpCore + RynnBot(plugin)，用于连接乐云平台/上层控制系统
- 配置向导脚本：
  - configure_so100.py：交互式配置（支持中文/英文），可自动完成相机、串口、标定及平台三元组填写以及机械臂标定

## 目录结构

```text
robots/so100/
├── config/
│   ├── rynnbot_config.yaml     # 乐云设备三元组/URL
│   └── so100_config.yaml       # RcpCore 配置：Action/Sensor/DeviceMonitor
├── configure_so100.py          # 交互式配置脚本（中/英文）
├── run_rcp_so100.py            # 启动示例：RcpCore + RynnBot(plugin)
├── README.md                   # 使用文档（英文）
└── README_cn.md                # 使用文档（中文）
```


## 前置条件

1. 已完成项目安装（Python 环境就绪，依赖已安装，Python环境已激活）
2. 已连接 SO100 机械臂控制器（串口方式）
3. 已连接相机（USB 摄像头，建议两路：front/wrist）
4. 具备乐云平台设备信息三元组：
   - product_key
   - device_name
   - device_secret
   - http_url（平台接入 URL，维持默认值即可）

## 快速开始
进入目录并运行配置脚本：
```bash
cd RynnRCP/robots/so100
python configure_so100.py
```
脚本启动后会先让你选择语言（中文/英文），然后进入菜单：
1. 设备设置：填写乐云平台信息，保存到 config/rynnbot_config.yaml
2. 相机设置：通过插拔检测相机，写入 config/so100_config.yaml
3. 机器人串口设置：通过插拔检测串口，并修改串口权限，同时写入 rcp_motion 的底层配置文件（RynnRCP/rcp_motion/robots/so100/configs/config.yaml）
4. 机械臂标定：执行机械臂标定程序，写入标定配置文件（RynnRCP/rcp_motion/robots/so100/.cache/calibration/so100/main_follower.json）
5. 一键配置全部：按 1→2→3→4 顺序执行

> 标定位姿参考：[SO100 标定位姿](../../docs/images/so100_calibration_diagram.png)

## 启动端侧服务

完成配置后，在本目录启动：

```bash
python run_rcp_so100.py
```
该脚本会读取：
- ./config/so100_config.yaml
- ./config/rynnbot_config.yaml

并启动端侧服务节点，连接乐云平台/上层控制。
