# RCP 配置文件说明

新增机器人通常需要两份配置：

- `server config`：某一台机器人实例的真实参数，例如串口、IP、相机编号、启用开关、能力开关。
- `robot integration`：某一类机器人如何接入 RCP，例如 Manifest、Component、Observation、Action、controller 方法。

App 使用 `rynnrcp_app_config`。App 不是 RCP Server，不写 `manifest.robot_id`；本地日志、缓存、录制等运行文件用 `app.app_id` 分目录保存。

配置原则：

- `server config` 写真实值，不写占位符。
- `robot integration` 可以用 `${...}` 引用 `server config`。
- `components` 是 server config 里的真实部署参数组。推荐与 integration 里的 `components[].name` 对齐；
  如果某个机器人包为了硬件语义使用更明确的参数组名，必须在 integration 里通过 `${components.xxx...}` 明确引用。
- 用户不写完整协议对象名；框架按 `observation.<component.name>.<observation.name>` 和 `action.<component.name>.<action.name>` 生成。
- 用户通常不写 `value_schema` / `input_schema`；框架按 `type` 生成。`custom` action 也是用户自定义动作，建议写 `input_schema` 明确入参；如果不写，框架按开放 object 处理。
- `role`、`port`、`device_id`、`ip` 等不是 RCP 协议字段；它们只是某个 controller 或硬件 source 的初始化参数。

## SO101 Server 示例

```yaml
config_type: rynnrcp_server_config
version: 1

manifest:
  robot_id: so101_follower
  robot_name: SO101 Follower
  capabilities:
    observations: true
    actions: true
    health: true
    resources: true
    data_collection: true
    policy_service: false

integration:
  config: package://rynnrcp_robot_so101/config/robot_integration.yaml

components:
  robot:
    enabled: true
    role: follower
    port: /dev/ttyACM0
  front_camera:
    enabled: true
    device_id: 0
  wrist_camera:
    enabled: true
    device_id: 1
```

`manifest.capabilities` 写真实 bool。能力开关必须包含 `observations`、`actions`、`health`、`resources`、`data_collection`、`policy_service` 六项；不暴露本地推理策略时写 `policy_service: false`。控制端、只读机器人或只提供状态的设备可以关闭 action 和 data collection：

```yaml
manifest:
  capabilities:
    observations: true
    actions: false
    health: true
    resources: true
    data_collection: false
    policy_service: false
```

这里的 `role: follower` 和 `port: /dev/...` 是 SO101Controller 需要的参数，不是所有机器人都要写。换成别的机器人时，应写该机器人 controller 真正需要的参数，例如 `ip`、`can_bus`、`serial_number` 或 `model_path`。

## SO101 Integration 示例

```yaml
config_type: rynnrcp_robot_integration
version: 1

manifest:
  robot_id: ${manifest.robot_id}
  robot_name: ${manifest.robot_name}
  embodiment_type: single_arm
  capabilities:
    observations: ${manifest.capabilities.observations}
    actions: ${manifest.capabilities.actions}
    health: ${manifest.capabilities.health}
    resources: ${manifest.capabilities.resources}
    data_collection: ${manifest.capabilities.data_collection}
    policy_service: ${manifest.capabilities.policy_service}
  model_refs:
    urdf: null
    mjcf: null
    calibration: null
    assets: []
  metadata: {}

components:
  - name: robot
    type: arm
    parent_component: null
    dof: 6
    frame: base
    description: SO101 arm hardware
    enabled: ${components.robot.enabled}
    health:
      frame_rate: 1
      source:
        connector: module
        module_name: rynnrcp_robot_so101.controller.SO101Controller
        init:
          robot_id: ${manifest.robot_id}
          port: ${components.robot.port}
          role: ${components.robot.role}
        start: start
        stop: shutdown
        method_name: get_health
    observations:
      - name: joint_state
        type: joint_state
        description: Current SO101 arm joint positions
        frame_rate: 60
        source:
          connector: module
          module_name: rynnrcp_robot_so101.controller.SO101Controller
          init:
            robot_id: ${manifest.robot_id}
            port: ${components.robot.port}
            role: ${components.robot.role}
          start: start
          stop: shutdown
          method_name: get_joint_positions
        codec:
          adapter: ProtocolInputAdapter
    actions:
      - name: joint_position
        type: joint_position
        description: Target SO101 arm joint positions
        frame_rate: 60
        source:
          connector: module
          module_name: rynnrcp_robot_so101.controller.SO101Controller
          init:
            robot_id: ${manifest.robot_id}
            port: ${components.robot.port}
            role: ${components.robot.role}
          start: start
          stop: shutdown
          method_name: set_joint_positions
        codec:
          adapter: ProtocolActionOutputAdapter

      - name: home
        type: prearranged
        description: Move SO101 arm to the configured home pose
        frame_rate: 1
        source:
          connector: module
          module_name: rynnrcp_robot_so101.controller.SO101Controller
          init:
            robot_id: ${manifest.robot_id}
            port: ${components.robot.port}
            role: ${components.robot.role}
          start: start
          stop: shutdown
          method_name: go_home
        codec:
          adapter: ProtocolActionOutputAdapter

      - name: move_to
        type: custom
        description: Interpolate SO101 arm to a target joint position
        frame_rate: 1
        source:
          connector: module
          module_name: rynnrcp_robot_so101.controller.SO101Controller
          init:
            robot_id: ${manifest.robot_id}
            port: ${components.robot.port}
            role: ${components.robot.role}
          start: start
          stop: shutdown
          method_name: move_to
        codec:
          adapter: ProtocolActionOutputAdapter

  - name: front
    type: camera
    parent_component: robot
    frame: front_camera
    description: Front camera hardware
    enabled: ${components.front_camera.enabled}
    observations:
      - name: image
        type: image
        description: Latest image captured by the front camera
        frame_rate: 30
        source:
          connector: port
          port_type: cameras.usb_camera.USBCamera
          init:
            name: front_camera
            device_id: ${components.front_camera.device_id}
            width: 640
            height: 360
            encoding: jpg
            fps: 30
            rotate: 0
            native_compressed: true
            capture_backend: auto
          method_name: read
        codec:
          adapter: ProtocolImageInputAdapter

  - name: wrist
    type: camera
    parent_component: robot
    frame: wrist_camera
    description: Wrist camera hardware
    enabled: ${components.wrist_camera.enabled}
    observations:
      - name: image
        type: image
        description: Latest image captured by the wrist camera
        frame_rate: 30
        source:
          connector: port
          port_type: cameras.usb_camera.USBCamera
          init:
            name: wrist_camera
            device_id: ${components.wrist_camera.device_id}
            width: 640
            height: 360
            encoding: jpg
            fps: 30
            rotate: 0
            native_compressed: true
            capture_backend: auto
          method_name: read
        codec:
          adapter: ProtocolImageInputAdapter
```

`components[].health` 是可选项，不是每个 controller 都必须实现 `get_health`。
只有当配置文件里写了 `health.source.method_name: get_health` 时，对应 controller 才需要提供这个方法。
如果机器人没有本地健康检查，直接不写 `health` 即可。实现时该方法返回：

```python
{
    "errors": [],
    "warnings": [
        {
            "code": "camera.read_failed",
            "message": "failed to read frame",
            "source": "front_camera",
            "timestamp": 1745472000.888,
            "details": {"device_id": 1}
        }
    ]
}
```

如果不配置 `health`，不会调用 controller 的 `get_health`。框架仍可以根据 Observation 的 timestamp
上报 `observation.no_data` 或 `observation.stale`，这属于 RCP Runtime 的数据新鲜度检查，不要求用户实现
controller 方法。

## 字段说明

### app config

```yaml
config_type: rynnrcp_app_config
version: 1

app:
  name: teleop
  app_id: teleop_app
```

| 字段 | 必写 | 说明 |
| --- | --- | --- |
| `config_type` | 是 | 固定写 `rynnrcp_app_config` |
| `version` | 是 | 当前写 `1` |
| `app.name` | 是 | App 类型或显示名，例如 `teleop` / `mcp` / `rynnbot` |
| `app.app_id` | 是 | 当前 App 实例 ID，用于本地运行文件目录 |

App 本地文件默认保存到：

```text
~/.rynnrcp/apps/<app.app_id>/
```

`manifest.robot_id` 只属于 server config 和 robot integration，不属于 app config。

### server config

| 字段 | 必写 | 说明 |
| --- | --- | --- |
| `config_type` | 是 | 固定写 `rynnrcp_server_config` |
| `version` | 是 | 当前写 `1` |
| `manifest.robot_id` | 是 | 当前机器人实例 ID |
| `manifest.robot_name` | 是 | 当前机器人显示名 |
| `manifest.capabilities` | 是 | 当前实例对外暴露的能力。六项必须写：`observations` / `actions` / `health` / `resources` / `data_collection` / `policy_service` |
| `integration.config` | 是 | robot integration 配置路径 |
| `components` | 是 | 当前实例的 component 参数 |
| `server.storage.collection_dir` | 否 | 数据采集保存目录 |
| `connectors` | 否 | connector 参数 |
| `runtime` | 否 | runtime 参数 |
| `storage.collection_dir` | 否 | 数据采集保存目录 |
| `policies.paths` | 否 | 本地 policy 搜索目录列表；只有 `manifest.capabilities.policy_service: true` 时才需要 |

同时写了 `server.storage.collection_dir` 和 `storage.collection_dir` 时，优先使用
`server.storage.collection_dir`。普通配置推荐只写顶层 `storage.collection_dir`。

`components.<name>.enabled` 决定某个 component 是否启动。被禁用的 component 不会解析它的串口、相机编号等参数。

`components.<name>` 里的其他字段由该 component 的 `source.init` 决定。例如 SO101 的 `role`、`port` 是 `SO101Controller` 的初始化参数；USB 相机的 `device_id` 是 `USBCamera` 的初始化参数。新增机器人时不要照抄 SO101 字段，而是写自己的 controller 或硬件 source 真正需要的参数。

开启策略服务时，Server 启动后会注册 `list_policies`、`start_policy`、`update_policy_inputs`、`stop_policy`。`policies.paths` 中的相对路径按 server config 所在目录解析；每个路径下按 `*/policy.yaml` 扫描策略：

```yaml
manifest:
  capabilities:
    observations: true
    actions: true
    health: true
    resources: true
    data_collection: true
    policy_service: true

policies:
  paths:
    - ../policies
```

单个 policy 目录至少包含 `policy.yaml` 和入口 Python 文件。字段细节见
[`RCP 策略服务接入指南.html`](RCP%20策略服务接入指南.html)；协议方法见
[`RCP使用场景及协议.md`](RCP使用场景及协议.md)。

### robot integration

| 字段 | 必写 | 说明 |
| --- | --- | --- |
| `config_type` | 是 | 固定写 `rynnrcp_robot_integration` |
| `version` | 是 | 当前写 `1` |
| `manifest.robot_id` | 是 | 通常引用 `${manifest.robot_id}` |
| `manifest.robot_name` | 是 | 通常引用 `${manifest.robot_name}` |
| `manifest.embodiment_type` | 是 | 本体类型，例如 `single_arm` |
| `manifest.capabilities` | 是 | 通常引用 server config 的能力开关，包含六项：`observations` / `actions` / `health` / `resources` / `data_collection` / `policy_service` |
| `manifest.model_refs` | 否 | URDF、MJCF、标定等模型资源引用；可为空 |
| `manifest.metadata` | 否 | 用户自定义补充信息 |
| `components` | 是 | Component、Observation、Action 接入定义 |

`manifest.model_refs` 在配置文件里写 server 本机可访问的路径。当
`manifest.capabilities.resources: true` 时，RynnRCP 启动时会把这些文件注册成 Resource，并在
`get_manifest` 里返回 `resource_id`、`format`、`hash`、`size_bytes` 等协议字段。App、Agent、
云端不能使用 server 本地路径读取文件，应通过 Resource 接口读取。

当 `manifest.capabilities.resources: false` 时，表示这个 Server 不对外暴露文件类资源；
路径型 `model_refs` 只作为本地配置存在，不会进入 Manifest，也不能被 App 读取。

相对路径按该机器人本地存储根目录解析；如果模型文件放在别处，建议写绝对路径。为空时写 `null`：

```yaml
model_refs:
  urdf: /opt/robots/so101/models/so101.urdf
  mjcf: null
  calibration: /opt/robots/so101/calibration/follower.yaml
  assets:
    - /opt/robots/so101/models/meshes/base.stl
```

### component

`components` 是 robot integration 里最重要的结构。每一项代表机器人对外暴露的一个部件，
例如机械臂、夹爪、相机、底盘、IMU。最终协议对象名里的第二段来自 `component.name`。

```text
observation.<component.name>.<observation.name>
action.<component.name>.<action.name>
```

| 字段 | 必写 | 怎么写 | 说明 |
| --- | --- | --- | --- |
| `name` | 是 | 短名，小写 snake_case，当前 integration 内唯一，例如 `robot` / `front` / `wrist` / `gripper_0` | 协议对象名的一部分。不要写完整协议名。 |
| `type` | 是 | 按部件类型写，例如 `arm` / `camera` / `gripper` / `mobile_base` / `imu` / `sensor` | 描述这是哪类硬件部件，供 Manifest、App UI 和 Agent 理解。 |
| `parent_component` | 否 | 没有父级写 `null`；挂在机械臂上写 `robot` | 用来表达组件层级，例如相机或夹爪挂在机械臂上。 |
| `dof` | 否 | 数字，例如 6；没有自由度的部件不写 | 机械臂、夹爪、底盘等需要表达自由度时写。相机通常不写。 |
| `frame` | 否 | 字符串，例如 `base` / `front_camera` / `wrist_camera` | 该部件关联的坐标系名称。坐标系本身由机器人构型定义，不在 Manifest 顶层单独列 frames。 |
| `description` | 否 | 一句人类可读描述，例如 `SO101 arm hardware` | 给 UI、文档和调试使用。建议写，但不参与协议逻辑。 |
| `enabled` | 是 | bool 或 `${components.xxx.enabled}` | 控制该部件是否启动。被禁用时，该 component 下的 observation、action、health 都不会启动。 |
| `health` | 否 | 一个 health 配置块 | 该部件有本地健康检查方法时写；没有就不写。 |
| `observations` | 否 | 列表 | 该部件提供哪些观测数据。没有观测就不写。 |
| `actions` | 否 | 列表 | 该部件接收哪些控制命令。没有动作就不写。 |

推荐写法：

```yaml
components:
  - name: robot
    type: arm
    parent_component: null
    dof: 6
    frame: base
    description: Arm hardware
    enabled: ${components.robot.enabled}
```

相机示例：

```yaml
components:
  - name: front
    type: camera
    parent_component: robot
    frame: front_camera
    description: Front camera
    enabled: ${components.front_camera.enabled}
```

这里 `front` 是协议 component 名，会生成 `observation.front.image`；
`components.front_camera.device_id` 是 server config 里的部署参数组名，由 integration 显式引用。
如果没有特殊原因，推荐让 server config 参数组名和 component `name` 保持一致。

不要这样写：

```yaml
name: observation.front.image   # 错：component.name 不是完整协议对象名
observations: []                # 可省略
actions: []                     # 可省略
```

### observation / action

Observation 和 Action 都写在某个 component 下面。

| 字段 | 必写 | 怎么写 | 说明 |
| --- | --- | --- | --- |
| `name` | 是 | 局部对象名，小写 snake_case，例如 `joint_state` / `image` / `joint_position` / `home` / `move_to` | 和 component 名拼成完整协议对象名。不要写 `observation.robot.joint_state`。 |
| `type` | 是 | 协议 value 类型，例如 `joint_state` / `image` / `ee_pose` / `joint_position` / `prearranged` / `custom` | 决定 value_schema 或 input_schema。 |
| `description` | 否 | 一句说明，例如 `Current arm joint positions` | 建议写清楚该数据或命令是什么。 |
| `frame_rate` | Observation 必写；Action 建议写 | 数字，单位 Hz，例如 60 / 30 / 1 | Observation 中用于 runner 轮询频率；Action 中表示期望执行或数据语义频率。 |
| `source` | 是 | source 配置块 | Observation 表示从哪里读取；Action 表示写到哪里或调用哪个方法。 |
| `codec` | 是 | codec 配置块 | 选择协议适配器。 |
| `value_schema` | 否 | object | Observation value schema。通常不写，由 `type` 生成。 |
| `input_schema` | 否 | object | Action input schema。普通 Action 通常不写，由 `type` 生成；`custom` 建议写清楚，不写则表示开放 object。 |

`name` 和 `type` 不是同一件事。`name` 生成协议对象名，`type` 决定 value schema。例如：

```yaml
name: state
type: gripper_state
```

会生成：

```text
observation.gripper_0.state
```

常见写法：

```yaml
observations:
  - name: joint_state
    type: joint_state
    description: Current arm joint positions
    frame_rate: 60
    source: ...
    codec: ...

actions:
  - name: joint_position
    type: joint_position
    description: Target arm joint positions
    frame_rate: 60
    source: ...
    codec: ...

  - name: home
    type: prearranged
    description: Move robot to home pose
    frame_rate: 1
    source: ...
    codec: ...

  - name: move_to
    type: custom
    description: Interpolate to target joint positions
    frame_rate: 1
    source: ...
    codec: ...
```

`prearranged` 和 `custom` 都是构型侧可以定义的动作。区别是：

- `prearranged` 表示机器人侧预定义动作序列，调用时 value 必须是 `{}`。
- `custom` 表示机器人侧自定义且需要结构化入参的动作，调用时 value 必须是 object，并会原样传给 controller 方法。建议在 action 上写 `input_schema`，这样 `list_actions` 能告诉 App 怎么填参数。

例如 SO101 的 `action.robot.move_to` 是一个 custom action，value 可以是：

```json
{"joint_positions": [0.0, -1.2, 1.4, 0.0, 0.0, 0.0]}
```

### source / codec

`source` 描述数据从哪里读、动作写到哪里；`codec` 描述真实数据和 RCP 协议 value 如何转换。
`source.init` 里的字段会传给 controller、端口对象或 connector，它们来自 server config 的真实部署参数，
不是 RCP 对外协议字段。

#### source 通用字段

| 字段 | 必写 | 怎么写 | 说明 |
| --- | --- | --- | --- |
| `connector` | 是 | `module` / `port` / `ros2` / `lcm` | 选择连接方式。 |
| `method_name` | module/port 必写 | 方法名字符串 | Observation 调用该方法读取数据；Action 调用该方法执行命令。 |
| `init` | 视情况 | object | 初始化参数。字段名必须匹配 controller、端口对象或 connector 的构造参数。 |
| `start` | module 建议写 | 方法名字符串 | Runtime 启动时调用，例如 `start`。没有启动方法可不写。 |
| `stop` | module 建议写 | 方法名字符串 | Runtime 停止时调用，例如 `shutdown`。没有停止方法可不写。 |

#### codec 通用字段

| 字段 | 必写 | 怎么写 | 说明 |
| --- | --- | --- | --- |
| `adapter` | 是 | adapter 类名字符串 | 决定真实数据和协议 value 的转换方式。 |

常用 adapter：

| 场景 | adapter |
| --- | --- |
| Python controller 返回普通协议 value | `ProtocolInputAdapter` |
| Python controller / LCM / JSON action | `ProtocolActionOutputAdapter` |
| 图像 observation | `ProtocolImageInputAdapter` |
| ROS2 标准 observation | `Ros2StandardInputAdapter` |
| ROS2 标准 action | `Ros2StandardActionOutputAdapter` |

`module` 适合 Python controller：

```yaml
source:
  connector: module
  module_name: my_robot.controller.MyController
  init:
    robot_id: ${manifest.robot_id}
  start: start
  stop: shutdown
  method_name: get_joint_positions
codec:
  adapter: ProtocolInputAdapter
```

Observation 的 `method_name` 不接收参数，返回该 observation 的协议 value。Action 的 `method_name` 接收该 action 的协议 value。例如：

```python
class MyController:
    def get_joint_positions(self):
        return {"joint_positions": [0.0, -1.2, 1.4, 0.0, 0.0, 0.0]}

    def set_joint_positions(self, value):
        positions = value["joint_positions"]
        ...

    def move_to(self, value):
        positions = value["joint_positions"]
        ...
```

如果 action 是 `type: custom`，框架不会限制 action name，也不会按名字做特殊编码。component 是 `robot`、action name 是 `move_to` 时，会生成 `action.robot.move_to`；调用时 `frames[]` 会作为一个 object 原样传给 `source.method_name` 对应的方法，例如：

```json
{"joint_positions": [0.0, -1.2, 1.4, 0.0, 0.0, 0.0]}
```

`port` 适合本地设备端口：

```yaml
source:
  connector: port
  port_type: cameras.usb_camera.USBCamera
  init:
    name: wrist_camera
    device_id: ${components.wrist_camera.device_id}
  method_name: read
codec:
  adapter: ProtocolImageInputAdapter
```

`ros2` / `lcm` 默认使用协议 JSON：

```yaml
source:
  connector: ros2
  topic: /robot/state
  msg_type: std_msgs.msg.String
  payload_mode: protocol_json
  qos:
    depth: 1
codec:
  adapter: ProtocolInputAdapter
```

LCM 只作为 pub/sub 通信机制使用，不维护 RCP 专用 `.lcm` 消息类型；消息体统一为
UTF-8 JSON，结构按协议对象的 `name` / `type` / `value` 解析。

ROS2 也可以桥接少量标准消息类型。该模式只支持白名单类型，不支持任意字段映射。
例如 `sensor_msgs.msg.JointState` 转成 RCP `joint_state`：

```yaml
source:
  connector: ros2
  topic: /joint_states
  msg_type: sensor_msgs.msg.JointState
  payload_mode: ros2_standard
  qos:
    depth: 1
codec:
  adapter: Ros2StandardInputAdapter
```

当前 ROS2 标准消息白名单：

| ROS2 message | RCP type | adapter |
| --- | --- | --- |
| `sensor_msgs.msg.JointState` | `joint_state` observation | `Ros2StandardInputAdapter` |
| `sensor_msgs.msg.Image` | `image` observation | `ProtocolImageInputAdapter` |
| `sensor_msgs.msg.CompressedImage` | `image` observation | `ProtocolImageInputAdapter` |
| `geometry_msgs.msg.PoseStamped` | `ee_pose` observation/action | `Ros2StandardInputAdapter` / `Ros2StandardActionOutputAdapter` |
| `geometry_msgs.msg.Twist` | `base_velocity` action | `Ros2StandardActionOutputAdapter` |
| `sensor_msgs.msg.JointState` | `joint_position` / `joint_velocity` action | `Ros2StandardActionOutputAdapter` |

`sensor_msgs.msg.CompressedImage` 不包含宽高，配置中必须补充 `width`、`height` 和
`encoding`。

## 生成规则

```text
observation.<component.name>.<observation.name>
action.<component.name>.<action.name>
```

SO101 follower 会生成：

```text
observation.robot.joint_state
observation.front.image
observation.wrist.image
action.robot.joint_position
action.robot.home
action.robot.move_to
```

Aero Hand 单手/双手构型会生成：

```text
observation.robot.joint_state
action.robot.joint_position
```

单手 `joint_positions` 为 7 维，双手为 14 维，顺序见 `robots/aero_hand/README.zh-CN.md`。

## 检查配置

不启动硬件，只检查配置展开：

```bash
PYTHONPATH=robots/so101:robots/aero_hand:robots/atom01 python - <<'PY'
from rynnrcp.config import RuntimeConfig, build_runner_config

for path in [
    "robots/so101/rynnrcp_robot_so101/config/so101_follower_server.yaml",
    "robots/so101/rynnrcp_robot_so101/config/so101_leader_server.yaml",
    "robots/aero_hand/rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml",
    "robots/aero_hand/rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml",
    "robots/atom01/rynnrcp_robot_atom01/config/atom01_server.yaml",
]:
    runtime = RuntimeConfig.load(path)
    runner = build_runner_config(runtime)
    print(path)
    print("observations:", [item.name for item in runner.input_specs])
    print("actions:", [item.name for item in runner.output_specs])
PY
```
