# RCP使用场景及协议

# RCP使用场景及协议

# RCP一句话定义

统一机器人技能，本体，传感器，以及数据等接入并对其进行管理，提供给 Physical Agent/外部系统/应用 进行数据采集，推理和调度等。大部分接入并不会随着agent能力增强而变化，除了部分技能，比如vla，可能被将来Agent内的模型所接管，可参考LLM和Agent的相互演进过程。RCP的实现可部署在real或者sim本体侧，提供运行时环境。

# 协议定位

RCP 是面向机器人本体、传感器、动作、技能和执行数据的标准接入与运行时管理协议。

RCP 的核心目标不是把机器人简单封装成若干工具，而是提供一套稳定的机器人能力接入边界，让上层 Agent、业务应用、数据平台和技能系统可以通过一致的方式理解、调用、记录和复用机器人能力。

RCP 主要解决四类问题：

```plaintext
机器人能力如何标准接入
机器人执行过程如何被管理和记录
执行数据如何被回放、导出和复用
技能如何被注册、发现、评估和再次调用
```

# 场景

与Agent / System的交互

1.  VLA/policy 读取camera，机械臂状态驱动机械臂执行某个具体task（类似现在的乐云平台）

2.  whole body control 驱动 全身关节运动

3.  Agent 读取单个机器人camera，机器人状态，驱动机器人进行行走，抓取，跳舞，讲解等

4.  Agent 读取单个机器人camera，机器人状态，技能信息 驱动机器人进行行走，抓取，跳舞，讲解等

5.  Agent 读取多个机器人相关信息 进行 机器人协同控制

6.  以上均有可能同时进行数据采集

7.  在线RL：在进行推理任务时，可被打断，并被遥操接管

8.  同构/异构遥操+数据采集

9.  以上通信可能在本机，局域网或者广域网

10.  演示沉淀为技能，并被调用


# 架构

```text
Agent / App / Cloud / MCP
        |
        v
RCP Interface
        |
        v
RCP Server
        |
        v
Robot SDK / ROS2 / LCM / Camera / Controller / Policy
```

## Agent / System <-> RCP

### 通信协议

①MCP ②自定义JSON-RPC ③MQTT

#### 面向Agent / System接口

标准接口

##### 本体与能力清单（Manifest）

Manifest 是机器人对外暴露的本体与能力清单，用于描述机器人身份、组件、观测对象、动作对象、模型资源和能力声明。Manifest 提供概览级发现信息；观测和动作的完整参数由 `list_observations` / `list_actions` 提供。

Manifest 可用工具：

| 工具名 | 意义 | 回调 | 参数 | 返回 |
| --- | --- | --- | --- | --- |
| get\_manifest | 获取机器人本体与能力清单 | × | `{}` | Manifest 对象 |

Manifest对象 包含以下字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| robot\_id | string | 必选 | 机器人唯一标识 |
| robot\_name | string | 必选 | 机器人可读名称 |
| embodiment\_type | string | 必选 | 机器人本体类型，如 `single_arm` / `dual_arm` / `humanoid` / `mobile_manipulator` / `quadruped` |
| components | array | 必选；可为空数组 | 机器人对外暴露的主要组件 |
| observations | array | 必选；可为空数组 | 机器人暴露的观测对象概要 |
| actions | array | 必选；可为空数组 | 机器人暴露的动作对象概要 |
| capabilities | object | 可选 | 当前 RCP 实例支持的协议能力 |
| model\_refs | object | 可选 | 模型、资产与标定资源描述 |
| metadata | object | 可选 | 用户自定义补充信息 |

`components` 描述机器人对外暴露的主要组件，不要求与 URDF / MJCF 中的 link、joint、body 一一对应。若提供组件列表，可通过 `parent_component` 描述父子关系。

Manifest 按三个层次描述机器人能力：

*   `components` 表示机器人有哪些部件，例如机械臂、相机、夹爪、底盘。

*   `observations` 表示这些部件对外提供哪些可读取数据，例如关节状态、相机图像、IMU、末端位姿。

*   `actions` 表示这些部件对外接受哪些控制命令，例如关节位置、末端位姿、夹爪开合、底盘速度。


Observation / Action 通过自身的 `component_name` 字段归属到某个 Component。协议对外仍保留 `components`、`observations`、`actions` 三个列表，便于 Agent 快速分别发现“有哪些部件”“能读什么”“能控制什么”。

每个 Component 对象包含以下字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 组件名称，在当前 Manifest 的 `components` 内唯一 |
| `type` | string | 必选 | 组件类型，如 `base` / `mobile_base` / `arm` / `gripper` / `camera` / `imu` / `head` / `torso` / `lidar` |
| `parent_component` | string/null | 可选 | 父组件名称；根组件可为 `null` |
| `dof` | int | 可选 | 组件自由度 |
| `frame` | string | 可选 | 组件主要关联坐标系名称 |
| `description` | string | 可选 | 人类可读描述 |

`observations` 描述观测对象概要，只包含名称、类型、组件等信息。完整 `value_schema`、帧率、值结构等由 `list_observations` 提供。

Observation / Action 的 `name` 使用固定三段点分层格式：`<category>.<component_or_scope>.<object>`，如 `observation.robot.joint_state`、`action.robot.joint_position`。Observation 的观测值类型、Action 的动作输入类型由 `type` 字段描述。

每个 Observation 概要对象包含以下字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 观测对象名称，在当前 Manifest 的 `observations` 内唯一 |
| `type` | string | 必选 | 观测值类型，如 `image` / `robot_state` / `joint_state` / `ee_pose` / `gripper_state` / `imu` / `force_torque` / `tactile` / `point_cloud` |
| `component_name` | string | 可选 | 所属组件名称 |
| `description` | string | 可选 | 人类可读描述 |

actions 描述动作对象概要。完整 input\_schema、限位、帧率、超时、停止策略等由 list\_actions 提供。prearranged 表示机器人侧预定义并可直接调用的固定动作序列，frames\[ \] 必须为空对象 {}；custom 表示机器人侧自定义且需要结构化入参的动作，动作语义由 Action 的 name 决定，具体入参结构由该 Action 自己的 input\_schema 声明。

每个 Action 概要对象包含以下字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 动作对象名称，在当前 Manifest 的 `actions` 内唯一 |
| `type` | string | 必选 | 动作对象类型，如 `joint_position` / `joint_velocity` / `ee_pose` / `gripper` / `base_velocity` / `head_pose` / `prearranged` / `custom` |
| `component_name` | string | 可选 | 所属组件名称 |
| `description` | string | 可选 | 人类可读描述 |

`capabilities` 用于声明当前 RCP 实例支持的协议能力。字段值为 `true` 表示支持，`false` 表示不支持；字段缺失表示未声明，Agent 不应假定支持。

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `observations` | boolean | 必选 | 是否支持观测对象查询和读取 |
| `actions` | boolean | 必选 | 是否支持动作对象查询和执行 |
| `health` | boolean | 必选 | 是否支持本体健康状态查询 |
| `resources` | boolean | 必选 | 是否支持资源描述、枚举和读取；为 `false` 时表示当前实例不对外暴露文件类资源 |
| `data_collection` | boolean | 必选 | 是否支持数据采集 |
| `policy_service` | boolean | 必选 | 是否支持推理策略服务 |

`model_refs` 用于声明机器人模型、资产与标定资源，供仿真、可视化、调试或系统集成使用。协议不要求也不建议暴露服务端本地文件路径。URDF / MJCF / calibration 等较小文本资源可直接以内联 `content` 返回；mesh、texture、较大模型资产或资源包通过 Resource 对象描述，并由资源读取接口按需获取。

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `urdf` | object | 可选 | URDF 模型资源；可包含 `format`, `content`, `hash`，或使用 `resource_id` 引用 |
| `mjcf` | object | 可选 | MJCF 模型资源；可包含 `format`, `content`, `hash`，或使用 `resource_id` 引用 |
| `calibration` | object | 可选 | 标定资源；可内联文本内容或使用 `resource_id` 引用 |
| `assets` | array | 可选 | mesh、texture 等模型资产资源列表，元素为 Resource 对象 |

模型资源对象常见字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `format` | string | 可选 | 资源格式，如 `urdf` / `mjcf` / `yaml` / `stl` / `obj` / `png` |
| `content` | string | 可选 | 内联文本内容，适用于较小的文本模型或标定文件 |
| `resource_id` | string | 可选 | 资源标识，用于通过 Resource 接口读取资源内容 |
| `hash` | string | 可选 | 资源内容哈希，如 `sha256:...` |
| `size_bytes` | int | 可选 | 资源大小，单位字节 |

对于 `urdf` / `mjcf` / `calibration` 等模型资源对象，`content` 和 `resource_id` 至少应提供一个。`assets` 中的元素为 Resource 对象，通过 `resource_id` 读取。调用方不应假定 `resource_id` 可被解析为本地路径。

`metadata` 为用户自定义补充信息，协议不限制字段结构；未提供时不影响协议功能。

Agent 的典型使用流程：先查 Manifest 了解全貌 → 再查 list 接口获取具体对象的详细参数 → 进行状态读取、动作执行或策略启动。

##### 状态（Observation）

Observation 是 RCP 对外暴露的只读观测对象，用于读取机器人状态、传感器数据和空间位姿。Manifest 提供 Observation 概要；完整描述由 `list_observations` 提供，当前值由 `get_observations` 或 `subscribe_observations` 获取。

Observation 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| list\_observations | 查询所有 Observation 的完整描述 | × | para：无<br>ret：{observations: \[{name, type, description?, frame\_rate?, value\_schema}\]} |
| get\_observations | 获取指定 Observation 的当前值（一次性拉取） | × | para：names（名称列表）, sync?（是否尽量时间对齐，默认 false）<br>ret：{observations: \[{name, timestamp, value}\]} |
| subscribe\_observations | 订阅 Observation 持续推送（流式） | ✓ | para：names（订阅列表）, frame\_rate（推送帧率，单位 Hz）<br>ret：{subscription\_id}<br>callback：{type, subscription\_id, data: {name, timestamp, value}} |

`list_observations` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `observations` | array | 必选 | Observation 完整描述列表 |
| `observations[ ].name` | string | 必选 | 观测对象唯一标识，如 `observation.robot.joint_state` |
| `observations[ ].type` | string | 必选 | 观测值类型，如 `image` / `joint_state` / `ee_pose` / `gripper_state` / `imu` / `force_torque` / `tactile` / `point_cloud` |
| `observations[ ].description` | string | 可选 | 人类可读描述 |
| `observations[ ].frame_rate` | float | 可选 | 默认或当前输出帧率，单位 Hz |
| `observations[ ].value_schema` | object | 必选 | `value` 字段的轻量结构说明，用于描述返回值的类型、字段、维度、单位或编码 |

`value_schema` 用于描述 `value` 的数据结构，包括类型、字段、维度、单位和编码。常用基础类型包括 `float` / `int` / `bool` / `string` / `array` / `object` / `bytes`。

在 JSON 传输中，`bytes` 类型通常以 base64 字符串承载。图像类 Observation 的 `value` 返回结构化对象：`{width, height, encoding, image}`，其中 `image` 是图像字节内容，JSON 传输时以 base64 字符串承载；图像编码由 `value.encoding` 和 `value_schema.fields.image.encoding` 描述。

`value_schema` 写法：

*   若 `value` 是二进制数据，使用 `{"type": "bytes", "encoding": "..."}` 描述。

*   若 `value` 是图像数据，使用 `{"type": "object", "fields": {"width": {"type": "int"}, "height": {"type": "int"}, "encoding": {"type": "string"}, "image": {"type": "bytes", "encoding": "jpg"}}}` 描述。

*   若 `value` 是结构化对象，使用 `{"type": "object", "fields": {...}}` 描述。

*   `fields` 中的每个字段使用 `type`、`items`、`shape`、`unit`、`optional` 描述。

*   `optional=true` 表示该字段可不返回；未声明 `optional` 的字段默认为必选。

*   `shape` 使用整数数组描述实际维度；若字段按机器人统一关节顺序排列，`shape` 中应填写真实关节维度，例如 `[6]`。


Observation 常见类型参考：

| name 示例 | type 示例 | 说明 | value 示例 |
| --- | --- | --- | --- |
| observation.camera\_x.image | `image` | 相机图像 | {width: int, height: int, encoding: string, image: bytes}；JSON 中 image 为 base64 字符串 |
| observation.robot.joint\_state | `joint_state` | 机器人关节状态；夹爪若作为关节建模，也包含在统一关节顺序中 | {joint\_positions, joint\_velocities?} |
| observation.arm\_x.ee\_pose | `ee_pose` | 末端位姿 | {position: \[x,y,z\], orientation\_quat\_xyzw: \[x,y,z,w\]} |
| observation.gripper\_x.state | `gripper_state` | 夹爪状态 | {position, force?} |
| observation.imu\_x.state | `imu` | IMU 数据 | {accel: \[x,y,z\], gyro: \[x,y,z\], mag?: \[x,y,z\]} |
| observation.force\_x.state | `force_torque` | 力/力矩数据 | {force: \[fx,fy,fz\], torque?: \[tx,ty,tz\]} |
| observation.tactile\_x.state | `tactile` | 触觉数据 | {contact\_press\_map?: {width, height, encoding, data}, pressure\_map?: float\[\], contact?: bool, slip?: bool} |
| observation.lidar\_x.points | `point_cloud` | 点云数据 | 按传输协议约定的点云数据 |

触觉数据字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `contact_press_map` | object | 按触觉传感器阵列排布组织的接触压力图，包含 `width`, `height`, `encoding`, `data`；`encoding` 可为 `mono8` / `float32` / `float64`，JSON 中 `data` 为 base64 字符串 |
| `pressure_map` | float\[\] | 展平后的压力数组，适用于不需要表达二维阵列排布的触觉传感器 |
| `contact` | bool | 当前是否检测到接触 |
| `slip` | bool | 当前是否检测到滑移 |

`get_observations` 获取指定 Observation 的当前值。`sync=true` 时，RCP 尽量返回时间戳差异最小的一组观测值（高频数据会向低频数据对齐，意味着获得的数据中，高频数据将会是历史帧，而不是最新的）；`sync=false` 或未提供时，各观测对象返回当前最新值。

`get_observations` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `names` | array | 必选 | 要读取的 Observation 名称列表 |
| `sync` | boolean | 可选 | 是否尽量时间对齐；默认为 `false` |

`get_observations` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `observations` | array | 必选 | 观测值列表 |
| `observations[ ].name` | string | 必选 | Observation 名称 |
| `observations[ ].timestamp` | float | 必选 | 观测值时间戳 |
| `observations[ ].value` | any | 必选 | 观测值，结构由对应 Observation 的 `value_schema` 定义 |

`subscribe_observations` 按指定帧率持续推送 Observation 值。

`subscribe_observations` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `names` | array | 必选 | 要订阅的 Observation 名称列表 |
| `frame_rate` | float | 必选 | 推送帧率，单位 Hz |

`subscribe_observations` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `subscription_id` | string | 必选 | 当前 RCP 实例内递增的订阅 ID |

`subscribe_observations` 回调字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `type` | string | 必选 | 固定为 `observation` |
| `subscription_id` | string | 必选 | 对应订阅 ID |
| `data.name` | string | 必选 | Observation 名称 |
| `data.timestamp` | float | 必选 | 观测值时间戳 |
| `data.value` | any | 必选 | 观测值，结构由对应 Observation 的 `value_schema` 定义 |

##### 执行（Action）

Action 是 RCP 可提交给底层 controller / SDK 的动作入口。RCP 的 action 接口提交有限时长或有限步长的动作请求，高频控制、轨迹插补、力控、碰撞保护和稳定控制由机器人本体侧 controller / SDK / safety layer 负责。典型 Action 包括：机械臂关节目标、末端位姿目标、夹爪开合、底盘速度、头部/全身动作等。

Action 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| list\_actions | 查询所有 Action 的完整描述 | × | para：无<br>ret：{actions: \[{name, type, description?, frame\_rate?, input\_schema}\]} |
| run\_action\_chunk | 同步向指定 Action 对象提交有限动作片段（非硬实时控制接口） | × | para：name（目标 Action）, frames（动作帧序列，元素直接为目标 Action 输入对象）, frame\_rate（目标执行帧率，单位 Hz）<br>ret：{accepted\_frames} |
| run\_action\_chunk\_async | 异步向指定 Action 对象提交有限动作片段（非硬实时控制接口） | ✓ | para：name（目标 Action）, frames（动作帧序列，元素直接为目标 Action 输入对象）, frame\_rate（目标执行帧率，单位 Hz）<br>ret：{accepted\_frames}<br>callback：{id, type: "action\_step", data: {name, step, total, done}} |
| stop\_action | 停止当前所有动作 | × | para：reason?<br>ret： |

> Manifest 提供 Action 概要（名称+类型），list\_actions 提供完整描述（含 input\_schema），run\_action\_chunk / run\_action\_chunk\_async 负责执行，stop\_action 负责停止。Action chunk 接口只用于有限动作片段提交、短轨迹、调试或回放，不承担硬实时控制闭环。

`list_actions` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 动作对象唯一标识，如 `action.robot.joint_position` / `action.gripper_0.position` |
| `type` | string | 必选 | 动作输入类型，如 `joint_position` / `joint_velocity` / `ee_pose` / `gripper` / `base_velocity` / `head_pose` / `prearranged` / `custom` |
| `description` | string | 可选 | 人类可读描述 |
| `frame_rate` | float | 可选 | 默认或建议执行帧率，单位 Hz |
| `input_schema` | object | 必选 | `frames[ ]` 字段的轻量结构说明，用于描述动作输入的类型、字段、维度或单位 |

`input_schema` 采用与 `value_schema` 一致的数据结构说明。`input_schema` 只描述 Action chunk 中每个 `frames[ ]` 的结构，不描述 `name`、`frames` 或 `frame_rate`。

`input_schema` 写法：

*   若 `frames[ ]` 是结构化对象，使用 `{"type": "object", "fields": {...}}` 描述。

*   `fields` 中的每个字段使用 `type`、`items`、`shape`、`unit`、`optional` 描述。

*   `optional=true` 表示该字段可不传；未声明 `optional` 的字段默认为必传。

*   `shape` 使用整数数组描述实际维度；若字段按机器人统一关节顺序排列，`shape` 中应填写真实关节维度，例如 `[6]`。


`prearranged` 与 `custom` 的边界：

*   `prearranged` 用于固定预定义动作，例如 `action.robot.home`；调用方只负责触发动作，`frames[ ]` 必须为 `{}`。

    custom 用于机器人侧自定义且需要结构化入参的动作，例如 action.robot.move\_to；调用方直接按该 Action 在 list\_actions 返回的 input\_schema 传入 frames\[ \]。

*   `custom` 不要求动作对象命名为 `action.robot.custom`。协议推荐使用语义明确的 Action 名称，例如 `action.robot.move_to`、`action.robot.calibrate`、`action.gripper_0.custom_grasp`。


Action 常见类型参考：

| name 示例 | type 示例 | 说明 | frames\[ \] 示例 |
| --- | --- | --- | --- |
| action.robot.joint\_position | `joint_position` | 机器人整体关节位置控制，适用于 VLA / RL / policy 整体输出；夹爪若作为关节控制，也包含在统一关节顺序中 | `{joint_positions: [float...]}` |
| action.mobile\_base.velocity | `base_velocity` | 底盘速度控制 | `{linear_vel?: [x,y,z], angular_vel?: [x,y,z]}` |
| action.gripper\_x.position | `gripper` | 夹爪位置或力控制 | `{position?: float, force?: float}` |
| action.robot.home | `prearranged` | 机器人侧预定义固定动作序列 | `{}` |
| action.robot.move\_to | `custom` | 机器人侧自定义动作 | `joint_positions: [float...]}` |

`run_action_chunk` 的 `name` 指定目标 Action；`frames` 为动作帧序列，每项直接是目标 Action 的输入对象，结构由对应 Action 的 input\_schema 定义。`frame_rate` 表示动作帧执行帧率，单位 Hz。RCP 同步执行本次 chunk，执行完成后返回。

`run_action_chunk` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 目标 Action 名称 |
| `frames` | array | 必选 | 动作帧序列 |
| `frames[ ]` | object | 必选 | 单帧动作输入，结构由目标 Action 的 `input_schema` 定义 |
| `frame_rate` | float | 必选 | 动作帧执行帧率，单位 Hz |

`run_action_chunk_async` 的参数字段与 `run_action_chunk` 一致。RCP 接收动作帧后立即返回，每执行完一帧通过 `action_step` 回调通知进度，`done=true` 表示本次 chunk 已执行到最后一帧。

`run_action_chunk_async` 异步回调字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `id` | string | 必选 | 对应本次 `run_action_chunk_async` 请求 ID |
| `type` | string | 必选 | 固定为 `action_step` |
| `data.name` | string | 必选 | 目标 Action 名称 |
| `data.step` | int | 必选 | 已执行完成的帧序号，从 1 开始 |
| `data.total` | int | 必选 | 本次 chunk 的总帧数 |
| `data.done` | boolean | 必选 | 是否已执行到本次 chunk 的最后一帧 |

`stop_action` 停止当前所有动作，`reason` 可用于填写停止原因。

`stop_action` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `reason` | string | 可选 | 停止原因 |

##### 策略服务（PolicyService）

PolicyService 用于运行绑定在机器人构型上的本地策略。策略读取 Observation 和运行时输入，输出 `run_action_chunk` 入参形状，最终仍通过 ActionService 执行。策略不直接访问 channel、controller、串口或相机。

策略服务保持以下语义：

*   `start_policy` 启动后持续推理，直到 `stop_policy` 或切换到另一个策略。

*   `start_policy` 也用于策略切换；Server 会先加载并 reset 新策略，成功后再停止旧策略。

*   `update_policy_inputs` 只更新最新运行时输入，不排队。后续推理一直使用最新值。

*   未设置 `stale_after_ms` 的运行时输入会持续保持最新值；设置后，超过该时间未更新则回到 `default`。

*   策略输出的 `name` 必须是完整 RCP Action 名称，例如 `action.robot.joint_position`，并且必须出现在策略的 `outputs.actions` 中。

*   策略输出的 `frames` 和 `frame_rate` 直接作为 `run_action_chunk` 参数。RL 通常返回 1 帧，VLA 通常返回多帧；本次输出覆盖的动作时长为 `len(frames) / frame_rate`。


PolicyService 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| list\_policies | 列出可用策略和当前运行策略 | × | para：无<br>ret：{policies, active\_policy\_id?, last\_error?} |
| start\_policy | 启动策略；若已有策略运行，则切换到新策略 | × | para：policy\_id, runtime\_inputs?<br>ret：{active\_policy\_id} |
| update\_policy\_inputs | 更新当前策略的运行时输入 | × | para：policy\_id?, runtime\_inputs<br>ret：{active\_policy\_id, runtime\_inputs} |
| stop\_policy | 停止当前策略 | × | para：policy\_id?, reason?<br>ret：{stopped\_policy\_id} |

`list_policies` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `policies` | array | 必选 | 可用策略列表 |
| `policies[ ].policy_id` | string | 必选 | 策略唯一标识 |
| `policies[ ].name` | string | 必选 | 策略展示名称 |
| `policies[ ].inputs.observations` | array | 必选 | 策略需要的 Observation 引用；每项包含 `component`、`name` 和解析后的 `key` |
| `policies[ ].inputs.runtime_inputs` | object | 可选 | App 可更新的运行时输入及其 schema |
| `policies[ ].outputs.actions` | array | 必选 | 策略允许输出的 Action 引用；每项包含 `component`、`name` 和解析后的 `key` |
| `active_policy_id` | string/null | 可选 | 当前运行策略 ID；无策略运行时为空 |
| `last_error` | string | 可选 | 最近一次策略循环错误信息 |

`runtime_inputs` schema 字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `type` | string | 必选 | 使用与 `value_schema` 一致的基础类型：`float` / `int` / `bool` / `string` / `array` / `object` / `bytes` |
| `items` | string | 可选 | `type=array` 时的元素类型 |
| `shape` | array | 可选 | `type=array` 时的维度 |
| `default` | any | 可选 | 默认值 |
| `stale_after_ms` | int | 可选 | 多久未更新后回到 `default`，通常用于速度、方向等连续控制输入 |

`start_policy` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `policy_id` | string | 必选 | 要启动的策略 ID |
| `runtime_inputs` | object | 可选 | 启动时传入的运行时输入 |

`update_policy_inputs` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `policy_id` | string | 可选 | 要更新的策略 ID；未传时默认当前运行策略 |
| `runtime_inputs` | object | 必选 | 要更新的运行时输入；未传字段保持原值 |

`stop_policy` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `policy_id` | string | 可选 | 要停止的策略 ID；未传时默认当前运行策略 |
| `reason` | string | 可选 | 停止原因 |

策略输出必须为 `run_action_chunk` 入参形状：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 必选 | 完整 RCP Action 名称 |
| `frames` | array | 必选 | 动作帧序列；每项直接是目标 Action 的输入对象，结构由目标 Action 的 `input_schema` 定义 |
| `frame_rate` | float | 必选 | 动作帧执行帧率，单位 Hz |

##### 本体健康状态（Health）

Health 用于获取机器人本体、控制器或驱动侧当前健康状态、错误与告警信息，也可订阅健康状态变化。

Health 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| get\_health | 获取当前健康状态、错误与告警信息 | × | para：无<br>ret： |
| subscribe\_health | 订阅健康状态变化 | ✓ | para：无<br>ret：{subscription\_id}<br>callback：{type, subscription\_id, data: {timestamp, msg}} |

`get_health` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `msg` | array | 必选 | 当前健康状态消息列表；无消息时为空数组 |

`subscribe_health` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `subscription_id` | string | 必选 | 当前 RCP 实例内递增的订阅 ID |

`subscribe_health` 回调字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `type` | string | 必选 | 固定为 `health` |
| `subscription_id` | string | 必选 | 对应订阅 ID |
| `data.timestamp` | float | 必选 | 本次状态变化时间 |
| `data.msg` | array | 必选 | 当前健康状态消息列表；无消息时为空数组 |

健康状态消息项字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `level` | string | 必选 | 消息级别，如 `error` / `warning` / `info` |
| `code` | string | 可选 | 本体、控制器或驱动返回的消息码 |
| `message` | string | 必选 | 健康状态消息内容 |
| `source` | string | 可选 | 来源，如 robot / controller / driver |
| `timestamp` | float | 可选 | 消息产生时间 |
| `details` | object | 可选 | 设备、驱动或控制器返回的补充信息 |

Health 可通过 `msg` 返回组件、控制器或驱动相关的不同健康状态消息；当观测数据缺失或超时时，也可通过 `msg` 表达。

##### 资源读取（Resource）

Resource 用于在不暴露服务端本地路径的前提下，描述、枚举和读取 RCP 实例侧的文件、目录、日志、模型资产、采集数据等资源。Resource 只约束跨 RCP 协议边界的资源交换。

资源通过 `resource_id` 标识。`resource_id` 由 RCP 实例生成，在当前 RCP 实例内唯一；调用方不应解析 `resource_id` 的内部结构，只能将其作为后续资源接口的参数使用。

Resource 可用于：

*   获取 URDF / MJCF / calibration / mesh / texture 等模型资源。

*   获取日志文件或实时日志。

*   导出数据采集产物。

*   将目录打包为 archive 后下载。


每个 Resource 对象包含以下字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `resource_id` | string | 必选 | 资源标识，不应包含服务端本地路径 |
| `type` | string | 必选 | 资源类型，如 `file` / `directory` / `archive` / `stream` |
| `domain` | string | 可选 | 资源领域，如 `model` / `asset` / `calibration` / `log` / `data` / `report` |
| `name` | string | 可选 | 人类可读资源名，不要求唯一 |
| `format` | string | 可选 | 资源格式，如 `urdf` / `mjcf` / `json` / `yaml` / `zip` / `text` / `jpeg` / `bin` |
| `mime_type` | string | 可选 | MIME 类型 |
| `mode` | string | 必选 | `snapshot` 表示内容固定；`live` 表示内容可能变化 |
| `size_bytes` | int | 可选 | 当前资源大小；`live` 资源中表示当前已知大小 |
| `hash` | string | 可选 | 内容哈希，通常用于 `snapshot` 资源 |
| `created_at` | float | 可选 | 创建时间戳 |
| `updated_at` | float | 可选 | 最近更新时间戳 |
| `metadata` | object | 可选 | 资源相关补充信息 |

Resource 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| get\_resource\_info | 获取资源描述信息 | × | para：resource\_id<br>ret： |
| list\_resources | 枚举当前 RCP 实例可公开访问的资源 | × | para：domain?, kind?, cursor?, limit?<br>ret： |
| list\_resource\_entries | 枚举目录资源的子资源 | × | para：resource\_id, recursive?, cursor?, limit?<br>ret： |
| read\_resource | 分片读取字节型资源 | × | para：resource\_id, offset?, limit?<br>ret： |
| snapshot\_resource | 将 live 资源固化为 snapshot 资源 | × | para：resource\_id<br>ret： |
| prepare\_resource\_archive | 将目录或多个资源打包为 archive 资源 | × | para：resource\_id 或 resource\_ids, format?<br>ret： |

`list_resources` 用于列出当前 RCP 实例主动公开的资源目录和资源项。服务端只应把协议允许跨边界访问的资源注册到 Resource catalog 中，例如采集数据、日志、模型文件、标定文件或临时归档文件。

`list_resources` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `domain` | string | 可选 | 按资源领域过滤，如 `data` / `log` / `model` / `calibration` |
| `kind` | string | 可选 | 按 `metadata.kind` 过滤，如 `collection` / `log` / `model` |
| `cursor` | string | 可选 | 分页游标 |
| `limit` | int | 可选 | 单次返回数量 |

`list_resources` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `resources` | array | 必选 | Resource 对象列表 |
| `next_cursor` | string | 可选 | 下一页游标；不存在表示已返回完毕 |

常见过滤方式：

| 场景 | 调用 |
| --- | --- |
| 查看服务端采集 episode | `list_resources(domain="data", kind="collection")` |
| 查看服务端日志资源 | `list_resources(domain="log")` |
| 查看模型或标定资源 | `list_resources(domain="model")` 或 `list_resources(domain="calibration")` |

当前标准 runtime 默认会注册以下 Resource catalog：

| catalog | domain | kind | 内容 |
| --- | --- | --- | --- |
| `collections` | `data` | `collections_root` / `collection` | 采集数据根目录，以及其中包含 `collection_meta.json` 的 episode 目录 |
| `logs` | `log` | `logs_root` / `log` | server 日志目录，以及其中的日志文件 |

后续接入模型、标定或其他机器人资源时，应继续通过 Resource catalog 暴露。

`read_resource` 仅用于读取 `file` / `archive` 等字节型资源。若资源类型为 `directory`，调用方应使用 `list_resource_entries` 枚举其子资源，或使用 `prepare_resource_archive` 将目录打包为 `archive` 后读取。

对于 `live` 资源，`read_resource` 应支持 `offset` / `limit` 增量读取，并返回 `next_offset` 和 `eof`。若调用方需要稳定内容，可先调用 `snapshot_resource` 生成固定快照。

在 JSON 传输中，`read_resource.data` 使用 base64 字符串承载二进制内容。

##### 数据采集（Data Collection）

Data Collection 提供原始数据录制能力，使用用户指定的 `collection_id` 和 `episode_id` 标识采集任务，支持同时录制多个 Observation 和 Action 对象。

`collection_id` 表示一批采集任务，`episode_id` 表示该批任务中的一次采集。两者都是字符串，不从目录名互相解析。任务语义由 `task_prompt` 和 `task_description` 记录。同一时间仅支持一个采集任务。

Data Collection 协议返回采集产物时使用 Resource 对象描述。调用方若需要获取本次采集目录内容，可通过返回的 `collection_resource` 调用 `list_resource_entries` 枚举，或通过 `prepare_resource_archive` 生成 archive 资源后使用 `read_resource` 下载。调用方若需要查看服务端历史采集数据，应使用 `list_resources(domain="data", kind="collection")` 获取可用采集资源列表。

Data Collection 可用工具：

| 工具名 | 意义 | 回调 | 参数 |
| --- | --- | --- | --- |
| start\_collection | 开始原始录制 | × | para：names, collection\_id, episode\_id, task\_prompt, task\_description, max\_duration?<br>ret： |
| stop\_collection | 停止录制并归档 | × | para：无<br>ret： |
| get\_collection\_status | 查询当前录制状态 | × | para：无<br>ret： |
| delete\_collection | 删除指定采集资源 | × | para：resource\_id<br>ret： |

`start_collection` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `names` | array | 必选 | 需要录制的对象名称列表，可包含 Observation 和 Action |
| `collection_id` | string | 必选 | 一批采集任务的标识 |
| `episode_id` | string | 必选 | 该批任务中的一次采集标识，可为任意字符串 |
| `task_prompt` | string | 必选 | 本次采集对应的任务指令 |
| `task_description` | string | 必选 | 本次采集对应的任务描述 |
| `max_duration` | float | 可选 | 最大录制时长，单位秒 |

`start_collection` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `collection_id` | string | 必选 | 一批采集任务的标识 |
| `episode_id` | string | 必选 | 该批任务中的一次采集标识 |
| `collection_resource` | object | 必选 | 本次采集产物对应的 Resource 对象；录制中通常为 `directory` / `live` |
| `names` | array | 必选 | 实际录制的对象名称列表 |
| `task_prompt` | string | 必选 | 本次采集对应的任务指令 |
| `task_description` | string | 必选 | 本次采集对应的任务描述 |
| `started_at` | float | 必选 | 采集开始时间 |

`stop_collection` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `collection_id` | string | 必选 | 一批采集任务的标识 |
| `episode_id` | string | 必选 | 该批任务中的一次采集标识 |
| `duration` | float | 必选 | 实际录制时长，单位秒 |
| `per_name_counts` | object | 必选 | 各对象录制帧数 |
| `collection_resource` | object | 必选 | 本次采集产物对应的 Resource 对象；停止后通常为 `directory` / `snapshot` |

`get_collection_status` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `running` | boolean | 必选 | 当前是否正在录制 |
| `collection_id` | string | 可选 | 当前采集任务标识 |
| `episode_id` | string | 可选 | 当前 episode 标识 |
| `collection_resource` | object | 可选 | 当前采集产物对应的 Resource 对象 |
| `names` | array | 可选 | 当前录制的对象名称列表 |
| `counts` | object | 可选 | 各对象已录制帧数 |
| `duration` | float | 可选 | 当前已录制时长，单位秒 |
| `task_prompt` | string | 可选 | 当前采集对应的任务指令 |
| `task_description` | string | 可选 | 当前采集对应的任务描述 |

`delete_collection` 参数字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `resource_id` | string | 必选 | 要删除的采集资源标识，通常来自 `collection_resource.resource_id` |

`delete_collection` 返回字段：

| 字段 | 类型 | 必选性 | 说明 |
| --- | --- | --- | --- |
| `deleted` | boolean | 必选 | 是否已删除指定采集资源 |

---

### JSON通信协议规范

针对基于JSON的通信协议（MCP、自定义JSON-RPC等），符合如下规范：

#### 通用请求格式

```json
{
    "id": "1",
    "method": "tool_call",
    "params": {
        "tool": "<工具名>",
        "args": { ... }
    }
}

```

#### 通用响应格式

```json
{
    "id": "1",
    "code": 0,
    "message": "ok",
    "data": { ... }
}

```

#### 错误码

| code | 含义 |
| --- | --- |
| 0 | 成功 |
| 1001 | 工具不存在 |
| 1002 | 参数校验失败 |
| 1003 | 通道无数据 |
| 1004 | 设备未连接 |
| 1005 | 重复启动 |
| 1006 | 未在运行 |
| 1007 | 操作超时 |
| 1008 | 内部错误 |

#### 回调/推送格式（长程任务、订阅）

```json
{
    "id": "1",
    "type": "progress",
    "data": {
        "progress": 50,
        "total": 100,
        "message": "Executing frame 50/100"
    }
}

```
```json
{
    "id": "1",
    "type": "push",
    "data": {
        "obs_name": "obs/robot_0",
        "timestamp": 1745472000.456,
        "obs_value": { ... }
    }
}

```
---

#### 各接口JSON示例

##### get\_manifest

```json
// 请求
{"id": "1", "method": "get_manifest", "params": {}}

// 响应
{
    "id": "1", "ok": true,
    "result": {
        "robot_id": "so101-arm-01",
        "robot_name": "单臂机器人",
        "embodiment_type": "single_arm",
        "components": [
            {
                "name": "arm_0",
                "type": "arm",
                "dof": 6,
                "frame": "arm_0_ee",
                "description": "6DoF 右臂"
            },
            {
                "name": "gripper_0",
                "type": "gripper",
                "dof": 1,
                "parent_component": "arm_0",
                "frame": "gripper_0",
                "description": "二指夹爪"
            },
            {
                "name": "camera_0",
                "type": "camera",
                "parent_component": "arm_0",
                "frame": "camera_0",
                "description": "腕部 RGB 相机"
            },
            {
                "name": "imu_0",
                "type": "imu",
                "parent_component": "arm_0",
                "frame": "imu_0",
                "description": "腕部 IMU"
            }
        ],
        "observations": [
            {"name": "observation.camera_0.image", "type": "image", "component_name": "camera_0", "description": "腕部 RGB 相机图像"},
            {"name": "observation.robot.joint_state", "type": "joint_state", "description": "机器人关节状态"},
            {"name": "observation.arm_0.ee_pose", "type": "ee_pose", "component_name": "arm_0", "description": "机械臂末端位姿"},
            {"name": "observation.imu_0.state", "type": "imu", "component_name": "imu_0", "description": "腕部 IMU 数据"}
        ],
        "actions": [
            {"name": "action.robot.joint_position", "type": "joint_position", "description": "机器人整体关节位置控制"},
            {"name": "action.arm_0.ee_pose", "type": "ee_pose", "component_name": "arm_0", "description": "机械臂末端位姿控制"},
            {"name": "action.gripper_0.position", "type": "gripper", "component_name": "gripper_0", "description": "夹爪开合控制"},
            {"name": "action.robot.home", "type": "prearranged", "description": "机器人回到初始姿态"},
            {"name": "action.robot.move_to", "type": "custom", "description": "机器人侧插值移动到指定关节位置"}        ],
        "model_refs": {
            "urdf": {
                "format": "urdf",
                "content": "<robot name=\"so101\">...</robot>",
                "hash": "sha256:4d9672..."
            },
            "mjcf": {
                "format": "mjcf",
                "content": "<mujoco model=\"so101\">...</mujoco>",
                "hash": "sha256:9ab031..."
            },
            "calibration": {
                "format": "yaml",
                "content": "camera_0:\n  intrinsics: ...\n",
                "hash": "sha256:6efc20..."
            },
            "assets": [
                {
                    "resource_id": "res_asset_8f3c1d2e",
                    "type": "file",
                    "domain": "asset",
                    "name": "base.STL",
                    "format": "stl",
                    "mode": "snapshot",
                    "size_bytes": 2483912,
                    "hash": "sha256:8f3c1d2e..."
                }
            ]
        },
        "capabilities": {
            "observations": true,
            "actions": true,
            "health": true,
            "resources": true,
            "data_collection": true
        },
        "metadata": {
            "custom_note": "SO101 单臂机器人示例"
        }
    }
}


```

##### list\_observations

```json
// 请求
{"id": "2", "method": "list_observations", "params": {}}

// 响应
{
    "id": "2", "ok": true,
    "result": {
        "observations": [
            {
                "name": "observation.camera_0.image",
                "type": "image",
                "description": "腕部 RGB 相机",
                "value_schema": {
                    "type": "object",
                    "fields": {
                        "width": {"type": "int"},
                        "height": {"type": "int"},
                        "encoding": {"type": "string"},
                        "image": {"type": "bytes", "encoding": "jpg"}
                    }
                },
                "frame_rate": 30
            },
            {
                "name": "observation.robot.joint_state",
                "type": "joint_state",
                "description": "机器人关节状态",
                "value_schema": {
                    "type": "object",
                    "fields": {
                        "joint_positions": {"type": "array", "items": "float", "shape": [6], "unit": "rad"},
                        "joint_velocities": {"type": "array", "items": "float", "shape": [6], "unit": "rad/s", "optional": true}
                    }
                },
                "frame_rate": 100
            },
            {
                "name": "observation.arm_0.ee_pose",
                "type": "ee_pose",
                "description": "6DoF 右臂末端位姿",
                "value_schema": {
                    "type": "object",
                    "fields": {
                        "position": {"type": "array", "items": "float", "shape": [3], "unit": "m"},
                        "orientation_quat_xyzw": {"type": "array", "items": "float", "shape": [4]}
                    }
                },
                "frame_rate": 100
            },
            {
                "name": "observation.imu_0.state",
                "type": "imu",
                "description": "腕部 IMU",
                "value_schema": {
                    "type": "object",
                    "fields": {
                        "accel": {"type": "array", "items": "float", "shape": [3], "unit": "m/s^2"},
                        "gyro": {"type": "array", "items": "float", "shape": [3], "unit": "rad/s"},
                        "mag": {"type": "array", "items": "float", "shape": [3], "optional": true}
                    }
                },
                "frame_rate": 200
            },
            {
                "name": "observation.tactile_0.state",
                "type": "tactile",
                "description": "夹爪触觉阵列",
                "value_schema": {
                    "type": "object",
                    "fields": {
                        "contact_press_map": {
                            "type": "object",
                            "fields": {
                                "width": {"type": "int"},
                                "height": {"type": "int"},
                                "encoding": {"type": "string"},
                                "data": {"type": "bytes"}
                            },
                            "optional": true
                        },
                        "pressure_map": {"type": "array", "items": "float", "optional": true},
                        "contact": {"type": "bool", "optional": true},
                        "slip": {"type": "bool", "optional": true}
                    }
                },
                "frame_rate": 100
            }
        ]
    }
}


```

##### get\_observations

```json
// 请求
{
    "id": "3", "method": "get_observations",
    "params": {
        "names": ["observation.camera_0.image", "observation.robot.joint_state", "observation.tactile_0.state"],
        "sync": true
    }
}

// 响应
{
    "id": "3", "ok": true,
    "result": {
        "observations": [
            {
                "name": "observation.camera_0.image",
                "timestamp": 1745472000.123,
                "value": {
                    "width": 640,
                    "height": 480,
                    "encoding": "jpg",
                    "image": "/9j/4AAQSkZJRg..."
                }
            },
            {
                "name": "observation.robot.joint_state",
                "timestamp": 1745472000.125,
                "value": {
                    "joint_positions": [0.0, -1.57, 1.57, 0.0, 0.78, 0.0],
                    "joint_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                }
            },
            {
                "name": "observation.tactile_0.state",
                "timestamp": 1745472000.126,
                "value": {
                    "contact_press_map": {
                        "width": 16,
                        "height": 16,
                        "encoding": "mono8",
                        "data": "AAECAwQF..."
                    },
                    "contact": true,
                    "slip": false
                }
            }
        ]
    }
}


```

##### subscribe\_observations

```json
// 请求
{
    "id": "4", "method": "subscribe_observations",
    "params": {
        "names": ["observation.robot.joint_state"],
        "frame_rate": 30
    }
}

// 响应（确认）
{"id": "4", "ok": true, "result": {"subscription_id": "1"}}

// 推送（持续）
{
    "type": "observation",
    "subscription_id": "1",
    "data": {
        "name": "observation.robot.joint_state",
        "timestamp": 1745472000.456,
        "value": {
            "joint_positions": [0.1, -1.47, 1.47, 0.1, 0.68, 0.1]
        }
    }
}


```

##### list\_actions

```json
// 请求
{"id": "5", "method": "list_actions", "params": {}}

// 响应
{
    "id": "5", "ok": true,
    "result": {
        "actions": [
            {
                "name": "action.robot.joint_position",
                "type": "joint_position",
                "description": "机器人整体关节位置控制",
                "input_schema": {
                    "type": "object",
                    "fields": {
                        "joint_positions": {"type": "array", "items": "float", "shape": [6], "unit": "rad"}
                    }
                },
                "frame_rate": 30
            },
            {
                "name": "action.arm_0.ee_pose",
                "type": "ee_pose",
                "description": "6DoF 右臂末端位姿控制",
                "input_schema": {
                    "type": "object",
                    "fields": {
                        "position": {"type": "array", "items": "float", "shape": [3], "unit": "m"},
                        "orientation_quat_xyzw": {"type": "array", "items": "float", "shape": [4]}
                    }
                },
                "frame_rate": 30
            },
            {
                "name": "action.gripper_0.position",
                "type": "gripper",
                "description": "二指夹爪控制",
                "input_schema": {
                    "type": "object",
                    "fields": {
                        "position": {"type": "float", "optional": true},
                        "force": {"type": "float", "optional": true}
                    }
                },
                "frame_rate": 30
            },
            {
                "name": "action.robot.home",
                "type": "prearranged",
                "description": "机器人回到初始姿态",
                "input_schema": {"type": "object", "fields": {}}
            },
            {
                "name": "action.robot.move_to",
                "type": "custom",
                "description": "机器人侧插值移动到指定关节位置",
                "input_schema": {
                  "type": "object",
                  "fields": {
                    "joint_positions": {
                      "type": "array",
                      "items": "float",
                      "shape": [6],
                      "unit": "rad"
                    }
                  }
                }
            }
        ]
    }
}

```

##### run\_action\_chunk

```json
// 请求
{
    "id": "6", "method": "run_action_chunk",
    "params": {
        "name": "action.robot.joint_position",
        "frames": [
          {"joint_positions": [0.0, -1.57, 1.57, 0.0, 0.78, 0.5]},
          {"joint_positions": [0.1, -1.47, 1.47, 0.1, 0.68, 0.4]}
        ],
        "frame_rate": 30
    }
}

// 响应
{
    "id": "6", "ok": true,
    "result": {
        "accepted_frames": 2
    }
}

```

##### run\_action\_chunk\_async

```json
// 请求
{
    "id": "7", "method": "run_action_chunk_async",
    "params": {
        "name": "action.robot.joint_position",
        "frames": [
          {"joint_positions": [0.0, -1.57, 1.57, 0.0, 0.78, 0.5]},
          {"joint_positions": [0.1, -1.47, 1.47, 0.1, 0.68, 0.4]}
        ],
        "frame_rate": 30
    }
}

// 响应
{
    "id": "7", "ok": true,
    "result": {
        "accepted_frames": 2
    }
}

// 异步进度回调
{"id": "7", "type": "action_step", "data": {"name": "action.robot.joint_position", "step": 1, "total": 2, "done": false}}
{"id": "7", "type": "action_step", "data": {"name": "action.robot.joint_position", "step": 2, "total": 2, "done": true}}


```

##### stop\_action

```json
// 请求
{
    "id": "8", "method": "stop_action",
    "params": {
        "reason": "user_request"
    }
}

// 响应
{"id": "8", "ok": true, "result": {"stopped": true}}


```

##### list\_policies

```json
// 请求
{"id": "9", "method": "list_policies", "params": {}}

// 响应
{
    "id": "9", "ok": true,
    "result": {
        "policies": [
            {
                "policy_id": "walk",
                "name": "Walk Policy",
                "inputs": {
                    "observations": [
                        {"component": "robot", "name": "joint_state", "key": "observation.robot.joint_state"}
                    ],
                    "runtime_inputs": {
                        "velocity_xyz": {
                            "type": "array",
                            "items": "float",
                            "shape": [3],
                            "default": [0, 0, 0],
                            "stale_after_ms": 500
                        },
                        "yaw_rate": {
                            "type": "float",
                            "default": 0,
                            "stale_after_ms": 500
                        }
                    }
                },
                "outputs": {
                    "actions": [
                        {"component": "robot", "name": "joint_position", "key": "action.robot.joint_position"}
                    ]
                }
            },
            {
                "policy_id": "pick_place",
                "name": "Pick Place VLA",
                "inputs": {
                    "observations": [
                        {"component": "robot", "name": "joint_state", "key": "observation.robot.joint_state"},
                        {"component": "front_camera", "name": "image", "key": "observation.front_camera.image"}
                    ],
                    "runtime_inputs": {
                        "prompt": {
                            "type": "string",
                            "default": "PickPlace"
                        }
                    }
                },
                "outputs": {
                    "actions": [
                        {"component": "robot", "name": "joint_position", "key": "action.robot.joint_position"}
                    ]
                }
            }
        ],
        "active_policy_id": null,
        "last_error": ""
    }
}


```

##### start\_policy

```json
// 请求
{
    "id": "10", "method": "start_policy",
    "params": {
        "policy_id": "walk",
        "runtime_inputs": {
            "velocity_xyz": [0, 0, 0],
            "yaw_rate": 0
        }
    }
}

// 响应
{
    "id": "10", "ok": true,
    "result": {
        "active_policy_id": "walk"
    }
}


```

策略 `step` 返回的动作片段示例：

```json
{
    "name": "action.robot.joint_position",
    "frames": [
      {"joint_positions": [0.0, -1.57, 1.57, 0.0, 0.78, 0.5]}
    ],
    "frame_rate": 50
}

```

VLA 策略可一次返回多帧：

```json
{
    "name": "action.robot.joint_position",
    "frames": [
      {"joint_positions": [0.0, -1.57, 1.57, 0.0, 0.78, 0.5]},
      {"joint_positions": [0.1, -1.47, 1.47, 0.1, 0.68, 0.4]}
    ],
    "frame_rate": 50
}

```

##### update\_policy\_inputs

```json
// 请求
{
    "id": "11", "method": "update_policy_inputs",
    "params": {
        "runtime_inputs": {
            "velocity_xyz": [0.2, 0, 0],
            "yaw_rate": 0.1
        }
    }
}

// 响应
{
    "id": "11", "ok": true,
    "result": {
        "active_policy_id": "walk",
        "runtime_inputs": {
            "velocity_xyz": [0.2, 0, 0],
            "yaw_rate": 0.1
        }
    }
}


```

##### stop\_policy

```json
// 请求
{"id": "12", "method": "stop_policy", "params": {}}

// 响应
{
    "id": "12", "ok": true,
    "result": {
        "stopped_policy_id": "walk"
    }
}


```

##### get\_health

```json
// 请求
{"id": "13", "method": "get_health", "params": {}}

// 响应
{
    "id": "13", "ok": true,
    "result": {
        "msg": [
            {
                "level": "warning",
                "code": "LOW_BATTERY",
                "message": "Battery level is below warning threshold",
                "source": "robot",
                "timestamp": 1745472000.888
            }
        ]
    }
}



```

##### subscribe\_health

协议示例；当前 Python 标准 runtime 的 gRPC transport 暂未注册该方法。

```json
// 请求
{"id": "14", "method": "subscribe_health", "params": {}}

// 响应
{"id": "14", "ok": true, "result": {"subscription_id": "2"}}

// 推送
{
    "type": "health",
    "subscription_id": "2",
    "data": {
        "timestamp": 1745472001.123,
        "msg": [
            {
                "level": "error",
                "code": "MOTOR_FAULT",
                "message": "Joint motor fault detected",
                "source": "driver",
                "timestamp": 1745472001.120
            }
        ]
    }
}



```

##### get\_resource\_info

```json
// 请求
{"id": "15", "method": "get_resource_info", "params": {"resource_id": "res_asset_8f3c1d2e"}}

// 响应
{
    "id": "15", "ok": true,
    "result": {
        "resource": {
            "resource_id": "res_asset_8f3c1d2e",
            "type": "file",
            "domain": "asset",
            "name": "base.STL",
            "format": "stl",
            "mime_type": "model/stl",
            "mode": "snapshot",
            "size_bytes": 2483912,
            "hash": "sha256:8f3c1d2e..."
        }
    }
}


```

##### list\_resources

```json
// 请求：列出服务端采集数据 episode
{
    "id": "16",
    "method": "list_resources",
    "params": {
        "domain": "data",
        "kind": "collection",
        "limit": 100
    }
}

// 响应
{
    "id": "16", "ok": true,
    "result": {
        "resources": [
            {
                "resource_id": "res_collection_snapshot_7bc91a2f",
                "type": "directory",
                "domain": "data",
                "name": "teleop_dataset_1781774672/episode_000001",
                "format": "directory",
                "mode": "snapshot",
                "size_bytes": 28192311,
                "created_at": 1781774672.012,
                "updated_at": 1781774690.448,
                "metadata": {
                    "kind": "collection",
                    "catalog_root": "collections",
                    "collection_id": "teleop_dataset_1781774672",
                    "episode_id": "episode_000001",
                    "task_prompt": "teleop demo",
                    "task_description": "Teleoperation data collection"
                }
            }
        ]
    }
}

// 请求：列出服务端日志资源
{
    "id": "17",
    "method": "list_resources",
    "params": {
        "domain": "log",
        "limit": 100
    }
}

// 响应
{
    "id": "17", "ok": true,
    "result": {
        "resources": [
            {
                "resource_id": "res_log_3ab22c1",
                "type": "file",
                "domain": "log",
                "name": "server.log",
                "format": "log",
                "mode": "live",
                "size_bytes": 18321,
                "metadata": {
                    "kind": "log",
                    "catalog_root": "logs",
                    "relative_path": "server.log"
                }
            }
        ]
    }
}


```

##### list\_resource\_entries

```json
// 请求
{
    "id": "18", "method": "list_resource_entries",
    "params": {
        "resource_id": "res_collection_live_7bc91a2f",
        "recursive": true,
        "limit": 100
    }
}

// 响应
{
    "id": "18", "ok": true,
    "result": {
        "entries": [
            {
                "resource_id": "res_file_3ab22c1",
                "type": "file",
                "domain": "data",
                "name": "collection_meta.json",
                "format": "json",
                "mode": "snapshot",
                "size_bytes": 1024
            },
            {
                "resource_id": "res_dir_51ce921",
                "type": "directory",
                "domain": "data",
                "name": "streams",
                "mode": "snapshot"
            }
        ]
    }
}


```

##### read\_resource

```json
// 请求
{
    "id": "19", "method": "read_resource",
    "params": {
        "resource_id": "res_archive_91a2f7bc",
        "offset": 0,
        "limit": 1048576
    }
}

// 响应
{
    "id": "19", "ok": true,
    "result": {
        "resource_id": "res_archive_91a2f7bc",
        "offset": 0,
        "next_offset": 1048576,
        "data": "UEsDBBQAAAAI...",
        "encoding": "base64",
        "eof": false
    }
}


```

##### snapshot\_resource

```json
// 请求
{
    "id": "20", "method": "snapshot_resource",
    "params": {
        "resource_id": "res_collection_live_7bc91a2f"
    }
}

// 响应
{
    "id": "20", "ok": true,
    "result": {
        "resource": {
            "resource_id": "res_collection_snapshot_7bc91a2f",
            "type": "directory",
            "domain": "data",
            "name": "episode_000002",
            "mode": "snapshot",
            "size_bytes": 184920331,
            "hash": "sha256:7bc91a2f..."
        }
    }
}


```

##### prepare\_resource\_archive

```json
// 请求
{
    "id": "21", "method": "prepare_resource_archive",
    "params": {
        "resource_id": "res_collection_live_7bc91a2f",
        "format": "zip"
    }
}

// 响应
{
    "id": "21", "ok": true,
    "result": {
        "resource": {
            "resource_id": "res_archive_91a2f7bc",
            "type": "archive",
            "domain": "data",
            "name": "teleop_dataset_1781751227_episode_000002.zip",
            "format": "zip",
            "mode": "snapshot",
            "size_bytes": 184920331,
            "hash": "sha256:7bc91a2f..."
        }
    }
}


```

##### start\_collection

```json
// 请求
{
    "id": "22", "method": "start_collection",
    "params": {
        "names": ["observation.camera_0.image", "observation.robot.joint_state", "action.robot.joint_position"],
        "collection_id": "teleop_dataset_1781751227",
        "episode_id": "episode_000002",
        "task_prompt": "Pick up the red cup and place it on the tray.",
        "task_description": "采集红色杯子抓取与放置任务数据",
        "max_duration": 300.0
    }
}

// 响应
{
    "id": "22", "ok": true,
    "result": {
        "collection_id": "teleop_dataset_1781751227",
        "episode_id": "episode_000002",
        "collection_resource": {
            "resource_id": "res_collection_live_7bc91a2f",
            "type": "directory",
            "domain": "data",
            "name": "episode_000002",
            "mode": "live",
            "created_at": 1745472000.123
        },
        "names": ["observation.camera_0.image", "observation.robot.joint_state", "action.robot.joint_position"],
        "task_prompt": "Pick up the red cup and place it on the tray.",
        "task_description": "采集红色杯子抓取与放置任务数据",
        "started_at": 1745472000.123
    }
}



```

##### stop\_collection

```json
// 请求
{"id": "23", "method": "stop_collection", "params": {}}

// 响应
{
    "id": "23", "ok": true,
    "result": {
        "collection_id": "teleop_dataset_1781751227",
        "episode_id": "episode_000002",
        "duration": 15.0,
        "per_name_counts": {
            "observation.camera_0.image": 450,
            "observation.robot.joint_state": 450,
            "action.robot.joint_position": 450
        },
        "collection_resource": {
            "resource_id": "res_collection_snapshot_7bc91a2f",
            "type": "directory",
            "domain": "data",
            "name": "episode_000002",
            "mode": "snapshot",
            "size_bytes": 184920331,
            "updated_at": 1745472015.123
        }
    }
}



```

##### get\_collection\_status

```json
// 请求
{"id": "24", "method": "get_collection_status", "params": {}}

// 响应
{
    "id": "24", "ok": true,
    "result": {
        "running": true,
        "collection_id": "teleop_dataset_1781751227",
        "episode_id": "episode_000002",
        "collection_resource": {
            "resource_id": "res_collection_live_7bc91a2f",
            "type": "directory",
            "domain": "data",
            "name": "episode_000002",
            "mode": "live"
        },
        "names": ["observation.camera_0.image", "observation.robot.joint_state", "action.robot.joint_position"],
        "counts": {"observation.camera_0.image": 150, "observation.robot.joint_state": 150, "action.robot.joint_position": 150},
        "duration": 5.0,
        "task_prompt": "Pick up the red cup and place it on the tray.",
        "task_description": "采集红色杯子抓取与放置任务数据"
    }
}



```

##### delete\_collection

```json
// 请求
{"id": "25", "method": "delete_collection", "params": {"resource_id": "res_collection_snapshot_7bc91a2f"}}

// 响应
{"id": "25", "ok": true, "result": {"deleted": true}}



```
---

### 数据类型约定

| 数据 | JSON类型 | 单位/格式 |
| --- | --- | --- |
| 关节角度 | float | rad |
| 位置坐标 | float | m |
| 四元数 | \[x, y, z, w\] | Hamilton |
| 图像 | object | `{width, height, encoding, image}`；`image` 为 base64 |
| 时间戳 | float | Unix秒（精确到μs） |
| 夹爪开合 | float \[0,1\] | 0=闭合, 1=张开 |
| 帧率 | float | Hz |
| 角速度 | float | rad/s |
| 力 | float | N |
| 力矩 | float | Nm |
| 压力(触觉) | float | Pa |
| 百分比 | float \[0,100\] | % |
| 温度 | float | ℃ |
