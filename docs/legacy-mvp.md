# Armie：Rebot Arm 桌面宠物 MVP

独立 ROS 2 Jazzy / ament_python 包。默认纯 mock，不启动机器人、不切换控制器、不访问外部模型或 HMI 硬件。真实动作仅由明确的 `greet/nod/tilt` 命令触发；休眠/唤醒只有表情变化。视觉结果只用于表情和文案，不能生成动作。

## 已确认接口（2026-10-31）

- Docker：`zephyr_dev_24.04-aarch64-container`。
- 容器工作空间：`/workspaces/zephyr-dev/zephyr_ws`；宿主机：`/home/xyp/code/zephyr_ws`。
- ghost_game：`src/zephyr_mission/ghost_game`。轨迹通过 `control_msgs/action/FollowJointTrajectory` 发送；采用端点零速度/加速度。Armie **直接导入并复用**其纯函数 `ghost_game_orchestrator.mock_trajectory.minimum_quintic_duration`，没有启动游戏节点或复制游戏控制逻辑。
- ghost_game 默认 `zephyr_arm_*`；Rebot RS 实际配置在 `src/zephyr_common/rebot_arm_rs_ros2/rebot_arm_rs_bringup/config/controllers.yaml`，使用 `rebot_arm_rs_*`。Armie 默认接阻抗 JTC 的 `/rebot_arm_rs_impedance_trajectory_controller/follow_joint_trajectory`。
- 现有阻抗轨迹 guard 为 `/rebot_arm_rs_impedance_trajectory_guard`，原配置阻塞窗口为 3 秒。Armie 不改它，也不写增益、重力补偿、限位或切换服务。
- BT：`src/zephyr_mission/zephyr_bt/config/bt_executor.yaml` 的 `/behavior_server`，类型 `btcpp_ros2_interfaces/action/ExecuteTree`，请求 `target_tree` / `payload`。当前 executor 不使用 payload；默认树目录是 z2，含控制器切换及固定姿态，不能当 Rebot 宠物动作直接运行。`bt_trees_json` 默认空，必须填写已单独验证的树 ID；本包不注册、不执行现有 Z2 树。
- 腕部相机依据 ghost_game 感知配置使用 `sensor_msgs/msg/CompressedImage`：`/camera/color/image_raw/compressed`。支持 JPEG/PNG 压缩输入，转成最长边 640 的 JPEG 后送 VLM。不依赖当前容器存在 NumPy ABI 冲突的 cv_bridge。
- HMI：`arm_hmi_node` / `reBot_Arm_HMI` 与 `rebot_arm_hmi_bridge` 已实现 CAN v1 `0x6F0..0x6F2`，内容为模式切换/回执/状态，**未发现表情显示命令**。Armie 不占用这些 ID、不打开串口、不触发 DTR/RTS、不烧录固件。`/armie/hmi/tx` 与可选 UDP 是待板端接入的新显示协议，不是现有板卡的可用协议。
- 检查时当前 ROS 域只有系统话题，没有运行中的机器人/相机。以下测试不构成实机运动、板卡显示或外部 VLM 验收。

## 编译与运行

进入现有开发容器后：

```bash
cd /workspaces/zephyr-dev/zephyr_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select armie --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch armie armie.launch.py
```

另一个终端使用同一环境与域：

```bash
export ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /armie/state
ros2 topic pub --once /armie/command std_msgs/msg/String '{data: greet}'
ros2 topic pub --once /armie/command std_msgs/msg/String '{data: look}'
ros2 service call /armie/stop std_srvs/srv/Trigger '{}'
ros2 service call /armie/wake std_srvs/srv/Trigger '{}'
```

有效命令：`greet`、`nod`、`tilt`、`look`、`sleep`、`wake`、`stop`。MVP 中 greet 与 nod 复用同一小幅点头。不会自动抓取、回零或空闲随机运动。

```bash
colcon test --packages-select armie --event-handlers console_direct+
colcon test-result --test-result-base build/armie --verbose
```

## 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/armie/command` | `std_msgs/String` | 上述有限命令 |
| `/armie/state` | `std_msgs/String` JSON v1 | 状态、表情、动作阶段；保留最新值，1 Hz 心跳 |
| `/armie/expression` | `std_msgs/String` JSON v1 | `schema_version, sequence, expression, text, ttl_ms` |
| `/armie/look` | `std_srvs/Trigger` | 异步截图请求；success 表示受理，message 为 request_id |
| `/armie/perception` | `std_msgs/String` JSON v1 | `request_id, source, reason, scene`；source 为 mock/http/fallback |
| `/armie/snapshot/compressed` | `sensor_msgs/CompressedImage` | 本次 HTTP 识别使用的原始帧；不持久保存 |
| `/armie/stop`, `/armie/wake` | `std_srvs/Trigger` | 暂停互动并请求取消自己的动作 / 恢复互动 |
| `/armie/hmi/tx` | `std_msgs/String` JSON | 显示协议包；mock 也输出，可用于开发板端 |
| `/armie/hmi/status` | `std_msgs/String` JSON | 传输状态、超时、固件未实现标识；UDP 发送不代表已显示 |

`scene` 严格字段：`summary`（≤240 字）、`objects`（≤12 个短字符串）、`person_present`（布尔）、`mood`（neutral/happy/curious/confused）、`confidence`（有限数 0..1）。多余字段、非 JSON、NaN、超长结果都会 fallback。mock 和 fallback 的 confidence 都是 0，不代表真实观测。

## 配置真实视觉

复制 `config/armie.yaml` 到自己的配置文件，设置 `vlm_endpoint` 为完整 chat-completions HTTP URL、`vlm_model`，通过环境变量 `ARMIE_VLM_API_KEY` 提供可选密钥。启动时显式传 `vision_backend:=http config:=/绝对路径/配置.yaml`。请求格式为常见的 `messages + image_url + response_format=json_object`；后端须支持此格式。没有绑定模型供应商，没有实际外发任何图像进行本次验证。

只有请求 look 才发送一张新鲜帧。相机缺帧、过期、无效编码、HTTP 故障、响应格式不符或超时均产生标识为 fallback 的结果。网络请求在单独工作线程执行；同一时刻只允许一个请求，超时结果不会晚到覆盖当前表情。若网络工作线程迟迟不退出，后续 look 返回 busy，不会积压请求。参数为启动时只读，修改后重启对应节点。

## 实机动作接入边界

先独立启动并确认现有 Rebot bringup、阻抗链及 guard，确保该控制器的动作输入由 Armie 独占、ghost game/BT/其他运动客户端没有同时运行。Armie 仅观察控制器，不自动从位置/阻尼模式切换。需要显式 `motion_backend:=trajectory motion_enabled:=true`；配置选择本身不发送动作。

动作以新鲜的六轴测量值为锚点，joint4 点头 0.10 rad 或 joint6 倾斜 0.12 rad，再返回锚点。采用 ghost 的最小 jerk 时长计算，参考峰值速度 ≤0.12 rad/s，端点零速度/加速度。发送前检查已知限位、静止反馈、现有 guard 节点、阻抗控制器链、action 状态和直接轨迹发布者。还订阅 JTC 的 `controller_state`：要求参考/反馈均静止、相差 ≤0.03 rad、连续满足 0.5 秒，且状态发布者匹配至少 1 秒；避免从旧参考位置插值。反馈时效为 0.5 秒，动作期间反馈丢失会请求取消。保留控制器自身的默认目标容差。未进行碰撞模型/负载/末端 HMI 安装验证，因此这些检查不能替代首次实机调试。

`stop` 只请求取消 Armie 自己的 action goal，**不是急停**。发送 goal 与 stop 并发时会等到接受回执后取消；取消未确认、结果未知时保持 busy，拒绝后续动作。不会取消其他客户端目标或为了解锁而切控制器。动作状态检查与发送之间不是跨客户端原子锁，部署必须保证单一运动控制者。关闭节点前先调用 stop 并在 state 中确认 `motion_busy=false`；退出时仅尽力取消，进程被杀不保证停止。

BT 模式使用同样的实机开关和前置检查。只接受 greet/nod/tilt 到已验证树 ID 的显式映射；树自身必须负责其姿态边界及取消行为。默认没有可供实机执行的宠物 BT 树，不能直接填入原 Z2 树绕过确认。

## HMI 显示适配

mock 默认输出 ROS JSON。`hmi_transport:=udp` 可把同一 JSON 发到配置的主机/端口（默认 127.0.0.1:8766），用于屏幕模拟器或后续网关。每个 UDP datagram 是 UTF-8 JSON：

```json
{"protocol":"armie-face","version":1,"sequence":1,"face_id":1,"expression":"happy","text":"我在呢！","ttl_ms":2500}
```

face_id：0 idle、1 happy、2 curious、3 thinking、4 sleepy、5 confused、6 stopped。板端/网关应只把这些字段解释为显示指令，TTL 到期恢复断连脸。适配层丢失 expression 心跳后输出断连状态。现有 HMI 固件仍需增加独立表情页面与传输接收端；没有在本轮改动板端项目。

## 一天 MVP 范围

已实现的演示顺序：唤醒 → 打招呼/点头/歪头 → 看一眼 → 结构化理解改变表情/文案 → 休眠。先用 mock 完成接口闭环，再接真实相机/VLM，最后单独验证 Rebot 小幅动作与 HMI 显示。暂不做抓取、自由运动生成或根据 VLM 自动运动。
