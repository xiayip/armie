# Armie v0.2：四种桌面互动

本版实现《Armie 游戏设计 v0.2》的软件流程：朝人静止待机，通过数字手势 1–4 选择玩法，结束后回正。默认启动 `game` 和 `senses` 两个节点，可选本地按钮面板。默认不启动 HMI 或表情节点。

| 数字 | 玩法 | 已实现的软件流程 |
|---|---|---|
| 1 | 软软模式 | 阻尼切换 → 拖动 → 结束 → 确认松手 → 静止确认 → 阻抗保持 → 从实测姿态回正 |
| 2 | 好奇小球 | 彩色圆球检测 → 有界视线跟随 → 球与相机均停稳 → 恢复桌面观察位 → 固定停靠区检查 → 抓取尝试或歪头试探 → 回正 |
| 3 | 水草舞 | J2/J3 主导整臂连续起伏，底座及腕部错相跟随，三轮波浪后回正；当前 Zephyr 1.6 倍速轨迹约 11.7 秒 |
| 4 | 看着我 | 寻脸 → 目标一致性校验 → 平滑跟随 → 丢失时冻结参考 → 超时/结束后回正 |

**已做软件、ROS 模拟服务器和 mock 集成验证，并开始 Zephyr 完整末端真机验证；实体抓球尚未验证。** 相机方向、姿态、桌面停靠区和抓取路径必须在实际装置上确认。真实运行不接受 mock 感知结果。

## 启动

宿主机源目录：`/home/xyp/code/zephyr_ws/src/armie`。

进入容器：

```bash
docker exec -it -u admin zephyr_dev_24.04-aarch64-container bash
cd /workspaces/zephyr-dev/zephyr_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch armie armie.launch.py panel:=true
```

这会打开本地按钮面板并运行 mock。手动按 1/2/3/4 选择玩法；mock 会在跟随模式生成明确标为模拟的小球/人脸观测，不会自行生成菜单手势。软软模式退出顺序为“结束并回正 → 确认已松手”。“暂停”只停止当前互动，不自动回正；关闭面板也会请求暂停。

“结束并回正”与 ghost game 的 `return_home` 一致：任何状态（包括暂停、抓球流程中途、已待机）都接受。它只取消 Armie 自己的动作，等待其终止和关节静止约 0.7 秒，先张开夹爪（手中有球时放回原处附近，而不是在朝人位掉落），再切阻抗链并回到朝人待机位。软软模式仍需先“确认已松手”。控制器预检因尚未静止而拒绝时会在 10 秒内自动重试；超时、模式切换结果不确定或动作无法停止时暂停并说明原因，不强行切换。正在回正时再次点击会被拒绝。

无显示环境时使用 `panel:=false`，从另一同环境终端控制：

```bash
ros2 topic pub --once /armie/command std_msgs/msg/String '{data: "2"}'
ros2 service call /armie/finish std_srvs/srv/Trigger '{}'
ros2 service call /armie/release std_srvs/srv/Trigger '{}'
ros2 service call /armie/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /armie/state
```

模拟使用域 187，与机器人正在使用的域隔离。实机运行须改为实际机器人域，且保证 Armie 是唯一运动控制者。

## 真相机 + 模拟机械臂

```bash
ros2 launch armie armie.launch.py senses_backend:=camera panel:=true
```

此时仍然不移动真实机械臂，但可以用真实手势选择模式，检查小球检测与人脸识别。相机需向 `/camera/color/image_raw/compressed` 发布带有效 ROS 时间戳的 `sensor_msgs/CompressedImage`。图像最长边缩放到 640；只处理最新帧，旧帧不补队列。过期图像不会触发动作。

- 数字 1：伸食指；2：食指+中指；3：再加无名指；4：四指伸出，拇指收起。
- 用手部关键点的关节角度/相对距离判断数字，不依赖原分类器是否有“3/4”类别。多只手不触发。真实手型识别仍需在腕部相机视角验收。
- 同一手势持续约 0.8 秒触发；放下手约 0.3 秒后才可再次触发。
- 只有待机态接受数字菜单，活动模式中忽略数字选择；返回后持续举着原手势也不会重触发。
- 小球默认橙黄色，HSV 范围在 `config/game.yaml` 中设置。多个相似圆球时不选取目标。
- `/armie/camera_preview/compressed` 输出检测标注图；`/armie/senses_status` 报告相机/模型状态。

已建立独立的 `.armie-runtime` 环境，安装现有项目固定版本的 MediaPipe 及其依赖；加载并试跑了官方、校验过哈希的手势模型和原项目 YuNet 人脸模型。没有替换系统 NumPy/OpenCV。启动文件在当前容器自动使用此环境；换工作空间时通过 `senses_python:=/path/to/python` 和配置文件调整模型路径。

## 实机准备和姿态记录

1. 启动原有 Rebot bringup，确认机械臂反馈、控制器与原阻抗轨迹 guard 正常。不要同时运行 ghost game 运动节点、旧 Armie pet、其他直控或轨迹发送程序。
2. 复制 `config/game.yaml` 为自己的配置。实机需要 `motion_backend:=trajectory motion_enabled:=true`，相机用 `senses_backend:=camera`。启动后先处于暂停态，不自动移动。
3. 在现有控制工具中将机械臂调整到朝人姿态。稳定后点击“记录朝人姿态”。同样记录“桌面观察位”。记录操作只读当前姿态，不驱动机械臂。
4. 真机记录保存到 `calibration_file`（默认工作空间内的 `armie_calibration.json`）。mock 不写这个文件，实机不接受没有 `source: measured` 标识的文件，避免把演示姿态当作标定。
5. 点击“准备 / 恢复”：验证并切入阻抗链，等待参考和实测姿态稳定，回到已记录的朝人姿态后开放菜单。
6. 跟随前确认腕部相机 yaw/pitch 方向与动作范围，设置 `servo_commissioned: true`。默认方向来自原 ghost game 配置，不能据此视为本次安装已验收。

配置入口示例（仅在以上准备完成后使用）：

```bash
ros2 launch armie armie.launch.py \
  config:=/absolute/path/game-real.yaml \
  motion_backend:=trajectory motion_enabled:=true senses_backend:=camera panel:=true
```

## 模式切换与运动边界

- 复用 `rebot_arm_hmi_bridge.core` 的纯控制器检查/规划函数，检查底层 MIT、完整控制链与未知接口占用者。这只是软件函数复用，不启动该桥、不访问 HMI/CAN。
- 调用现有 `/controller_manager/switch_controller`，使用 STRICT，并复查实际控制器状态。保留原阻尼、增益、重力补偿、限位和 guard 参数。
- 阻尼退出必须显式确认松手，并连续观察静止状态；阻抗控制器自身的激活逻辑从实测姿态初始化。下一条回正轨迹再次以实测姿态为起点。
- 人脸/小球跟随复用 ghost_game 的 `servo_target`、扫描边界和目标一致性检查；通过原阻抗控制器 `/rebot_arm_rs_impedance_controller/commands` 发送受限参考，进入/退出时交接上游 JTC。
- 跟随参考速度上限 0.2 rad/s，yaw/pitch 相对锚点范围分别为 0.35/0.25 rad。反馈过期或参考偏离实测持续超限时暂停。
- 轨迹通过 `FollowJointTrajectory` action；复用 ghost 的 minimum-jerk 时长计算，保留控制器默认容差。返回成功后还等待实测位置与速度稳定，才开始下一阶段。
- 只取消 Armie 自己的 action goal。切换服务无法取消；超时或结果不确定会锁定后续动作，需要确认实际状态后重启，不能自动回滚或继续。
- 状态检查不构成跨客户端原子锁；部署必须保证唯一运动控制者。暂停/结束按钮不是硬件急停。

## 小球抓取的范围

第一版采用**固定停靠区、停稳后抓取**，未实现任意桌面位置的三维抓取规划或高速拦截。

追视结束后先回到已记录的桌面观察位，再检查该固定视角下的 `dock_roi_json`。只看图像中的球不动是不够的，因此同时检查腕部关节稳定，避免把相机追着球移动误判成球已停稳。

默认 `grasp_enabled=false`：停靠区检查完成后做一次歪头试探并收回，结果为 `curious_attempt_only`。这不是成功抓球。

要接真实抓取，需要确认当前安装的是官方并联夹爪，并配置：

- `gripper_enabled=true`；接口为 `/rebot_arm_rs_gripper_controller/gripper_cmd`，类型 `control_msgs/action/ParallelGripperCommand`。
- `grasp_path_json`：三个已验证的六轴姿态，顺序为预抓取、抓取、小幅抬起。
- `grasp_enabled=true`；使用与实际球大小匹配的停靠位置和路径。

流程：打开 → 接近/下探 → 闭合 → 小幅抬起/放回 → 打开 → 撤回 → 回正。命令使用实际接口的归一化夹爪位置（0 开、1 关），速度为 0.15。未接入对“球确实被抓住”的独立确认，因此实机结果明确标记为 `grasp_attempt_unverified`；夹爪动作成功或 stalled 本身不等于抓球成功。mock 完整路径标记为 `simulated_grasp`。

## 接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/armie/command` | `std_msgs/String` | `1/2/3/4` 或 `soft/ball/dance/face`；亦支持下列服务同名命令 |
| `/armie/state` | `std_msgs/String` JSON v2 | mode、phase、message、round_id、outcome、mock、控制器状态；5 Hz |
| `/armie/observation` | `std_msgs/String` JSON v1 | 带源时间戳、session、sequence、round_id 的数字/人脸/小球观测 |
| `/armie/finish` | `std_srvs/Trigger` | 任何状态下结束并回正（同 ghost `return_home`）：先停自己的动作，张开夹爪，再回朝人位 |
| `/armie/release` | `std_srvs/Trigger` | 软软模式退出时确认已松手 |
| `/armie/stop` | `std_srvs/Trigger` | 暂停，不回正 |
| `/armie/ready` | `std_srvs/Trigger` | 经验证恢复到朝人待机 |
| `/armie/record_front`、`/armie/record_table` | `std_srvs/Trigger` | 记录当前稳定实测姿态 |

外部感知可配置 `senses_backend:=external` 并向 observation 话题发送 `source: external` 的新鲜观测。坐标 x/y 是图像中心归一化误差 [-1,1]，width/height 为相对图像宽高乘 2。不同轮次、过期或重复序号会被丢弃。实现方必须使用当前 state 的 round_id，不能将历史检测重放成实时观测。

## 编译与测试

```bash
colcon build --packages-select rebot_arm_hmi_bridge armie --symlink-install
source install/setup.bash
colcon test --packages-select armie --event-handlers console_direct+
colcon test-result --test-result-base build/armie --verbose
```

测试在域 187/188、LOCALHOST 发现范围运行。包含 mock 四模式闭环、数字手势规则、球检测、静止判定、模拟控制器服务/action、拖动后回正、超时锁定，以及安装后的 launch 与真实 Tk 按钮回调。GUI 测试需要可访问的 DISPLAY，窗口在测试期间隐藏。

旧版 VLM 单帧理解仍可单独运行 `ros2 run armie vision`，不参与实时视觉伺服。旧 `pet` / HMI 原型与 `legacy.launch.py` 仅保留用于回归；不要与本版同时运行。旧说明见 `docs/legacy-mvp.md`。

## 连续水草舞

手势 3 使用六轴连续波浪轨迹，保留每个采样点的速度和加速度，避免在各段
关节点停顿。起止位置回到朝人姿态，起止速度与加速度为零。J2/J3 只从
待机位向上抬起，不向 0 rad 软限位区域摆动；超出关节范围的姿态直接拒绝。

`dance_amplitude_scale` 范围为 0.25–1，`dance_speed_scale` 范围为 0.5–1.6，Zephyr 当前使用 1.6，幅度仍为 1.0。
Zephyr 完整幅度下，J2 约 0.20–0.60 rad，J3 约 0.20–0.79 rad；底座与三个
腕轴同步错相摆动。标准速度的峰值上限 0.4 rad/s、加速度上限 0.65 rad/s²，
先按该基准整段定时，再应用速度倍率，不截断各轴波形。半速轨迹约 35 秒，1 倍约 18 秒，
1.6 倍约 11.7 秒，另加回位及稳定确认时间。仅舞蹈采用独立速度包络
[0.65,0.6,0.6,0.65,0.65,0.65] rad/s、加速度上限 1.7 rad/s²；
普通动作仍保留原来的 0.4/0.65 上限。现有控制器 J2/J3 参考位置限速为
0.6 rad/s，因此未提供原幅度 2 倍速。刚度及控制器保护参数不变。
动作开始前需确认整个摆动范围及线缆余量。

测试同时验证 JTC 的采样点间五次插值极值，检查速度、加速度、软限位边界
及 action 消息中非零速度的完整传递。测试环境不接入真机域。

## Zephyr 完整末端的真机准备

`config/game-zephyr.yaml` 对应 OmniPicker + 自制法兰 + Gemini 305，使用
`zephyr_arm_*` 控制器名称。临时朝人待机位由操作者指定为
`[-0.157, 0.20, 0.20, 0, 0, 0]` rad；这是操作者确认的目标配置，不伪装成实测标定。
该配置保持跟随方向、桌面观察位和抓取未标定状态。

使用原 `zephyr_arm_bringup` 的组合 URDF 和控制器文件，以便重力补偿包含
整个末端的质量及质心；不通过增加通用倍率或复制位置模式系数来修改柔顺模式。
CAN 先初始化为 1 Mbps。下面第一条指令会使能硬件并保持当前姿态：

```bash
ros2 launch zephyr_arm_bringup bringup.launch.py \
  can_interface:=can0 bring_up_can:=false activate_trajectory_controller:=true
```

等原 bringup 完成、位置控制链保持且阻抗控制器为 inactive 后，在另一终端加载
本套末端的 Armie 保持参数，再启动游戏：

```bash
ros2 param load /zephyr_arm_impedance_controller \
  /workspaces/zephyr-dev/zephyr_ws/src/armie/config/zephyr-hold.yaml
```

这份局部文件只将阻抗刚度从 `[20,15,15,12,8,8]` 调为
`[20,45,65,16,16,20]`，用于整臂水草舞的跟随与结束保持。原重力补偿、阻尼、
力矩上限、软限位、轨迹保护及手动拖动模式都不变；不覆盖原控制器配置文件。
本次运行已分步加载，无需在当前使能保持时再次加载。

随后启动 Armie（初始暂停，使用外部感知避免首次调试自动手势触发）：

```bash
ros2 launch armie armie.launch.py \
  config:=/workspaces/zephyr-dev/zephyr_ws/src/armie/config/game-zephyr.yaml \
  motion_backend:=trajectory motion_enabled:=true senses_backend:=external
```

通过 `/armie/ready` 才会切换阻抗链并回到配置的待机位。原先 J2/J3 的
0 rad 在模型下限及 0.15 rad 软限位缓冲区内，保护会施加推回力，因此改为
0.20 rad。若当前实测值超出限位，Armie 仍拒绝生成回正轨迹，不自动
清零、放宽限位或夹紧测量值。

静止确认同时检查约 0.5 秒的位置变化和平均速度，避免把速度反馈的交替噪声
误判成持续运动；位移、漂移、原始速度突增、时间戳异常及反馈丢失仍会阻止
启动。原位置 JTC 只有位置反馈，因此该接口另结合独立 JointState 的静止证据。
阻抗 JTC 仍要求完整速度反馈，且参考与实测偏差不超过 0.03 rad。

原阻抗 JTC 配置 `interpolate_from_desired_state: true`。Armie 新轨迹从新鲜且
已稳定的控制器参考位置接续，避免将首个点拉回实测位置而产生反向运动及摩擦
补偿。仍保留实测限位检查和上述偏差门槛。控制器返回的失败原因会显示在游戏
状态中。除了上述 Armie 局部阻抗刚度，原控制器的力矩限制、软限位或跟踪
保护保持不变，现有 ghost game 和控制器源文件未修改。

动作完成后也会等待同一套参考/反馈静止条件，最长 5 秒；不能仅凭 action
成功就启动下一段动作或宣布重新待机。

2026-10-31 真机结果：原位置链的 5 秒低速回位通过。初始柔顺参数下，舞蹈
结束后 J3/J5/J6 的保持偏差会阻止后续动作；经小步验证并采用上述局部刚度后，
已通过“准备 → 待机 → 预设舞蹈 → 回位 → 再次稳定待机”的一次完整实机闭环。
这仅覆盖本套末端、新待机位及这段舞蹈，不是全工作空间或四种玩法验收。
抓球、视觉伺服方向和阻尼拖动后的完整回正尚未完成本轮真机验证。

同日升级为连续整臂水草舞后，半速（约 35 秒）和标准速度（约 18 秒）均已
完成真机“舞蹈 → 回位 → 稳定待机”闭环。标准速度的 J2/J3 最大跟踪误差
约 0.026/0.031 rad。该版本使用上面的局部刚度 `[20,45,65,16,16,20]`，
该次验证使用完整幅度与标准速度，手势 3 直接触发；当时 72 项软件测试通过。

当前 Armie 抓取接口只适配官方 reBot action；OmniPicker 使用不同控制接口，
此配置保持 `gripper_enabled=false`、`grasp_enabled=false`。机械负载补偿正常
包含夹爪，但不会把官方夹爪 action 误发到 OmniPicker。

2026-10-31 提速验证：按用户选择保留完整幅度，速度倍率改为 1.6，轨迹
约 11.7 秒。原幅度 2 倍速会超过 J3 原参考限速，因此没有启用。
本次 1.6 倍速真机完成舞蹈、回位和稳定待机，J2/J3 最大跟踪误差约
0.0248/0.0313 rad；刚度与原控制器保护参数未改变。76 项软件测试通过。


## 2026-10-31：限速 1.5 倍与绳结球观察跟随

当前 Zephyr 配置 `motion_limit_scale: 1.5`。普通轨迹样点校验包络从
0.4/0.65 提高到 0.6 rad/s、0.975 rad/s²；舞蹈包络从
[0.65,0.6,0.6,0.65,0.65,0.65] / 1.7 提高到
[0.975,0.9,0.9,0.975,0.975,0.975] rad/s / 2.55 rad/s²。
这只放宽提交上限，不会自动把舞蹈或回位轨迹加速，舞蹈仍为 1.6 倍。

`config/zephyr-tracking-limits.yaml` 是独立的底层限速覆盖文件：参考位置
变化率由 [1,0.6,0.6,1,1,1] 改为 [1.5,0.9,0.9,1.5,1.5,1.5]；
参考速度上限由 [1,1,1,1.5,1.5,1.5] 改为 [1.5,1.5,1.5,2.25,2.25,2.25]。
两组参数仅初始化时读取，不能用运行时 set 假装已生效。本次在原位置链保持
当前姿态时，备份阻抗与阻抗 JTC 的全部实际参数，依次卸载 inactive 的 JTC、
阻抗控制器，再用备份加覆盖文件加载为 inactive。前后比对仅上述两组限速变化，
JTC 参数不变。原控制器文件、刚度、力矩限制、软限位和异常停机未修改。
底层阻抗控制器没有单独的加速度参数；未改动未参与此路径的 MoveIt 参数。

图像跟随速度从 0.2 提高到 0.3 rad/s，新增 0.975 rad/s² 的指令加速度约束
（原直接跟踪路径没有加速度斜坡）。积分保留制动距离，目标丢失后减速，
工作区和参考偏差检查保留。实际轴的加速度仍受机械负载和控制器影响，不能
把参考指令的界限当作实测机械加速度保证。

用户已选当前画面中的绳结球。`ball_detector: template` 使用保存在
`config/rope-ball-template.png` 的选定目标外观，进行多尺度、多角度匹配。
低置信度、存在另一个相似目标、过期图像时不输出目标。不自动更新模板，避免
遮挡时把手或背景学成球；大幅外观变化或离开视野可能丢失，不能承诺高速滚球。
相机仍执行最新帧策略与时间戳检查。`preview_ball` 命令可在 paused 状态
显示识别框，不发运动。已有 HSV 检测作为可配置备选保留。

本次记录了用户摆放的真实桌面视角。J5/J4 各作 +0.04 rad、2 秒的小幅试动并
回原位，确认图像方向：J5 增量对应图像 x 增大，J4 增量对应图像 y 增大，
因此球跟踪两轴方向均为 -1。只有 `ball_servo_commissioned` 置为 true；
人脸模式的 `servo_commissioned` 仍为 false，不借用桌面标定放行人脸模式。

`/armie/ready_table` 与原准备过程共用控制器检查、姿态静止门限及轨迹适配，
直接进入记录的桌面视角，不先绕行朝人位。就绪后手势 2 启动小球跟随。
`ball_follow_only: true` 跳过停稳后的抓取尝试，持续观察最多 60 秒；原目标
丢失退出和跟踪偏差停机保留。夹爪仍不执行抓取。当前只转腕部对准球，
不做末端沿桌面平移或球的三维拦截。


本轮实测和修复记录：首次闭环将球的水平归一化偏差从约 0.15 降至 0.05，
随后匹配失效触发原有目标丢失退出；没有降低识别置信度门限来硬跟踪。已增加
围绕上次目标位置的细尺度、细角度搜索，并继续做全画面相似目标检查。
全量 83 项测试通过；后续局部搜索和桌面返回行为的 9 项针对测试通过，包含
尺寸/转角变化、目标消失、重复目标、速度/加速度及换向制动。
另外一次从朝人位进入桌面位时，J3/J4 保持误差约 0.030/0.034 rad，原 0.03
门限阻止了继续跟踪。使用原位置控制链 5 秒低速精确恢复后再进入阻抗保持，
未调高刚度或放宽到位门限。纯跟球模式结束/丢失后的返回目标现在为桌面位；
其余玩法的回位行为不变。

细化后的真机跟踪在最后约 20 秒窗口内获得 272 次有效观测，水平偏差
0.315 → 0.071，末次外观匹配分数约 0.811；J4/J5 最大跟踪误差约
0.0152/0.0362 rad，未触发原异常保护。此结果不代表快速滚球或最高限速验收。

## 低延迟视觉与短时预测

相机订阅只保留最新一帧，视觉工作线程仍只有一个，完成轮询周期为 5 ms。
菜单手势保持原来的处理节奏；球观察尽可能跟随相机帧率。预览单独限制到
10 Hz，不为每次识别重复压缩图片。观测保留原始拍摄时间，附带
`timing.processing_ms` 和 `timing.capture_age_ms`，不能用接收时间掩盖延迟。
模式或轮次变化后丢弃旧任务的结果，图像重复或倒序时不重复处理。

模板全局搜索先用低分辨率选变换，再在原跟踪分辨率计算接受分数；局部高分
匹配只搜索平移，周期性修正尺度、角度。选中后用双向光流和 RANSAC 估计帧间
运动，检查内点、缩放、位移、固定颜色分布和纹理。定期重新验证选定模板，
光流不能无限保持身份；失配后重新识别。固定模板不从手或背景自动学习。
这里的 `score` 在 template 模式是相关系数，在 flow 模式是通过所有检查后
赋予的保守质量值，并非可校准的概率。

球目标按拍摄时间过期：120 ms 后不再用于跟随，明确无目标则立即撤销预测，
交给已有加速度约束减速。配置 `ball_prediction_enabled` 可独立关闭预测；
`ball_prediction_lead_sec` 默认 0.04 s。预测使用拍摄间隔估算图像运动，
累计至少三次连续观测后补偿图像年龄并增加短时提前量，总时域最多 0.12 s，
位移最多归一化 0.10（横向单轴相当于画面宽度的 5%）。反向、突跳、低质量或
失去目标重置估计。图像运动包含相机自身运动，因此这不是球的世界坐标速度，
也不能用于三维拦截或抓取。原机械臂限速、加速度、工作区与异常保护保持不变。

这仍是一天 MVP 的选定物体跟踪：绳结球大幅翻转、遮挡后以另一侧重新出现，
单个参考外观可能不能重识别。视觉频率、有效检出数和真实目标准确率是三个
不同指标，不能用输出频率代替快速滚球真机验收。

本轮验证：编译成功，96 项测试通过。原版 12 s 实拍输出 10.08 Hz、拍摄到
观测接收延迟中位数 165.9 ms；新版 12 s 滚球实拍输出 30.0 Hz，中位数
60.0 ms、P95 69.9 ms。新版静止目标窗口 300/300 帧有效。滚球窗口有
175/360 帧输出目标，但含离开视野、手取球及夹爪遮挡，不能当作准确率。
抽查录像显示一次穿越桌面中央的滚动被连续检测；靠近夹爪时发生丢失。
同一旧录像重放，单帧处理耗时中位数 80.9 → 23.9 ms；两版有效输出
127 → 130 帧，大幅外观变化后的重识别仍受单模板限制。
本轮机械臂保持静止，仅验证视觉和离线预测逻辑；没有把这些数据宣称为
机械臂在快速滚动下的闭环跟随或预测收益验收。

2026-10-31 夹爪保持张开：用户要求减少遮挡。实测确认现用接口为
`/zephyr_arm_gripper_controller/gripper_cmd`，关节 `omni_picker_finger_joint`，
归一化 0 为全开、1 为全闭。以速度 0.15、力 0.4 打开，动作成功且新鲜反馈
到达 0.0，相机确认中央视野扩大。Zephyr 配置启用 `gripper_open_only`，
进入球模式先完成打开动作再去桌面跟踪；打开失败则暂停。游戏拒绝闭合命令，
并禁止同时开启抓取。原控制器持续保持开口参考，不反复发送动作或改写底层保护。

2026-10-31 准备/回位提速：Zephyr 设置 `pose_transition_speed_scale: 2.0`。
回 ready 位、`ready_table` 和进入球观察位使用 0.24 rad/s（原 0.12）的
五次多项式峰值速度，最低段时长由 1.0 改为 0.5 s。保留 0.2 s 起始保持；
按记录的 ready/桌面位计算，单程 13.503 → 6.851 s。普通 `Robot.move`
现在还显式约束段内插值的峰值加速度，必要时延长短段；不是仅检查端点的
零加速度。Zephyr 仍为 0.975 rad/s²，不提高底层保护限值。
舞蹈、视觉跟随、人脸初始观察和抓取路径不使用这个提速倍率。

提速前排查：最近两次 JTC 日志均返回 Goal reached, success；游戏随后报
`post-motion settling failed: fresh settled JTC reference and feedback required`。
采样时 J3/J4 参考与反馈差约 0.03030/0.03106 rad，超过游戏 0.03 rad 的
准备检查，因此不能把暂停认定为 JTC 动作失败，也不能声称提速会解决它。
保留到位检查，并在本次软件加载后保持暂停，未自动发送新的真机准备动作。

2026-10-31 按用户要求放宽到位门槛：新增只读参数
`settled_position_tolerance`，默认仍为六轴 0.03 rad；Zephyr 配置为
[0.03,0.03,0.04,0.04,0.03,0.03]，只放宽此前卡住的 J3/J4，与这两轴原
JTC goal 容差相同。参数限定每轴 0.01..0.05 rad；新鲜度、静止时间窗、
速度检查、已有轨迹守护与软限位继续保留。轨迹仍从稳定的参考位置续接。
用户这次 ready 失败的消息是模式切换前静止证据不足，并非 JTC action 失败；
因此实测先等待静止检查通过，没有绕开该检查。加载后以 2 倍准备速度实测
回 ready 成功，含模式/到位检查耗时 7.762 s，进入 idle；末次最大关节偏差
约 0.02167 rad。30 项相关测试通过，另外重新运行的逐轴边界检查也通过。

## Dynamic ball workflow (nominal TCP trial)

`game-zephyr.yaml` now selects `ball_full_sequence: true`: gesture 2 enters the
usual table view and position tracking. Synchronized RGB-D and timestamped TF
measure ball motion in `base_link`. After 3 seconds within 1 cm, tracking stops.
The game checks the position controller, an existing `armie_observed_table`
collision object, and a complete collision-aware Cartesian path before starting.
It then executes approach → look left → look right → pregrasp → grasp → close →
lift. Every Cartesian segment is replanned from current measured joints, preserves
all MoveIt samples, and is slowed to at most 0.04 rad/s and 0.12 rad/s². Arrival
must settle within 0.01 rad before the next stage; closing happens only after
arrival at the grasp point.

After lifting, the game holds for `grasp_verify_sec` and decides from **two
independent sources**: the `omni_picker_finger_joint` position must stop between
`grasp_min_finger_position` and `grasp_empty_finger_position` (an empty claw closes
fully), and at least three fresh camera frames must no longer localize the ball
within 2 cm of its table position. Missing finger feedback or too few frames count
as a miss — motion completion alone never counts as success.

- Success (`grasp_success`): short show (`grasp_show_sec`), set the ball down 1 cm
  above where it was taken, open, back off to pregrasp, return to the start view,
  switch to impedance, return to the front pose.
- Miss (`grasp_missed`): open at the lift point, back off to the start view, switch
  to impedance, a small puzzled J6 head tilt, return to the front pose.

Mock runs report `simulated_grasp`; set `mock_grasp_success:=false` to exercise
the miss branch. The finger thresholds are defaults, **not calibrated against the
real ball**, and the place/retreat path has not been run on hardware.

Missing/stale target, >1 cm target movement, missing table geometry, incomplete
paths, action rejection/failure, stale feedback, excessive following error, or
operator stop abort the sequence (paused, no automatic return or gripper opening).
Only owned actions are cancelled. "结束并回正" is the explicit operator request
to go home from that state; it opens the claw first, then returns. An outstanding
action acceptance or cancellation keeps the workflow busy and prevents a new mode
from starting.
After confirmed closing, lift no longer requires visible ball depth because the
ball should move with the gripper; controller and collision checks remain active.

The current 142 mm TCP is **not physically calibrated**. The Zephyr profile
explicitly enables `allow_nominal_tcp_trial`; this does not waive collision or
freshness checks. `grasp_enabled: true` and `gripper_open_only: false` enable
closure in this profile. `motion_enabled` still defaults to false and hardware
starts paused. Merely building or editing this package does not execute the robot.
The generic profile retains its previous behavior.

Existing camera, robot bringup and table collision geometry must be available.
Start the MoveIt planning-only server separately, or use the optional
`grasp_planning:=true` launch argument (do not start a second server if one exists).
This does not automatically create a table map: if the collision scene is missing,
the workflow stops. Restart the game to load the changed configuration; do not
restart a holding robot's controller stack to reload game code.

Mock validation (no robot domain):

```sh
ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch armie armie.launch.py \
  config:=/workspaces/zephyr-dev/zephyr_ws/install/armie/share/armie/config/game-zephyr.yaml \
  motion_backend:=mock senses_backend:=mock motion_enabled:=false
```

The mock runs the same stage order and cancellation logic with simulated geometry
and motion. It is not evidence of real reachability or collision clearance.
Template reacquisition now searches 0.7–2.0× scale, retaining original appearance,
confidence, and ambiguity checks. Large searches occur on acquisition; established
tracks continue using local refinement/optical flow.
