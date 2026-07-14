# Dobot CR5 + Quest 数据采集部署手册

这份手册覆盖 Vision-Thinking fork 里的 CR5 适配层。当前实现不依赖 ROS2：

- Quest/BeaVR App 发送手部 keypoints 到后端。
- BeaVR 后端把 keypoints 转成 end-effector Cartesian target。
- `DobotCR5Robot` 通过 Dobot Dashboard TCP `29999` 直连 CR5 控制器。
- LeRobot 录制器从 BeaVR state topic 采集 observation/action。

当前真机运动只下发 XYZ 平移，使用 `RelMovLUser`。Quest/BeaVR 产生的姿态 quaternion 会被记录到 action，但暂时不会下发给 CR5。

## 仓库

```bash
git clone --recurse-submodules https://github.com/Vision-Thinking/beavr-bot.git
cd beavr-bot
git checkout vision-dobot-cr5
```

如果已经 clone：

```bash
git remote -v
git pull
git submodule sync
git submodule update --init --recursive beavr-app
```

`beavr-app` 是 Meta Quest 端 Unity App，上游来自 `ARCLab-MIT/BeaVR-app`。

## 网络拓扑

右臂推荐部署在 SSH config 里的 right 服务器：

```text
Quest headset  ->  right server: 192.168.1.145
right server   ->  Dobot CR5 controller: 192.168.5.2:29999
```

Quest App 里输入的是后端服务器 IP，也就是 `BEAVR_HOST_ADDRESS=192.168.1.145`。不要输入 `192.168.5.2`，那是 right 服务器内侧网卡能访问的 CR5 控制器地址。

常用端口：

| 方向 | 端口 | 用途 |
| --- | ---: | --- |
| Quest -> backend | 8087 | 右手 keypoints |
| Quest -> backend | 8110 | 左手 keypoints |
| Quest -> backend | 8095 | 分辨率/按钮 |
| Quest -> backend | 8100 | pause 输入 |
| BeaVR internal | 10109 | CR5 end-effector command/reset |
| BeaVR internal | 10110 | CR5 current pose |
| BeaVR internal | 10116 | CR5 state for recorder |
| backend -> CR5 | 29999 | Dobot Dashboard command |

## 后端环境

在 right 服务器上：

```bash
ssh dobot-cr5a-right-computer
git clone --recurse-submodules https://github.com/Vision-Thinking/beavr-bot.git
cd beavr-bot
git checkout vision-dobot-cr5

uv python install 3.10.13
uv venv --python 3.10.13
source .venv/bin/activate
uv sync --extra dev
```

准备 CR5 环境变量：

```bash
cp configs/dobot_cr5_right.env.example .env.dobot_cr5_right
source .env.dobot_cr5_right
```

默认 `DOBOT_DRY_RUN=1`，不会连接真机。

## Dry-Run 验证

先确认 BeaVR CR5 配置能启动：

```bash
source .venv/bin/activate
source .env.dobot_cr5_right
export DOBOT_DRY_RUN=1
./src/beavr/scripts/dobot_cr5_right_teleop.sh
```

录制流程 dry-run：

```bash
source .venv/bin/activate
source .env.dobot_cr5_right
export DOBOT_DRY_RUN=1
NUM_EPISODES=1 EPISODE_TIME_S=10 VIDEO=false PUSH_TO_HUB=false \
  ./src/beavr/scripts/dobot_cr5_right_record.sh
```

## 真机启动

真机前确认：

- 示教器/控制器处于远程 TCP/IP 可控制状态。
- `29999` 没有被 Dobot ROS2 bridge、官方 SDK 或其他 Dashboard 客户端占用。
- 机械臂周围清空，操作者站在急停旁。
- 先把 `DOBOT_MAX_STEP_MM` 和 `DOBOT_WORKSPACE_RADIUS_MM` 保持默认小范围。

连通性检查：

```bash
nc -vz 192.168.5.2 29999
```

只启动 teleop：

```bash
source .venv/bin/activate
source .env.dobot_cr5_right
export DOBOT_DRY_RUN=0
export DOBOT_ENABLE_GRIPPER=0
./src/beavr/scripts/dobot_cr5_right_teleop.sh
```

启动数据采集：

```bash
source .venv/bin/activate
source .env.dobot_cr5_right
export DOBOT_DRY_RUN=0
export DOBOT_ENABLE_GRIPPER=0
DATASET_REPO_ID=vision-thinking/dobot_cr5_quest_test \
DATASET_ROOT=data/dobot_cr5_quest \
NUM_EPISODES=10 \
EPISODE_TIME_S=30 \
RESET_TIME_S=5 \
VIDEO=false \
PUSH_TO_HUB=false \
./src/beavr/scripts/dobot_cr5_right_record.sh
```

`PUSH_TO_HUB=false` 是默认值。确认数据字段和 episode 正常后，再改为上传。

## Quest 端

安装方式：

1. 在 Meta Quest 手机 App 里启用 Developer Mode。
2. 安装 Meta Quest Developer Hub 或 Android platform tools。
3. 如果已有 BeaVR APK，用 `adb install -r path/to/BeaVR.apk` 安装。
4. 如果没有 APK，打开 `beavr-app/BeaVR-Unity`，用 Unity 6.2 构建 Android/Quest：
   - Platform: Android
   - Scripting Backend: IL2CPP
   - Target API Level: 32+
   - XR Provider: OpenXR
   - Build and Run

Quest App 设置：

1. Quest 和 right 服务器在同一 WiFi/LAN。
2. 启动 BeaVR App。
3. 在 VR 键盘里输入后端服务器 IP：`192.168.1.145`。
4. 连接成功后 UI 会显示当前 IP，连接状态变成功。
5. 左手 index-thumb pinch 开始 teleoperation。

## 调试

`Connection refused` 或提示端口占用：

- 检查 `29999` 是否被其他客户端占用。
- 停掉 Dobot ROS2 bridge 或官方 SDK 进程。

Quest 连接失败：

- Quest App IP 必须是后端服务器 IP，例如 `192.168.1.145`。
- 确认 Quest 和服务器在同一网络。
- 确认服务器防火墙没有拦截 `8087/8095/8100`。

后端启动但真机不动：

- 确认 `DOBOT_DRY_RUN=0`。
- 确认示教器允许远程 TCP/IP 控制。
- 确认目标没有超过 `DOBOT_WORKSPACE_RADIUS_MM`。
- 当前实现不会自动 home，reset 只重新获取当前 TCP pose 作为 teleop baseline。

运动方向不对：

- 先停真机，回到 dry-run。
- 调整 `src/beavr/teleop/components/operator/robots/dobot_cr5_operator.py` 里的 `H_R_V_RIGHT` 和 `H_T_V_RIGHT`。

## 数据字段

`dobot_cr5_only_adapter` 当前记录：

- `observation.state`: 6 轴关节角，单位 rad。
- `action`: `[x, y, z, qx, qy, qz, qw]`，位置单位 m。
- `dobot_cartesian_states`: 当前 TCP 位置。
- `commanded_cartesian_state`: 最近一次 operator/policy 命令。
- `gripper_state` 和 `error_state`: 如启用抓手/错误读取则发布。

如果后续要训练需要图像，给 `DobotCR5OnlyAdapterConfig.cameras` 增加 OpenCV/RealSense camera config，再把 `VIDEO=true` 打开。
