# Metal 机械臂 ACT Rollout 笔记（本次会话总结）

> 记录时间：2026-07-23
> 涉及文件：`tools/rollout_towel_right.sh`、`tools/rollout_towel_right.json`
> 训练产物：`outputs/train/act_sort_towel_right/`

本文档总结本次会话围绕「用 metal 右臂跑通并调优 ACT 折毛巾策略」所做的全部工作、踩过的坑和结论。

---

## 1. 最新的 ACT 训练在哪

- **训练目录**：`outputs/train/act_sort_towel_right/`（2026-07-22 19:19 完成，训练到 20000 步）
- **策略权重（rollout 用这个）**：
  ```
  outputs/train/act_sort_towel_right/checkpoints/last/pretrained_model
  ```
  `last` 是软链，指向 `020000`；另有 005000 / 010000 / 015000 中间 checkpoint。
- **训练数据集**：`IsaacSinn/sort_towel_right_20260722_164806`
  - 实际只录了 **35 集 / 23317 帧**（配置写 100 集，提前停了）。对双相机 7 自由度任务来说数据偏少，是策略预测方差大的根因。
- **策略要点**：ACT + resnet18(ImageNet) + VAE；观测 = `observation.state`(7 维) + 双相机 `top`/`wrist`(3×480×640)；动作 7 维；`chunk_size=100`，`n_action_steps=50`，`temporal_ensemble_coeff=null`。

---

## 2. 录制配置（数据是怎么录的）

由 `tools/record_towel_right.json` 定义，`tools/record_towel_right.sh` 启动（脚本自动加时间戳后缀、支持 `resume`）。

- **Follower（`metal_follower`）**：`can0` + socketcan，`torque_feedforward: true`，带 `max_relative_target` 限幅和逐关节 MIT 增益。
- **相机**（都是 640×480@30）：
  - `top` → `/dev/video0`
  - `wrist` → `/dev/video2`（**必须 `fourcc: MJPG`**，否则退回 YUYV 掉帧）
- **Leader（`rebot_102_leader`）**：`/dev/star_right`，含 `joint_directions` / `joint_ranges`。
- **关键约定**：相机的 key 名（`top`/`wrist`）→ 数据集特征 `observation.images.top/.wrist`，**rollout 时必须逐字一致**，策略是按名字取张量的。

---

## 3. Rollout 配置对齐

用脚本做了程序化 diff，确认：

- `rollout_towel_right.json` 的 `robot` 块与 `record_towel_right.json` **逐字节相同**（相机、增益、限幅、`torque_feedforward` 全一致）。
- `fps=30`、任务字符串 `"Sort the towel"` 两边一致。
- 相机 key、`observation.state`/`action` 的 7 维都与 checkpoint 的 `config.json` 对得上。

结论：**rollout 与训练条件一致，可以直接跑。**

---

## 4. 让 ACT 更平滑：Temporal Ensembling

### 问题
`n_action_steps=50` 意味着机械臂一次开环执行 ~1.7 秒动作，然后硬切到「用 1.7 秒前的旧观测算出的新 chunk」。这个不连续就是抖动/顿挫的来源。

### 方案（**纯推理期设置，不用重训**）
开启 temporal ensembling：每个 tick 都推理一次，对所有覆盖当前时刻的 chunk 做指数加权平均，指令轨迹天然连续。

- 代码依据：`modeling_act.py:67` 读 `temporal_ensemble_coeff`；`rollout/configs.py:329` 会把 `--policy.*` 透传进 checkpoint 配置。
- 约束：`configuration_act.py:138` 强制 ensembling 时 `n_action_steps=1`（必须每 tick 推理），所以这俩是绑定开关。
- 系数：`0.01` 是 ACT 论文默认（偏重旧动作）；负值偏重新动作，但 `modeling_act.py:177` 提示这会削弱 chunking 收益。

### 成本
在本机 4090 + 本 checkpoint 实测：`select_action` 中位数 5.2ms / p95 5.3ms，预算 33.3ms（30fps），有 6 倍余量。
> ⚠️ 注意：该 benchmark 喂的是预置 GPU 张量，**未含**每 tick 的相机读取 + H2D 拷贝 + 归一化，真实回路会更慢（见第 7 节 FPS 问题）。

### 写进脚本
`tools/rollout_towel_right.sh` 默认开启，用 `TE_COEFF` 环境变量控制：
- `./tools/rollout_towel_right.sh` → ensembling，coeff 0.01
- `TE_COEFF=off ...` → 旧的 50 步 chunking（A/B 对照用）
- `TE_COEFF=0.0 ...` → 均匀加权

---

## 5. Rollout 时录制相机

`--strategy.type=base` **不录任何数据**（`rollout/configs.py:275` 传 `--dataset.*` 会直接报错）。要录相机需换策略，`episodic` 是最接近 `lerobot-record` 的（分集边界、方向键控制、集间复位）。

脚本里用 `RECORD=1` 切换：
- `RECORD=1 ./tools/rollout_towel_right.sh` → 录制
- `RECORD=1 EPISODES=10 ... last 30` → 10 集 × 30 秒
- 双相机自动都录（数据集特征从同一个 `robot.cameras` 派生，无需重复声明）。

**踩坑（都已修正）**：
- 数据集名**必须 `rollout_` 前缀**（`context.py:354`），否则报错。
- 不要手动加时间戳，`stamp_repo_id()`（`configs/dataset.py:75`）会在创建时自动加，手动加会双重时间戳。
- `episodic` 模式下 **`--duration` 无效**，时长 = `num_episodes × episode_time_s`（+ 复位时间）；第二个位置参数变成「每集秒数」。

> 提醒：rollout 录的是**策略自己的动作**，适合看效果/算成功率，但**不是干净的训练数据**（拿它训练 = DAgger 式模仿当前策略的错误）。要纠正数据用 `--strategy.type=dagger`。

---

## 6. 上传 Hub

脚本里用 `PUSH=1` 开启：
- `RECORD=1 PUSH=1 ...` → 上传（默认私有）
- `RECORD=1 PUSH=1 PRIVATE=false ...` → 公开

**踩坑**：
- **命名空间不匹配**：CLI token 认证身份是 `makermods`，但 repo_id 是 `IsaacSinn/...`。若 `makermods` 无 `IsaacSinn` 写权限，会在录完后上传阶段 403（数据不丢，只是没传上去）。需要时改 `REPO=makermods/rollout_...`。
- **代理坑**：本机 `ALL_PROXY=socks://127.0.0.1:7892/`，httpx 不认 `socks://`（要 `socks5://`），会让所有 `hf` 调用报 `Unknown scheme for proxy URL`。脚本在上传路径里只 `unset ALL_PROXY`（保留合法的 `http://` 的 `https_proxy`）。建议在 shell profile 里根治。

---

## 7. Rollout 回路比 FPS 慢（27.7 Hz < 30 Hz）

警告来自 `episodic.py:245`，**只在 `RECORD=1` 模式**出现（base 模式回路里没有建帧/写盘）。27.7Hz = 单 tick 约 36ms，超预算 ~3ms，不算严重。

**最可能的原因是第 4 节开的 ensembling**：之前 50 tick 才推理一次，现在每 tick 都推理，加上真实回路含双相机 H2D + 归一化，benchmark 低估了成本。

**零成本判定**：
```
RECORD=1 TE_COEFF=off ./tools/rollout_towel_right.sh
```
- 警告消失 → 是 ensembling，改用短 horizon chunking：`--policy.n_action_steps=10`（跳变更小，N tick 才推理一次）。
- 警告仍在 → 是相机/磁盘，ensembling 洗清嫌疑。

> 不要只在 rollout 时给 `top` 加 `MJPG`：训练用的是未压缩帧，只改推理会造成训练/推理观测分布不一致。要改就连 `record_towel_right.json` 一起改并重训。

---

## 8. CAN 连接失败 / SocketCAN 未正常关闭

### 现象
- `ConnectionError: Handshake failed. The following motors did not respond: [joint1..gripper]`
- `WARNING SocketcanBus was not properly shut down`

### 结论
- **主机侧完全正常**：`can0` UP，经 slcand 绑到 `/dev/ttyACM2`（序列号 `206735964543` = 右臂 CANable2，与 `tools/can_map.conf` 一致），无进程抢占。
- 实测被动监听 `can0` 2 秒 **零流量** → **总线上没有任何设备**，是**机械臂没上电或急停锁定**，不是软件问题。
- 「SocketcanBus was not properly shut down」是**红鲱鱼**：这是进程崩溃后 `__del__` 的抱怨。SocketCAN 是内核 fd，进程一退出（正常/崩溃/kill -9）内核就无条件回收，**没有残留状态要清理**。实测重新 open→listen→close 干净无误。

### 处理
1. 查急停是否锁定 → 查 24V 供电。
2. 都正常仍静默 → 断电重启 24V 清除电机故障锁存。
3. `slcand` 重启 / `ip link down/up` / 重启电脑都**没用**（问题不在软件）。
4. 电机层故障（崩溃后 torque 仍开/故障锁存）用 `tools/torque_off.py`，但需电机先响应，所以上电后才有用。

---

## 9. 关于 `--config_path` 和 metal 机器人配置

- `--config_path` 是**通用参数**：所有 draccus 系 lerobot CLI（`lerobot-record`/`-rollout`/`-train`/`-teleoperate`）都支持，指向 JSON/YAML，键名对应 CLI flag，**显式 flag 覆盖文件**。
- 「config.json」有两个别混：
  - `checkpoints/*/pretrained_model/config.json` = **策略**配置（ACT 超参）。
  - metal **机器人**配置无独立文件，就是 `--config_path` 文件里的 `robot` 块，schema = `MetalFollowerConfig` 数据类（`src/lerobot/robots/metal_follower/config_metal_follower.py`）。
- metal follower 关键默认值：`velocity_feedforward=true`（joint1 跟踪修复）、`startup_sync_speed_deg=1.0`（连接时缓慢就位而非硬拽）、`torque_feedforward=false`（需 pinocchio，默认关）。
- 相机条目**必须**含 `width`/`height`/`fps`，否则 `RobotConfig.__post_init__` 报错。

---

## 附：常用命令（从 `~/metal-lerobot` 执行）

**纯推理（平滑开，300 秒）**
```bash
uv run lerobot-rollout --config_path=tools/rollout_towel_right.json --strategy.type=base --inference.type=sync --policy.path=outputs/train/act_sort_towel_right/checkpoints/last/pretrained_model --policy.device=cuda --policy.n_action_steps=1 --policy.temporal_ensemble_coeff=0.01 --duration=300
```

**推理 + 录双相机（5 集 × 30 秒，仅本地）**
```bash
uv run lerobot-rollout --config_path=tools/rollout_towel_right.json --strategy.type=episodic --inference.type=sync --policy.path=outputs/train/act_sort_towel_right/checkpoints/last/pretrained_model --policy.device=cuda --policy.n_action_steps=1 --policy.temporal_ensemble_coeff=0.01 --dataset.repo_id=IsaacSinn/rollout_sort_towel_right --dataset.single_task="Sort the towel" --dataset.fps=30 --dataset.num_episodes=5 --dataset.episode_time_s=30 --dataset.reset_time_s=15 --dataset.push_to_hub=false
```

**用封装脚本（等价，且带上述所有开关）**
```bash
./tools/rollout_towel_right.sh                          # 纯推理，平滑开
TE_COEFF=off ./tools/rollout_towel_right.sh             # 关平滑（FPS 排查）
RECORD=1 ./tools/rollout_towel_right.sh                 # 录双相机
RECORD=1 PUSH=1 ./tools/rollout_towel_right.sh          # 录 + 上传 Hub
```

> 机械臂一启动就自主运动，手放急停上。
