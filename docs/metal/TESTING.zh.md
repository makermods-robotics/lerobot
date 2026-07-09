# Metal 机械臂 — 分阶段硬件联调与测试（中文版）

> 软件（离线）部分已完成并用 mock 做了单元测试。下面的步骤需要**实体机械臂**,由操作者执行。
> 有几步会**使能力矩**,请清空作业区、随时准备断电。
> 英文原版:`TESTING.md`(代码/命令保持一致)。

## 0. 前置条件
- 已安装集成的环境:`pip install -e ".[metal]"`(会拉入 python-can + pinocchio)。
- 从臂接 **can0**,主臂接 **can1**。电机已供电(24 V),CAN 接线 + 120 Ω 终端电阻到位。
- 拉起两条总线(经典 CAN @ 1 Mbps):
  ```
  bash docs/metal/start_can.sh
  ```
  (若用 DM-USB2FDCAN USB 适配器走 slcan,而非原生 CAN 卡:改用 `slcand`,并在 config 里设 `can_interface="slcan"`。)

## 1. 从臂单关节 sanity(can0)
验证 CAN id 映射、使能、读回、以及一次下发的移动。
```
python docs/metal/tests/follower_smoke.py
```
预期:打印全部 7 个电机的观测,并看到 `joint6` 移动约 +3° 再回来。若 connect 报错 → 检查 CAN 是否已拉起、id 是否 `0x01..0x07`/`0x11..0x17`、供电、终端电阻。

## 2. 主臂重力补偿"悬浮"测试(can1)—— 关键检查
```
python docs/metal/tests/leader_float.py
```
**验收标准:机械臂手感失重,松手能停在原位。**
- 某关节**下沉**(往下漂)→ 重力被低估:调大 `src/lerobot/motors/metal/gravity.py` 里该关节的 `GRAVITY_COE`,和/或把**主臂把手质量**加进模型(把手目前不在 URDF 里)。
- 某关节**反向推开 / 跑飞** → **符号错**:该关节电机正方向与 URDF 关节轴方向相反。**不要继续跑**,先修符号。
- **失重但抖动/振荡** → 稍微调大 `leader_kd`(MIT 阻尼)。

## 3. 主臂 ↔ 从臂 零点对齐
teleop 是把主臂关节角(度)1:1 映射到从臂目标角(度),所以两臂在同一物理位姿下必须读数一致。
- 把两臂摆到同一位姿,逐关节对比 `get_observation()`(从臂)和 `get_action()`(主臂)。
- 若不一致,通过总线标定(`MotorCalibration`)给每个电机设 `homing_offset` 并持久化,直到两臂在同一参考位姿下读数相同。

## 4. 端到端遥操作
```
lerobot-teleoperate \
  --robot.type=metal_follower --robot.port=can0 \
  --teleop.type=metal_leader  --teleop.port=can1
```
预期:从臂跟随主臂;捏主臂夹爪会带动从臂夹爪。录一段短视频给上游 PR(最强社会信号)。之后用 `lerobot-record` 采一个数据集(见 `docs/source/metal.mdx`)。

## 已知调优项(从离线构建中延后)
- **从臂增益**:目前是 DamiaoMotorsBus 默认值(kp=10,kd=0.5)→ 跟随偏软。调向厂商"follow"增益(J1 200/3、J2 500/5、J3 400/5、J4 200/2、J5/J6 20/0.1 —— 来自 `y1_sdk .../config/motor_config.cpp`)。逐电机用 `bus.sync_write("Kp", {...})` / `bus.sync_write("Kd", {...})` 设置,信任绝对值前先确认 MIT kp/kd 量程和厂商一致。
- **夹爪单位**:目前按电机原始角度(度)直通(对同款双臂 teleop 是正确的)。要给数据集/策略暴露物理 0–100 行程,接入随包的非线性查表(`src/lerobot/motors/metal/gripper.py`)。
- **主臂把手质量**:尚未进重力模型;若第 2 步显示末端下沉,把它加进去(单独 URDF 或末端质量偏置)。
- **适配器/吞吐**:7 电机控制环优先用原生 SocketCAN;若用 USB 适配器,验证 slcan 吞吐是否够。
