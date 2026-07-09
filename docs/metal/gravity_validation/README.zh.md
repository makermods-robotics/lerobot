# 重力 / 科氏项验证 —— Pinocchio vs 厂商 KDL（中文版）

metal 机械臂的重力补偿以**纯 Python 的 Pinocchio**(`src/lerobot/motors/metal/gravity.py`)交付,替代了厂商 SDK 的 **KDL**(ROS)动力学。本目录用于证明这次替换在数值上是精确的:在相同输入下,Pinocchio 与"厂商完全相同代码路径的 KDL"给出的关节力矩一致到浮点精度。

> 英文原版:`README.md`(命令/代码保持一致)。

## 结果(300 组随机关节角,在软限位内,seed 0)

| 项 | 最大 \|误差\| | 平均 \|误差\| | 最大 \|信号\| | 误差 / 信号 |
|---|---|---|---|---|
| 重力 `g(q)` | 4.9e-11 N·m | 1.7e-12 | 14.08 N·m | ~3e-12 |
| 科氏 `C(q,q̇)q̇` | 4.9e-12 N·m | 1.5e-13 | 1.97 N·m | ~2e-12 |

残差是两边递归累加顺序不同带来的机器精度舍入 —— 比信号本身小约 12 个数量级。**结论:Pinocchio == 厂商 KDL。**

## 为什么两者一致
两者都实现了完全相同的 RNEA(递归牛顿–欧拉)算法,并读取**同一个** URDF 的惯量。已核对模型对齐:两者都用 6 个转动关节 `JOINT1..JOINT6`;夹爪作为固定质量灌入 `Link6`(不是独立分支),所以 KDL 的 `getChain("base_link","Link6")` 和 Pinocchio 的全模型是同一个系统;`fixed_base_joint` 是单位变换,因此重力 `(0,0,-9.81)` 在两边是同一个物理方向。厂商那套经验系数 `gravity_coe`/`coriolis_coe`/摩擦,是叠加在这个(已证明一致的)引擎输出之上的纯乘子。

## 文件
- `kdl_oracle.cpp` —— 厂商完全一致的 KDL 计算器:`kdl_parser::treeFromFile` → `getChain("base_link","Link6")` → `ChainDynParam::JntToGravity` / `JntToCoriolis`(与 `y1_sdk .../kdl_solver.cpp` 一致)。读入 `q[6] qd[6]` 行,输出 `gravity[6],coriolis[6]`。
- `validate.py` —— Pinocchio 侧 + 对比(在 `metal-lerobot` 环境里运行)。

## 复现
KDL 侧需要 ROS Humble + orocos-kdl(一次性;**不属于**交付包,也不进 CI)。在仓库根目录执行:

```
source /opt/ros/humble/setup.bash
g++ docs/metal/gravity_validation/kdl_oracle.cpp -o docs/metal/gravity_validation/kdl_oracle \
  -I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include -I/usr/include/kdl \
  -I/opt/ros/humble/include/urdfdom_headers -I/opt/ros/humble/include/urdf -I/usr/include/eigen3 \
  -L/opt/ros/humble/lib -lkdl_parser -lorocos-kdl -lurdf -Wl,-rpath,/opt/ros/humble/lib
python docs/metal/gravity_validation/validate.py   # metal-lerobot 环境,且 LD_LIBRARY_PATH 含 ROS 库
```

编译出的 `kdl_oracle` 可执行文件和临时 CSV 已被 git 忽略;仓库只跟踪源码 + 本结果文档。
