# Day 2 末端动力学辨识、阻抗/导纳控制与力控接触实验报告

## 0. 证据状态与禁止夸大表述

本报告当前交付的是 Gazebo/MoveIt 仿真链路、几何聚合得到的夹具惯量参数、阻抗/导纳控制计算程序和 topic 录制方案。尚未完成真机动力学辨识、真实六维力传感器标定、真实重力补偿保存/恢复、夹持力阈值标定和真实轨迹误差测量。

因此现场汇报时不要说“已完成真机动力学辨识”或“力控精度已达到 +/-1 mm”。可以说“已完成仿真用夹具质量/质心/惯量参数配置和控制算法验证；真机参数需要按本报告流程现场实测后替换”。

## 1. 实验目标

Day 2 用于展示右臂末端夹具在 Gazebo 中具备明确的重力、质量和转动惯量配置，并在阻抗/导纳控制框架下完成两个现象。阻抗部分用于解释“碰到固体后力如何随位移变化”，导纳部分用于解释“受到外力后机械臂如何产生运动响应”：

| 视频现象 | Gazebo 画面 | topic 证据 |
| --- | --- | --- |
| 机械臂碰固体后的受力变化 | 右臂末端接触 `d2_force_target_panel`/`d2_contact_pad` | `/right_rm_driver/rm_driver/udp_six_force`、`/force_control/gazebo_contact_force_state` |
| 机械臂受到力后的运动 | 右臂在导纳修正后产生额外关节响应 | `/force_control/admittance_offset`、`/force_control/corrected_right_joint_states`、`/joint_states` |

### 1.1 Gazebo 画面怎么读

Day 2 的场景可以按一个简单抽象理解：右臂做“按压弹性垫”的力控实验。

| 画面物体 | 含义 |
| --- | --- |
| 黑色竖板 `d2_force_target_panel` | 刚性墙/固定工装，是被按压的背景结构 |
| 蓝色方块 `d2_contact_pad` | 真正希望右臂接触的力控垫，带 Gazebo contact sensor |
| 黄色小方块 `d2_contact_target_center` | 接触垫中心点，帮助判断末端是否对准 |
| 黄色箭头 `d2_press_arrow_*` | 右臂应沿这个方向压向接触垫 |
| 右侧小刻度 `d2_compliance_gauge_*` | 抽象表示导纳/柔顺修正，不是实物传感器 |
| 黄色探针 `d2_force_contact_probe` | Gazebo 动态接触体，用于触发接触力反馈 |

## 2. Gazebo 重力和转动惯量配置

D2 启动时，`scripts/play_harmonic_planned_record.sh` 会把 RM65-B URDF 转换为 Gazebo SDF，并对 D2 单独启用动力学配置：

| 对象 | 配置 |
| --- | --- |
| 机械臂 link | 保留官方 URDF 中的 `<inertial>` 质量和惯量；D2 中 `gravity=true` |
| 夹爪掌部 | 使用辨识后的 palm/flange 聚合质量和惯量 |
| 上/下手指 | 使用 finger/hook 聚合质量和惯量 |
| 力控探针 | `d2_force_probe_tip` 固定在右夹爪掌部前方 `0.135 m` |
| 固体目标 | 黑色 `d2_force_target_panel` 和蓝色 `d2_contact_pad` 带 Gazebo contact sensor |

关键文件：

| 文件 | 作用 |
| --- | --- |
| `rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh` | 生成 D2 Gazebo SDF，写入 gravity 和惯量 |
| `rm65b_dual_arm_ws/src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml` | 夹具辨识参数和控制参数 |
| `rm65b_dual_arm_ws/scripts/generate_day_world.py` | 生成 D2 按压墙、接触垫、方向箭头、柔顺刻度和接触探针 |

## 3. 末端夹具动力学辨识

### 3.1 方法

夹具按刚体 box 组件近似，组件包括：

| 组件 | 质量 kg | 尺寸 m | 中心位置 m |
| --- | ---: | --- | --- |
| flange_adapter | 0.160 | `[0.038, 0.038, 0.012]` | `[0.000, 0.000, 0.000]` |
| palm | 0.110 | `[0.040, 0.026, 0.018]` | `[0.030, 0.000, 0.000]` |
| upper_finger | 0.035 | `[0.052, 0.006, 0.010]` | `[0.070, 0.018, 0.000]` |
| upper_yarn_hook | 0.010 | `[0.014, 0.014, 0.010]` | `[0.098, 0.018, 0.000]` |
| lower_finger | 0.035 | `[0.052, 0.006, 0.010]` | `[0.070, -0.018, 0.000]` |
| lower_yarn_hook | 0.010 | `[0.014, 0.014, 0.010]` | `[0.098, -0.018, 0.000]` |

计算过程：

```text
m = sum(m_i)
c = sum(m_i * r_i) / m
I_i_box = diag(m_i*(b_i^2+c_i^2)/12,
               m_i*(a_i^2+c_i^2)/12,
               m_i*(a_i^2+b_i^2)/12)
I = sum(I_i_box + m_i * ((d_i^T d_i)E - d_i d_i^T))
```

其中 `d_i = r_i - c`，第二项是平行轴定理。

### 3.2 辨识结果

夹具总参数，以 `attached_scaled_gripper_palm` 为参考系：

| 参数 | 数值 |
| --- | --- |
| 总质量 | `0.360 kg` |
| 质心 | `[0.028222, 0.000, 0.000] m` |
| `Ixx` | `0.0000607867 kg*m^2` |
| `Iyy` | `0.0004030022 kg*m^2` |
| `Izz` | `0.0004525089 kg*m^2` |

Gazebo 实际写入分 link 参数：

| link | 质量 kg | inertial pose | `Ixx, Iyy, Izz` |
| --- | ---: | --- | --- |
| `attached_scaled_gripper_palm` | 0.270 | `[0.016296,0,0,0,0,0]` | `0.00007002, 0.000194813, 0.000243800` |
| `attached_scaled_gripper_upper_finger` | 0.045 | `[0.007111,0,0,0,0,0]` | `0.00000129, 0.000018649, 0.000018859` |
| `attached_scaled_gripper_lower_finger` | 0.045 | `[0.007111,0,0,0,0,0]` | `0.00000129, 0.000018649, 0.000018859` |

复现辨识：

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
python3 scripts/d2_identify_end_effector_dynamics.py \
  --config src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml \
  --output outputs/d2_dynamics_identification/d2_identified_end_effector_dynamics.json
```

## 4. 阻抗和导纳控制原理

### 4.1 阻抗控制

阻抗控制描述“给定位移后产生多大力”：

```text
F_imp = M_d * xdd + B_d * xd + K_d * x
```

在本实验中，Gazebo 接触力和导纳位移会被送入阻抗观测器，输出：

| topic | 含义 |
| --- | --- |
| `/force_control/impedance_wrench` | 阻抗模型估计力 |
| `/force_control/impedance_state` | 阻抗参数和估计力文本状态 |

### 4.2 导纳控制

导纳控制描述“受到力后产生多少位移/运动”：

```text
M_d * xdd + B_d * xd + K_d * x = F_ref - F_meas
```

D2 使用右臂力反馈 `F_meas` 和目标力 `F_ref` 计算虚拟位移 `x`，再映射为右臂 `joint6` 的修正量：

```text
q6_offset = joint6_rad_per_m * x
```

当前参数：

| 参数 | 数值 |
| --- | ---: |
| `M_d` | `1.20 kg` |
| `B_d` | `38.0 N*s/m` |
| `K_d` | `180.0 N/m` |
| `joint6_rad_per_m` | `1.60 rad/m` |
| `max_offset_rad` | `0.08 rad` |

对应 topic：

| topic | 含义 |
| --- | --- |
| `/force_control/target_wrench` | 目标力 |
| `/right_rm_driver/rm_driver/udp_six_force` | 测得力 |
| `/force_control/wrench_error` | `F_ref - F_meas` |
| `/force_control/admittance_displacement_m` | 导纳虚拟位移 |
| `/force_control/admittance_velocity_mps` | 导纳虚拟速度 |
| `/force_control/admittance_offset` | 映射到右臂末端关节的偏移 |
| `/force_control/corrected_right_joint_trajectory` | 修正后的右臂轨迹 |
| `/force_control/corrected_right_joint_states` | 修正后的右臂关节位置 |

复现控制计算：

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
python3 scripts/d2_impedance_admittance_calculation.py \
  --config src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml \
  --output outputs/d2_force_control/d2_impedance_admittance_step_response.csv \
  --force-step-n 3.0
```

## 5. D2 视频录制设计

### 5.1 启动命令

只想先看懂画面时，优先运行这个 live 可视化脚本：

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day02_force_visual.sh 180
```

它会循环演示右臂靠近蓝色接触垫、按压、退出；夹爪在整个 Day2 接触任务中保持闭合。这个脚本用于快速观察 Day2 任务，不替代正式 evidence 录制。

正式证据录制命令如下：

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

export OPS_ROOT=/mnt/e/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
DAY=day02
DOMAIN=212
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 \
SYNC_REPLAY_START_DELAY=8.0 \
MOVEIT_SEGMENT_DURATION=10.0 \
CAPTURE_TIMEOUT=420 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

### 5.2 录屏窗口

| 窗口 | 内容 |
| --- | --- |
| Gazebo | 右臂末端、夹爪、黑色目标墙、蓝色接触垫、黄色按压方向箭头 |
| RViz | 双臂 RobotModel，右臂关节运动 |
| 终端 1 | `/right_rm_driver/rm_driver/udp_six_force` |
| 终端 2 | `/force_control/state` 和 `/force_control/impedance_state` |
| 终端 3 | `/joint_states` 或 `/force_control/corrected_right_joint_states` |

建议终端命令：

```bash
ros2 topic echo /right_rm_driver/rm_driver/udp_six_force
ros2 topic echo /force_control/gazebo_contact_force_state
ros2 topic echo /force_control/state
ros2 topic echo /force_control/impedance_state
ros2 topic echo /force_control/corrected_right_joint_states
ros2 topic echo /joint_states
```

### 5.3 视频必须看到

| 编号 | 视频要求 | 判定标准 |
| --- | --- | --- |
| 1 | 右臂末端碰到固体目标 | Gazebo 中末端/探针沿黄色箭头靠近并接触蓝色接触垫 |
| 2 | 碰撞后受力 topic 变化 | `/right_rm_driver/rm_driver/udp_six_force.force_fz` 和 `/force_control/gazebo_contact_force_state` 有变化 |
| 3 | 受力后机械臂产生运动 | `/force_control/admittance_offset` 非零，`/force_control/corrected_right_joint_states` 有变化 |
| 4 | RViz 显示良好 | 双臂 RobotModel 不空白，右臂运动与 Gazebo 大体一致 |
| 5 | 解释清楚 | 说明阻抗是力-位移关系，导纳是力输入到运动输出 |

## 6. 局限说明

当前 D2 是 Gazebo/MoveIt 联合仿真实验，不是实机动力学标定。夹具质量和惯量来自几何组件聚合辨识，适合仿真和报告说明；如果老师要求实机标定，需要用实测质量、质心吊挂/摆振实验或 CAD 精确模型替换当前参数。
