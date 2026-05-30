# Day 4 编织动作状态机、示教复现与线材张力实验报告

## 0. 证据状态与禁止夸大表述

本报告当前交付的是标准化 primitive 种子轨迹、BehaviorTree.CPP 状态机、示教录制脚本、平滑插值脚本、回放脚本、Gazebo 织机/线材/张力模型和离线 PID/柔顺整定记录。`outputs/d4_teach_replay/day04_smoothed_primitives.yaml` 的 metadata 明确写明 `source: standardized D4 weaving primitive seed; replace with teach-pendant recording for final hardware evidence`，不是现场 teach pendant 实录轨迹。

因此现场汇报时不要说“示教复现重复精度已经小于 +/-1 mm”或“真实线材张力已经完成标定”。可以说“状态机、轨迹处理和张力仿真链路已完成；真实示教 YAML、真实重复精度和真实张力响应必须现场录制后替换当前种子轨迹证据”。

## 1. 结论

D4 不是单一“机械臂摆动”视频，而是编织原子动作、状态机、示教复现和张力适配的组合实验。当前交付物已覆盖：

1. 五个标准化编织 primitive：勾线 `hook_yarn`、挑线 `lift_yarn`、拉紧 `pull_tight`、移位 `shift`、换位 `exchange`。
2. BehaviorTree.CPP 状态机：互锁、张力等待、到位确认都写入 XML 和 C++ runtime，不再只靠延时。
3. 示教轨迹链路：示教器/手动牵引录制 `/joint_states`，再进行平滑插值和限速，最后用 BT/trajectory 回放。
4. Gazebo 线材张力：场景有织机、经线、纬线、张力弹簧质量块、张力计和材质颜色；张力节点使用弹簧-质点-阻尼模型，并发布 PID/柔顺参数与状态。

## 2. 交付物索引

| 交付物 | 文件 |
| --- | --- |
| 编织动作状态机逻辑图 | `docs/diagrams/day04_weaving_state_machine.drawio` |
| BehaviorTree.CPP XML | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/config/weaving_tree.xml` |
| C++ BT 节点代码 | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/src/weaving_bt_runner.cpp` |
| Python BT/回放兼容节点 | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/rm65b_weaving_primitives/weaving_coordinator.py` |
| 标准 primitive 轨迹 | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/trajectories/weaving_primitives.yaml` |
| 示教录制脚本 | `rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py` |
| 轨迹平滑脚本 | `rm65b_dual_arm_ws/scripts/d4_smooth_teach_trajectory.py` |
| 回放脚本 | `rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh` |
| 张力 PID/柔顺配置 | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/config/d4_tension_pid_compliance.yaml` |
| 张力仿真节点 | `rm65b_dual_arm_ws/src/rm65b_weaving_primitives/rm65b_weaving_primitives/tension_simulator.py` |
| PID 整定计算脚本 | `rm65b_dual_arm_ws/scripts/d4_tension_pid_tuning.py` |

## 3. 编织原子动作拆解

| 动作 | Primitive | 执行逻辑 | 触发条件 | 完成条件 |
| --- | --- | --- | --- | --- |
| 勾线 | `hook_yarn` | 左工具从纬线下方进入，右臂保持支撑位 | 双臂 ready，primitive lock 空闲 | `/joint_states` 到达目标关节位 |
| 挑线 | `lift_yarn` | 左臂抬升纬线越过经线通道 | 勾线到位，张力 0.45-1.60 N | 左工具到挑线路径末端 |
| 拉紧 | `pull_tight` | 右夹爪保持线材并向张力计方向拉紧 | 挑线到位，右夹爪/线材 attach | `/weaving/tension_status=OK` 且到位 |
| 移位 | `shift` | 双臂保持张力并移动到下一针位 | 拉紧张力 OK | 双臂到下一针位，张力 0.70-3.20 N |
| 换位 | `exchange` | 双臂回到下一半周期起点，释放过大张力 | 移位到位，张力未超限 | 双臂 ready，张力 0.35-2.00 N |

对应轨迹文件中的每个 primitive 都包含左右臂关节序列、触发条件、完成条件和验收参数。`pick_yarn` 保留为旧脚本兼容别名，但 D4 正式讲解使用 `lift_yarn`。

## 4. BehaviorTree.CPP 状态机设计

状态机图位于：

```text
docs/diagrams/day04_weaving_state_machine.drawio
```

核心状态流：

```text
arms_ready
 -> AcquirePrimitiveLock
 -> hook_yarn -> WaitForArrival
 -> WaitForTension -> lift_yarn -> WaitForArrival
 -> pull_tight -> WaitForTension -> WaitForArrival
 -> shift -> WaitForTension -> WaitForArrival
 -> exchange -> WaitForTension -> WaitForArrival
 -> OpenGripper -> ReleasePrimitiveLock
```

互锁机制：

| 机制 | 作用 |
| --- | --- |
| `CheckInterlock` | 检查 primitive lock，防止两个动作同时占用机械臂 |
| `AcquirePrimitiveLock` | 状态机开始时占用动作锁 |
| `ReleasePrimitiveLock` | 正常结束时释放动作锁 |
| `WaitForArrival` | 读取 `/joint_states`，确认目标关节到位 |
| `WaitForTension` | 读取 `/weaving/tension_n` 和 `/weaving/tension_status`，确认张力进入窗口 |

C++ runtime 使用 `MultiThreadedExecutor`，因此 BT 等待张力或到位时仍能接收 topic 更新。关键话题：

```text
/weaving/events
/joint_states
/weaving/tension_n
/weaving/tension_status
/weaving/tension_pid_state
/weaving/compliance_offset_m
```

## 5. 轨迹录制、平滑与回放

### 5.1 示教录制

现场用示教器或拖动示教时，按 primitive 分段录制：

```bash
python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py \
  --output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --primitive hook_yarn \
  --primitive lift_yarn \
  --primitive pull_tight \
  --primitive shift \
  --primitive exchange \
  --service-mode
```

每段开始和停止：

```bash
ros2 param set /d4_teach_replay_recorder primitive_name hook_yarn
ros2 service call /d4_teach_replay_recorder/start_recording std_srvs/srv/Trigger {}
ros2 service call /d4_teach_replay_recorder/stop_recording std_srvs/srv/Trigger {}
```

录制 YAML 必须包含：

```text
metadata.evidence_type: real_teach_replay_primitive_yaml
metadata.source: live_ros_joint_states
primitive_order: hook_yarn/lift_yarn/pull_tight/shift/exchange
```

### 5.2 平滑插值和速度规划

原始点位不直接回放，先用平滑脚本生成插值轨迹：

```bash
python3 rm65b_dual_arm_ws/scripts/d4_smooth_teach_trajectory.py \
  --input outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --output outputs/on_site/day04/logs/day04_smoothed_primitives.yaml \
  --quality-json outputs/on_site/day04/logs/day04_trajectory_quality.json \
  --sample-hz 25 \
  --max-velocity-rad-s 0.45
```

平滑方法：

```text
q(t) = q0 + (q1 - q0) * (3a^2 - 2a^3), a=(t-t0)/(t1-t0)
```

脚本会记录每个 primitive 的原始点数、平滑点数、最大关节步长和最大关节速度。验收时重点看：

```text
max_step_rad <= 0.08
max_velocity_rad_s <= 0.45
repeatability target < +/-1 mm
```

### 5.3 轨迹回放

回放脚本：

```bash
DRY_RUN=false BT_RUNTIME=cpp \
bash rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh \
  outputs/on_site/day04/logs/day04_smoothed_primitives.yaml
```

仿真或无真实 action server 时可先用：

```bash
DRY_RUN=true BT_RUNTIME=cpp bash rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh
```

## 6. Gazebo 场景与材质

D4 场景由 `generate_day_world.py` 生成，包含：

| 模型 | 材质/颜色 | 作用 |
| --- | --- | --- |
| `d4_upper_loom_rail`, `d4_lower_loom_rail` | 深灰金属 | 织机上下梁 |
| `d4_warp_*` | 米白色线材 | 经线 |
| `weft_yarn`, `d4_weft_preview_*` | 红色线材 | 纬线 |
| `d4_shuttle_lane` | 红色梭道 | 勾线/移位通道 |
| `d4_tension_scale` | 蓝色张力计 | 张力观测区域 |
| `d4_tension_spring_mass_*` | 黄色弹簧质量块 | 弹簧-质点张力模型可视化 |
| `d4_hook_target_marker`, `d4_lift_target_marker`, `d4_exchange_target_marker` | 绿/黄/紫 | 三个关键动作目标位 |

Gazebo 中还启用接触传感器：

```text
/world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact
/world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact
```

## 7. 线材张力模拟与 PID/柔顺整定

张力模型采用弹簧-质点-阻尼简化：

```text
m * x_ddot = k * (x_cmd + x_compliance - x) - c * x_dot
T = T0 + k * x
```

PID 计算张力误差：

```text
e = T_target - T
u = Kp * e + Ki * integral(e) + Kd * de/dt
x_compliance = clamp(u * compliance_gain, -x_max, x_max)
```

当前整定参数在：

```text
rm65b_dual_arm_ws/src/rm65b_weaving_primitives/config/d4_tension_pid_compliance.yaml
```

选定参数：

| 参数 | 数值 |
| --- | --- |
| 目标张力 | `1.85 N` |
| 合格张力窗口 | `0.80-3.40 N` |
| 弹簧刚度 | `95.0 N/m` |
| 阻尼 | `1.85 N*s/m` |
| 线材质量 | `0.020 kg` |
| PID | `Kp=0.42, Ki=0.08, Kd=0.018` |
| 柔顺增益 | `0.0035 m/N` |
| 最大柔顺位移 | `0.010 m` |

离线整定记录生成：

```bash
python3 rm65b_dual_arm_ws/scripts/d4_tension_pid_tuning.py \
  --config rm65b_dual_arm_ws/src/rm65b_weaving_primitives/config/d4_tension_pid_compliance.yaml \
  --csv rm65b_dual_arm_ws/outputs/d4_tension_pid/step_response.csv \
  --json rm65b_dual_arm_ws/outputs/d4_tension_pid/tuning_record.json
```

在线录制时关注：

```bash
ros2 topic echo /weaving/tension_n
ros2 topic echo /weaving/tension_status
ros2 topic echo /weaving/tension_pid_state
ros2 topic echo /weaving/compliance_offset_m
```

## 8. D4 视频验收标准

合格视频必须同时说明：

| 要求 | 视频证据 |
| --- | --- |
| 动作拆解清晰 | `/weaving/events` 依次出现 hook/lift/pull/shift/exchange |
| 状态机稳定 | 互锁、到位确认、张力等待事件没有卡死或误触发 |
| 轨迹可复现 | 使用示教 YAML 或平滑 YAML 回放，RViz/Gazebo 运动一致 |
| 轨迹平滑 | 质量 JSON 中最大速度和最大步长达标，视频无明显抖动 |
| 张力适配 | `/weaving/tension_status=OK`，PID 状态和柔顺位移有记录 |
| 材质和场景完整 | 织机、经线、纬线、张力计、弹簧质量块都在 Gazebo 可见 |

老师可能追问时的回答口径：

| 问题 | 回答 |
| --- | --- |
| 是不是只写了延时？ | 不是，BT 中有 `WaitForArrival` 和 `WaitForTension`，分别读取 `/joint_states` 和张力 topic |
| primitive 能不能独立执行？ | 能，每个 primitive 在 `weaving_primitives.yaml` 中有独立左右臂轨迹，可单独录制和平滑 |
| 张力参数在哪里？ | `d4_tension_pid_compliance.yaml` 和 `outputs/d4_tension_pid/tuning_record.json` |
| 材质在哪里体现？ | Gazebo world 生成时给经线、纬线、导轨、张力计、弹簧质量块配置不同 material/RGBA |
| 怎么证明不是空跑？ | Gazebo 画面有线材/梭道/张力计，topic 有接触、张力、BT 事件和关节轨迹 |
