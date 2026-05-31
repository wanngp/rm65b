# Day 5 一键启动、故障安全与首轮编织 Demo 报告

## 0. 证据状态与禁止夸大表述

本报告当前交付的是 D5 单 launch 集成方案、安全节点、节点图/延时记录节点、故障注入脚本、Draw.io 节点图和录制流程。`outputs/d5_system/day05_node_graph.json` 目前是 `mode=expected` 的预期节点图；`day05_latency_budget.json` 是预算模板；当前仓库尚未生成现场运行后的 `day05_launch_audit.json`、`day05_latency_record.json`、`day05_fault_demo.json`。

因此现场汇报时不要说“系统端到端延时已经小于 200 ms”或“首轮编织 Demo 已经无中断跑通”。可以说“D5 的一键启动、安全、节点图和延时记录能力已经补齐；最终验收必须以现场 launch 后生成的 audit/latency/fault/demo 数据为准”。

## 1. Executive Summary

Day 5 的验收目标不是单独展示某个动作，而是证明系统可以通过一个 Launch 文件启动机械臂驱动、夹具、视觉、力控、运动规划、编织状态机、张力模拟、安全保护和记录节点，并能在同一套 ROS2 通信图里完成“视觉识别定位 -> 双臂协同夹持/运动 -> 编织原子动作 -> 张力拉紧”的最小闭环。

本次补齐了四类交付物：

| 类别 | 文件/节点 | 用途 |
| --- | --- | --- |
| 一键启动 | `rm65b_dual_arm_bringup/launch/full_system.launch.py` | 启动全系统功能节点，并默认启动节点图与延时记录节点 |
| 安全机制 | `rm65b_safety/safety_supervisor.py` + `config/safety_supervisor.yaml` | 急停、关节限位、速度超限、力/力矩超限、碰撞急停、确认复位、回零恢复 |
| 记录工具 | `launch_audit_recorder`、`d5_latency_probe`、`scripts/d5_capture_node_graph.py`、`scripts/d5_fault_injection_demo.py` | 记录节点图、topic、通信延时和故障触发/复位证据 |
| 图表报告 | `docs/diagrams/day05_system_node_graph.drawio` | 可导入 Draw.io 的 D5 系统节点图 |

验收视频应录制“启动节点过程、节点图、通信延时记录、故障机制、首轮编织 Demo”，不要再录成普通单臂动作视频。

## 2. Context

D5 对应系统集成验收，覆盖以下验收项：

| 任务项 | 本报告对应实现 |
| --- | --- |
| Launch 文件重构 | `full_system.launch.py` 集成驱动、夹具、视觉、规划、力控、Gazebo relay、编织 BT、张力、安全、记录节点 |
| 通信优化与延时记录 | `d5_latency_probe` 记录视觉、力控、张力、安全链路延时，验收阈值 `<200 ms` |
| 故障安全机制 | `safety_supervisor` 处理硬件急停、关节超限、速度超限、力觉超限、碰撞、停机复位 |
| 首轮编织 Demo | D5 场景中串联视觉定位、双臂 MoveIt 轨迹、夹具闭合、编织 primitive、张力拉紧 |

本报告默认在仿真/录屏环境下执行：

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Methods

### 3.1 一键启动 Launch 结构

单 Launch 文件：

```text
rm65b_dual_arm_ws/src/rm65b_dual_arm_bringup/launch/full_system.launch.py
```

关键启动参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_hardware` | `false` | `true` 时启动左右睿尔曼硬件驱动 |
| `use_sim_time` | `false` | 仿真集成 launch 中设为 `true` |
| `enable_vision` | `true` | 启动手眼相机识别和视觉伺服 |
| `enable_planning` | `true` | 启动 D5 双臂规划节点 |
| `enable_force_admittance` | `true` | 启动力控阻抗/导纳控制 |
| `enable_tension_simulator` | `true` | 启动线材张力模型 |
| `enable_weaving_coordinator` | `true` | 启动 BehaviorTree.CPP 编织状态机 |
| `enable_launch_audit` | `true` | 启动节点图/话题记录 |
| `enable_latency_probe` | `true` | 启动通信延时记录 |
| `d5_launch_audit_json` | `outputs/d5_system/day05_launch_audit.json` | 节点图记录输出 |
| `d5_latency_json` | `outputs/d5_system/day05_latency_record.json` | 通信延时记录输出 |
| `tension_config` | `d4_tension_pid_compliance.yaml` | 复用 D4 已整定张力 PID/柔顺参数 |

仿真一键启动推荐用集成 launch：

```bash
ros2 launch rm65b_dual_arm_bringup integrated_simulation.launch.py \
  day_id:=day05 \
  use_sim_time:=true \
  enable_gazebo:=true \
  enable_spawn_robots:=true \
  enable_move_group:=true \
  enable_gazebo_relays:=true
```

只启动 ROS2 功能节点时使用：

```bash
ros2 launch rm65b_dual_arm_bringup full_system.launch.py \
  day_id:=day05 \
  use_hardware:=false \
  use_sim_time:=true \
  dry_run:=true \
  d5_launch_audit_json:=outputs/d5_system/day05_launch_audit.json \
  d5_latency_json:=outputs/d5_system/day05_latency_record.json
```

### 3.2 节点图记录方法

实时记录节点：

```text
rm65b_safety/rm65b_safety/launch_audit_recorder.py
```

输出：

```text
outputs/d5_system/day05_launch_audit.json
/system/launch_audit
```

离线/现场导出节点图：

```bash
python3 scripts/d5_capture_node_graph.py \
  --output-json outputs/d5_system/day05_node_graph.json \
  --output-dot outputs/d5_system/day05_node_graph.dot
```

Draw.io 图：

```text
docs/diagrams/day05_system_node_graph.drawio
```

### 3.3 通信延时记录方法

实时延时探针：

```text
rm65b_safety/rm65b_safety/d5_latency_probe.py
```

输出：

```text
outputs/d5_system/day05_latency_record.json
```

记录链路：

| 链路 | 源 | 目的 | 验收阈值 |
| --- | --- | --- | --- |
| 视觉识别 | `/left_camera/image_rect` | `/vision/target_pose` | `<200 ms` |
| 视觉伺服 | `/vision/target_pose` | `/visual_servo/twist_cmd` | `<200 ms` |
| Gazebo 视觉执行 | `/visual_servo/twist_cmd` | `/visual_servo/gazebo_adapter_state` | `<200 ms` |
| 力控计算 | `/force_control/target_wrench` | `/force_control/wrench_error` | `<200 ms` |
| 导纳修正 | `/force_control/wrench_error` | `/force_control/admittance_offset` | `<200 ms` |
| 编织张力 | `/weaving/events` | `/weaving/tension_n` | `<200 ms` |
| 安全急停 | `/safety/fault` | `/safety/estop` | `<200 ms` |

延时预算模板：

```bash
python3 scripts/d5_generate_latency_budget.py \
  --output-json outputs/d5_system/day05_latency_budget.json \
  --output-csv outputs/d5_system/day05_latency_budget.csv
```

注意：`day05_latency_budget.json` 是预算模板；正式验收要看录制过程中生成的 `day05_latency_record.json`，并确认 `missing_chains` 为空。

### 3.4 故障安全机制设计

安全节点：

```text
rm65b_dual_arm_ws/src/rm65b_safety/rm65b_safety/safety_supervisor.py
rm65b_dual_arm_ws/src/rm65b_safety/config/safety_supervisor.yaml
```

安全阈值：

| 保护项 | 阈值/条件 | 触发结果 |
| --- | --- | --- |
| 硬件急停 | `/safety/hardware_estop=True` | 发布 `/safety/estop=True`，向左右臂 stop topic 发 Empty |
| 关节位置超限 | 左右 6 轴分别在配置上下限外 | 急停，记录 `joint_limit` fault |
| 关节速度超限 | `abs(qdot)>1.2 rad/s` | 急停，记录 `joint_velocity` fault |
| 力超限 | 六维力范数 `>35 N` | 急停，记录 `force_limit` fault，并要求回零 |
| 力矩超限 | 六维力矩范数 `>4 Nm` | 急停，记录 `torque_limit` fault |
| Gazebo 碰撞 | 配置 contact topic 中 contact 数量 `>=1` | 急停，记录 `collision` fault |
| 看门狗 | `/joint_states` 超过 `0.5 s` 无更新 | `/safety/state` 发布 WARN |

安全输出：

| Topic/Service | 用途 |
| --- | --- |
| `/safety/state` | JSON 状态，包含 mode、fault_type、reason、limits |
| `/safety/fault` | 故障触发瞬间记录 |
| `/safety/estop` | 急停布尔状态 |
| `/safety/reset_required` | 是否必须人工确认后复位 |
| `/safety/recovery_command` | 故障后回零/恢复命令 |
| `/safety/stop_all` | 手动急停 service |
| `/safety/recover_home` | 发布回零恢复命令 |
| `/safety/ack_reset` | 人工确认复位 |
| `/safety/reset` | 解除软件急停 |

现场故障演示：

```bash
python3 scripts/d5_fault_injection_demo.py \
  --fault force_limit \
  --output-json outputs/d5_system/day05_fault_demo.json
```

也可以换成：

```bash
python3 scripts/d5_fault_injection_demo.py --fault joint_limit
python3 scripts/d5_fault_injection_demo.py --fault collision
python3 scripts/d5_fault_injection_demo.py --fault hardware_estop
```

### 3.5 首轮编织 Demo 设计

D5 的 Demo 不是追求成品，而是跑通闭环：

| 步骤 | 节点/证据 | 视频中要看到 |
| --- | --- | --- |
| 视觉识别定位 | `left_aruco_target_node`、`ibvs_controller` | 左手眼相机识别靶标，`/vision/target_pose` 与 `/visual_servo/twist_cmd` 有数据 |
| 双臂协同夹持/运动 | `dual_arm_planner`、夹具 action server | 左右臂都加载在 RViz，Gazebo 和 RViz 运动同步 |
| 编织原子动作 | `weaving_bt_runner` | `/weaving/events` 依次出现 hook/lift/pull/shift/exchange |
| 张力拉紧 | `tension_simulator` | `/weaving/tension_n`、`/weaving/tension_pid_state`、`/weaving/compliance_offset_m` 有变化 |
| 故障保护 | `safety_supervisor` | 注入故障后 `/safety/estop=True`，复位需先 ack |

MoveIt D5 轨迹阶段在：

```text
rm65b_dual_arm_ws/src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/dual_moveit_plan_client.py
```

包含：

```text
d5_hook_yarn -> d5_lift_yarn -> d5_force_pull -> d5_shift -> d5_exchange
```

Gazebo 中夹具/线材 attach 与 release 事件由：

```text
rm65b_dual_arm_ws/scripts/physical_interaction_controller.py
```

输出日志关键字：

```text
d5_force_contact_close
d5_yarn_pick_close
d5_yarn_attach
d5_probe_attach
d5_probe_release
d5_yarn_release
d5_release_open
```

## 4. Results

当前文件级结果：

| 项目 | 结果 |
| --- | --- |
| 单 Launch | `full_system.launch.py` 已默认启动 D5 记录节点 |
| 安全机制 | 已支持硬件急停、关节/速度/力/力矩/碰撞、人工确认复位 |
| 节点图 | 已提供实时采集节点、DOT 导出脚本、Draw.io 图和现场输出路径 |
| 通信延时 | 已提供实时记录节点、输出路径 `day05_latency_record.json` 和预算模板 |
| 编织 Demo | 沿用 D4 完整 primitive、张力 PID/柔顺参数，并在 D5 中串联视觉、力控和编织链路 |

正式验收时必须以现场录制生成的文件为准；当前仓库中这些现场文件尚未齐全：

```text
outputs/d5_system/day05_launch_audit.json
outputs/d5_system/day05_latency_record.json
outputs/d5_system/day05_fault_demo.json
outputs/d5_system/day05_node_graph.json
outputs/d5_system/day05_node_graph.dot
```

## 5. Discussion

录制时老师可能会问的问题和回答要点：

| 问题 | 回答要点 |
| --- | --- |
| 这是不是一键启动？ | 展示 `ros2 launch rm65b_dual_arm_bringup integrated_simulation.launch.py day_id:=day05 ...` 的启动过程和 `/system/launch_audit` |
| 节点有没有都起来？ | 展示 `day05_launch_audit.json`、`ros2 node list`、Draw.io 节点图 |
| 延时怎么证明？ | 展示 `day05_latency_record.json`，`missing_chains` 必须为空，每条链路最大值必须 `<200 ms` |
| 急停是不是只写了代码？ | 现场运行 `d5_fault_injection_demo.py`，看 `/safety/fault`、`/safety/estop`、左右 stop topic 和复位 service |
| 故障后怎么恢复？ | 顺序是 `/safety/recover_home` -> `/safety/ack_reset` -> `/safety/reset` |
| 编织 demo 是否完整闭环？ | 同屏展示视觉 topic、双臂 RViz、Gazebo 编织工位、`/weaving/events`、`/weaving/tension_n` |

已知限制：

| 限制 | 处理 |
| --- | --- |
| 仿真 contact 与真机碰撞不完全等价 | 报告中明确 Gazebo contact 是仿真碰撞证据，真机需接硬件急停和实际力传感 |
| 延时探针以本机接收时间计算 | 录制时同一台机器、同一 ROS_DOMAIN_ID 下可作为端到端软件链路延时证据 |
| D5 Demo 是半循环 | 验收标准本身要求跑通最小闭环，不要求完整成品 |

## 6. Appendix

### A. D5 录制窗口

| 窗口 | 内容 |
| --- | --- |
| 终端 A | `ros2 launch ... integrated_simulation.launch.py` 启动过程 |
| Gazebo | 视觉靶标、力控接触板、编织线材、双机械臂 |
| RViz | 双臂 RobotModel、TF、joint_states |
| 终端 B | `ros2 topic echo /system/launch_audit` |
| 终端 C | `watch -n 1 cat outputs/d5_system/day05_latency_record.json` |
| 终端 D | `ros2 topic echo /safety/state`、`/safety/fault`、`/safety/estop` |

### B. 一键启动后检查命令

```bash
ros2 node list
ros2 topic echo /system/launch_audit --once
ros2 topic echo /safety/state --once
ros2 topic hz /joint_states
ros2 topic echo /weaving/events
ros2 topic echo /weaving/tension_n
```

### C. 故障复位命令

```bash
ros2 service call /safety/recover_home std_srvs/srv/Trigger {}
ros2 service call /safety/ack_reset std_srvs/srv/Trigger {}
ros2 service call /safety/reset std_srvs/srv/Trigger {}
```

### D. 交付物索引

| 交付物 | 路径 |
| --- | --- |
| D5 报告 | `docs/day05_launch_safety_weaving_demo_report.md` |
| D5 节点图 | `docs/diagrams/day05_system_node_graph.drawio` |
| 一键启动 launch | `rm65b_dual_arm_ws/src/rm65b_dual_arm_bringup/launch/full_system.launch.py` |
| 仿真集成 launch | `rm65b_dual_arm_ws/src/rm65b_dual_arm_bringup/launch/integrated_simulation.launch.py` |
| 安全节点 | `rm65b_dual_arm_ws/src/rm65b_safety/rm65b_safety/safety_supervisor.py` |
| 安全配置 | `rm65b_dual_arm_ws/src/rm65b_safety/config/safety_supervisor.yaml` |
| 节点图脚本 | `rm65b_dual_arm_ws/scripts/d5_capture_node_graph.py` |
| 延时预算脚本 | `rm65b_dual_arm_ws/scripts/d5_generate_latency_budget.py` |
| 故障演示脚本 | `rm65b_dual_arm_ws/scripts/d5_fault_injection_demo.py` |
