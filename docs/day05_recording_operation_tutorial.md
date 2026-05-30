# Day 5 现场录制操作教程：一键启动、节点图、延时、安全与编织 Demo

本教程只说明录制时要打开哪些 ROS2/Gazebo/RViz 窗口、执行哪些命令、观察哪些证据。技术报告见：

```text
docs/day05_launch_safety_weaving_demo_report.md
docs/diagrams/day05_system_node_graph.drawio
```

## 1. 终端 A：录一键启动过程

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
mkdir -p outputs/d5_system

ros2 launch rm65b_dual_arm_bringup integrated_simulation.launch.py \
  day_id:=day05 \
  use_sim_time:=true \
  enable_gazebo:=true \
  enable_spawn_robots:=true \
  enable_move_group:=true \
  enable_gazebo_relays:=true
```

录屏里要能看到 `safety_supervisor`、`launch_audit_recorder`、`d5_latency_probe`、视觉、力控、规划、编织和张力节点启动。

## 2. 终端 B：录节点图

```bash
source /opt/ros/humble/setup.bash
source /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 topic echo /system/launch_audit --once

python3 scripts/d5_capture_node_graph.py \
  --output-json outputs/d5_system/day05_node_graph.json \
  --output-dot outputs/d5_system/day05_node_graph.dot
```

同时打开：

```text
docs/diagrams/day05_system_node_graph.drawio
outputs/d5_system/day05_launch_audit.json
outputs/d5_system/day05_node_graph.json
```

合格标准：`day05_launch_audit.json` 中 `missing_nodes` 和 `missing_topics` 应为空。

## 3. 终端 C：录通信延时

`d5_latency_probe` 随 launch 自动启动，实时写：

```text
outputs/d5_system/day05_latency_record.json
```

现场查看：

```bash
watch -n 1 cat outputs/d5_system/day05_latency_record.json
```

如果还没开始动作，可以先生成预算模板：

```bash
python3 scripts/d5_generate_latency_budget.py
cat outputs/d5_system/day05_latency_budget.json
```

正式讲解时必须说明：预算模板不是测量值，测量值看 `day05_latency_record.json`。`missing_chains` 必须为空，所有链路的 `max_ms` 必须小于 `200 ms`。

## 4. 终端 D：录故障安全机制

打开状态窗口：

```bash
ros2 topic echo /safety/state
ros2 topic echo /safety/fault
ros2 topic echo /safety/estop
```

注入一次故障：

```bash
python3 scripts/d5_fault_injection_demo.py \
  --fault force_limit \
  --output-json outputs/d5_system/day05_fault_demo.json
```

视频中要看到 `/safety/fault` 出现故障类型，`/safety/estop` 变成 `True`，`/safety/state` 进入 `ESTOP`。手动复位顺序：

```bash
ros2 service call /safety/recover_home std_srvs/srv/Trigger {}
ros2 service call /safety/ack_reset std_srvs/srv/Trigger {}
ros2 service call /safety/reset std_srvs/srv/Trigger {}
```

不要跳过 `ack_reset`，报告里 D5 采用人工确认复位。

## 5. Gazebo 和 RViz 录制

Gazebo 画面顺序：

| 顺序 | 画面 |
| --- | --- |
| 1 | 全景：双臂、视觉靶标、力控板、编织线材、张力计 |
| 2 | 视觉：左手眼相机和靶标，配合 `/vision/target_pose` |
| 3 | 力控：右臂接触/受力工位，配合 `/force_control/state` |
| 4 | 编织：织机导轨、纬线、张力计，配合 `/weaving/events` |
| 5 | 全景收尾：说明这是系统闭环，不是单臂独立动作 |

RViz 要求：

| 检查项 | 正常现象 |
| --- | --- |
| Fixed Frame | `world` |
| RobotModel | 左右臂都可见 |
| TF | 左右 base、末端、夹具坐标都可见 |
| `/joint_states` | 与 Gazebo 动作同步 |
| MoveIt | 不手动点 Plan/Execute |

Demo 期间建议打开：

```bash
ros2 topic echo /vision/status
ros2 topic echo /vision/target_pose
ros2 topic echo /visual_servo/twist_cmd
ros2 topic echo /force_control/state
ros2 topic echo /force_control/wrench_error
ros2 topic echo /weaving/events
ros2 topic echo /weaving/tension_n
ros2 topic echo /weaving/tension_pid_state
ros2 topic echo /safety/state
```

## 6. D5 合格标准

| 项目 | 合格标准 |
| --- | --- |
| 一键启动 | 单个 launch 启动全系统，无依赖缺失和节点冲突 |
| 节点图 | `day05_launch_audit.json`、`day05_node_graph.json`、Draw.io 图能对应上 |
| 延时 | `day05_latency_record.json` 中 `missing_chains=[]`，所有链路 `max_ms < 200` |
| 安全 | force/joint/collision/hardware estop 至少演示一种，能触发急停并按流程复位 |
| Demo | 视频中能讲清视觉定位、双臂运动、夹具/线材动作、张力拉紧和安全监控 |
| 报告 | `docs/day05_launch_safety_weaving_demo_report.md` 中能找到每个文件位置和启动方法 |
