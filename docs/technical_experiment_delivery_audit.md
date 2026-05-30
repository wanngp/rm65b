# 睿尔曼双机械臂系统技术实验清单交付审计

审计日期：2026-05-30

## 1. 结论

当前仓库已经完成了 D1-D5 的仿真、配置、节点、脚本和报告骨架，但还不能说已经完成真机验收。主要风险不是代码缺文件，而是报告中容易把“仿真/离线/模板”讲成“现场实测已完成”。

本次已修正以下问题：

| 问题 | 修改 |
| --- | --- |
| D1 报告容易被理解成夹爪延时、TCP 重复精度已实测 | 在 D1 报告顶部加入证据状态，明确缺真机延时、TCP 重复精度和全行程无碰撞实测 |
| D2 报告容易把几何惯量计算说成真机动力学辨识 | 在 D2 报告顶部加入证据状态，明确当前是仿真参数和算法验证，缺真机动力学辨识/重力补偿/夹持力阈值标定 |
| D3 报告容易把 Gazebo 安装矩阵说成真实手眼标定 | 在 D3 报告顶部加入证据状态，明确 `hand_eye_matrix.json` 来自 Gazebo 配置，不是 easy_handeye2 真实采样 |
| D4 报告容易把种子轨迹说成 teach pendant 实录轨迹 | 在 D4 报告顶部加入证据状态，明确 smoothed YAML 来自标准 seed，缺现场示教 YAML 和重复精度实测 |
| D5 报告容易把预期节点图/延时预算说成现场测量结果 | 在 D5 报告顶部加入证据状态，明确当前缺现场 `day05_launch_audit.json`、`day05_latency_record.json`、`day05_fault_demo.json` |
| 早期展示策划与后续需求不一致 | 在 `five_day_experiment_showcase_design.md` 顶部标注其为早期策划，最终以本审计为准 |

## 2. 交付状态总表

| Day | 技术实验清单要求 | 当前已完成 | 当前缺口 | 不允许这样讲 |
| --- | --- | --- | --- | --- |
| D1 | 夹具驱动集成、TCP/TF 精校准、双臂避碰/同步/交替运动 | 夹爪控制节点、Gazebo 夹爪开合、`Link6 -> gripper -> TCP` 矩阵、SRDF 自碰撞矩阵说明、MoveIt 双臂同步/交替规划脚本 | 真机夹爪开合延时 `<100 ms`、TCP 重复定位 `<+/-0.5 mm`、真机全行程无碰撞、同步误差 `<50 ms` | “D1 真机指标已经全部达标” |
| D2 | 末端夹具动力学辨识、重力/惯量配置、阻抗/导纳控制、受力 topic 与关节 topic 视频 | 夹具几何聚合惯量 YAML、动力学计算脚本、阻抗/导纳计算程序、Gazebo contact 力估计、受力/关节 topic 设计 | 真机动力学辨识、真实重力补偿保存/恢复、真实六维力延时 `<10 ms`、夹持力阈值标定、真实受力轨迹误差 `<+/-1 mm` | “已经完成真实动力学辨识和力控精度验收” |
| D3 | 手眼标定报告、手眼转换矩阵、标定代码、OpenCV 标志物识别视频、视觉伺服代码和视频 | Gazebo 眼在手上矩阵、OpenCV hand-eye 脚本、目标识别节点、`/vision/debug_image` 输出、IBVS 控制节点、离线 IBVS trace | 真实相机内参、easy_handeye2 采样外参、真实 OpenCV 录屏、像素误差 `<5 px`、世界误差 `<+/-1 mm`、多初始位置成功率 `>=95%` | “已经完成真实手眼标定，视觉精度已达标” |
| D4 | BT 状态机、原子动作轨迹、示教复现、线材张力模型、PID/柔顺整定记录、材质化场景 | BehaviorTree.CPP XML/C++ runtime、五个 primitive、轨迹平滑/回放脚本、Gazebo 织机/线材材质、弹簧-质点张力模型、离线 PID 调参记录 | teach pendant 真实录制 YAML、真实 replay 重复精度 `<+/-1 mm`、真实张力响应、真实线材不断裂/不松脱验证 | “示教复现和真实张力已经现场达标” |
| D5 | 单 launch 全系统、一键启动、节点图、端到端延时 `<200 ms`、故障安全、首轮编织 Demo 视频和复盘 | `full_system.launch.py`、`integrated_simulation.launch.py`、安全节点、节点审计、延时探针、故障注入脚本、Draw.io 节点图、D5 报告和录制教程 | 现场 launch audit、现场 latency record、故障演示 JSON、完整 Demo 视频/rosbag、首轮编织问题复盘 | “D5 已经实测无缺节点、延时小于 200 ms、Demo 稳定循环” |

## 3. 按报告逐项检查

### D1 夹爪、TF、自碰撞、双臂运动

| 交付项 | 文件/证据 | 状态 |
| --- | --- | --- |
| 夹爪控制节点 | `rm65b_gripper_control/gripper_action_server.py`、`left_gripper.yaml`、`right_gripper.yaml` | 已有软件节点 |
| 夹爪 Gazebo 开合 | `play_harmonic_planned_record.sh`、`physical_interaction_controller.py` | 已有仿真演示链路 |
| 末端到夹爪转换矩阵 | `docs/day01_gripper_tf_collision_dualarm_report.md` | 已写入报告 |
| 双臂自碰撞矩阵 | `rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.srdf`、D1 报告 | 已说明 |
| 双臂同步/交替运动 | `moveit_plan_client.py` D1 stages | 已有规划阶段 |
| 真机延时/重复精度 | 现场视频和测量数据 | 未完成 |

### D2 动力学、阻抗/导纳、力控

| 交付项 | 文件/证据 | 状态 |
| --- | --- | --- |
| 夹具动力学参数配置 | `src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml` | 已有仿真参数 |
| 参数计算程序 | `scripts/d2_identify_end_effector_dynamics.py` | 已有，但属于几何聚合/仿真辨识 |
| 阻抗/导纳计算程序 | `scripts/d2_impedance_admittance_calculation.py` | 已有 |
| Gazebo contact 力估计 | `gazebo_contact_force_estimator.py` | 已有 |
| 力控 controller | `force_admittance_controller.py` | 已有 |
| 真实重力补偿/力传感标定 | 现场真机记录 | 未完成 |
| 夹持力阈值标定报告 | 不同线材/织物实测 | 未完成 |

### D3 手眼标定、OpenCV、视觉伺服

| 交付项 | 文件/证据 | 状态 |
| --- | --- | --- |
| 手眼标定代码 | `scripts/d3_hand_eye_calibration.py` | 已有 |
| 手眼位置转换矩阵 | `outputs/d3_hand_eye/hand_eye_matrix.json` | 已有 Gazebo 配置矩阵 |
| OpenCV 标志物识别节点 | `aruco_target_node.py` | 已有 |
| OpenCV 调试图像 topic | `/vision/debug_image` | 已有代码 |
| 视觉伺服节点 | `ibvs_controller.py`、`gazebo_trajectory_relays.py` | 已有 |
| OpenCV 识别视频 | 现场 `$OUT/videos/day03_opencv_marker_debug.mp4` | 未在当前仓库中确认 |
| 真实手眼/视觉精度 | 真实相机采样与误差统计 | 未完成 |

### D4 状态机、示教、张力

| 交付项 | 文件/证据 | 状态 |
| --- | --- | --- |
| 状态机逻辑图 | `docs/diagrams/day04_weaving_state_machine.drawio` | 已有 |
| BT XML 和 C++ 节点 | `weaving_tree.xml`、`weaving_bt_runner.cpp` | 已有 |
| 五个 primitive | `weaving_primitives.yaml` | 已有 seed 轨迹 |
| 示教录制脚本 | `scripts/record_d4_real_teach_replay.py` | 已有 |
| 平滑/回放脚本 | `d4_smooth_teach_trajectory.py`、`d4_replay_weaving_primitives.sh` | 已有 |
| 张力模型与 PID 配置 | `d4_tension_pid_compliance.yaml`、`tension_simulator.py` | 已有 |
| PID 调参记录 | `outputs/d4_tension_pid/tuning_record.json` | 已有离线记录 |
| 真实 teach pendant YAML | `outputs/on_site/day04/...` | 未完成 |
| 真实重复精度/张力验证 | 现场数据 | 未完成 |

### D5 Launch、安全、Demo

| 交付项 | 文件/证据 | 状态 |
| --- | --- | --- |
| 单 launch 集成 | `full_system.launch.py`、`integrated_simulation.launch.py` | 已有 |
| 节点图记录节点 | `launch_audit_recorder.py` | 已有 |
| 延时记录节点 | `d5_latency_probe.py` | 已有 |
| 故障安全机制 | `safety_supervisor.py`、`safety_supervisor.yaml` | 已有 |
| 故障演示脚本 | `scripts/d5_fault_injection_demo.py` | 已有 |
| 节点图 Draw.io | `docs/diagrams/day05_system_node_graph.drawio` | 已有 |
| 现场 launch audit | `outputs/d5_system/day05_launch_audit.json` | 未生成 |
| 现场延时记录 | `outputs/d5_system/day05_latency_record.json` | 未生成 |
| 现场故障记录 | `outputs/d5_system/day05_fault_demo.json` | 未生成 |
| 首轮编织 Demo 视频和复盘 | 现场视频/rosbag/问题记录 | 未完成 |

## 4. 当前仓库中已经存在的输出证据

| 输出文件 | 证据性质 | 能不能当现场验收 |
| --- | --- | --- |
| `outputs/d3_hand_eye/hand_eye_matrix.json` | Gazebo 配置手眼矩阵 | 不能，真实手眼需现场采样 |
| `outputs/d3_visual_servo/ibvs_trace.csv` | 离线视觉伺服计算 trace | 不能替代视频和真实误差 |
| `outputs/d4_teach_replay/day04_smoothed_primitives.yaml` | seed 轨迹平滑结果 | 不能替代 teach pendant 录制 |
| `outputs/d4_teach_replay/day04_trajectory_quality.json` | seed 轨迹平滑质量 | 不能证明真实重复精度 |
| `outputs/d4_tension_pid/tuning_record.json` | 离线张力模型调参记录 | 不能证明真实线材张力 |
| `outputs/d5_system/day05_node_graph.json` | `mode=expected` 的预期节点图 | 不能替代现场 `ros2 node list` |
| `outputs/d5_system/day05_latency_budget.json` | 延时预算模板 | 不能替代实时延时记录 |

## 5. 正式录制前必须补齐的现场证据

| Day | 必须补齐 |
| --- | --- |
| D1 | 夹爪开合视频、`/joint_states`、TF/RViz、开合延时测量、同步/交替运动视频 |
| D2 | 真机或 Gazebo 接触视频、六维力 topic、阻抗/导纳 topic、关节位置 topic、动力学/重力补偿实测记录 |
| D3 | `/vision/debug_image` OpenCV 视频、手眼相机原始视频、`/vision/target_pose`、`/visual_servo/twist_cmd`、真实标定/误差记录 |
| D4 | 现场 teach pendant 录制 YAML、平滑后 YAML、回放视频、`/weaving/events`、张力 PID topic、重复精度记录 |
| D5 | 一键启动录屏、`day05_launch_audit.json`、`day05_latency_record.json`、`day05_fault_demo.json`、首轮编织 Demo 视频/rosbag/复盘 |

## 6. 构建与验证限制

已确认 `rm65b_safety` 新增节点可以构建并安装，Python/Draw.io/YAML 语法检查通过。当前 `rm65b_dual_arm_bringup` 全依赖链构建曾被上游 `rm_ros_interfaces` 阻断，报错中把中文工作区路径截成 `/mnt/e/1-`。这属于既有接口包/CMake 路径问题，不应写成 D5 一键启动已经现场无报错通过。现场录制前建议先在不含中文和空格的 WSL 路径下做一次完整 `colcon build`。

## 7. 建议统一口径

可以说：

```text
当前交付已经完成 D1-D5 的仿真场景、ROS2 节点、MoveIt/Gazebo 链路、报告、录制教程和现场采样脚本。所有真机指标仍按清单要求保留为现场验收项，录制时必须用 topic、视频和 JSON/CSV 文件补齐证据。
```

不要说：

```text
D1-D5 已经全部通过真机验收。
D2 已经完成真实动力学辨识。
D3 已经完成真实手眼标定。
D4 已经完成 teach pendant 重复精度 +/-1 mm。
D5 已经实测端到端延时小于 200 ms。
```

## 8. 后续修改优先级

1. 先按 `docs/day05_recording_operation_tutorial.md` 现场启动 D5，生成 `day05_launch_audit.json`、`day05_latency_record.json`、`day05_fault_demo.json`。
2. 再录 D3 `/vision/debug_image` 和真实/仿真手眼相机画面，补视觉误差统计。
3. 然后录 D4 真正 teach pendant YAML 和 replay 重复精度。
4. 最后补 D1/D2 真机定量指标：夹爪延时、TCP 重复精度、力传感延时、重力补偿和夹持力阈值。
