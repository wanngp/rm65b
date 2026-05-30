# 睿尔曼双机械臂五天实验现场操作教程与位置审查

本文档不是后期录制脚本说明，而是给录屏现场使用的操作手册。目标是在录制过程中正确打开 Gazebo、RViz、手眼相机和话题终端，并按每天实验内容操作和检查，保证画面能解释“机械臂、物体、相机、MoveIt 轨迹、接触/视觉/张力状态”之间的对应关系。

核心原则：

| 原则 | 要求 |
| --- | --- |
| Gazebo 是物理场景主证据 | 物块、靶标、力控板、纱线必须在 Gazebo 里和机械臂动作对上 |
| RViz 是机器人状态证据 | RViz 必须显示双机械臂 RobotModel，关节运动要和 Gazebo 同步 |
| 手眼相机是相机证据 | 左右相机分别固定在左右夹具掌部，不用第三方相机替代手眼相机 |
| 话题终端是状态证据 | 视觉、力控、接触、张力、attach/release 不能只靠口头说明 |
| 不手动拖动物体 | 录制时不要在 Gazebo 里拖动物块/靶标/纱线，物体只能由 Gazebo 接触或 attach 机制运动 |

## 0. 坐标和基座约定

Gazebo 世界坐标单位为米，角度为弧度。

双臂固定基座：

| 机械臂 | Gazebo spawn 位姿 | 说明 |
| --- | --- | --- |
| 左臂 `left_rm65b` | `x=-0.45 y=0 z=0.02 yaw=+1.5708` | 面向中间工作区 |
| 右臂 `right_rm65b` | `x=0.45 y=0 z=0.02 yaw=-1.5708` | 面向中间工作区 |

末端参考：

| 参考点 | 用途 |
| --- | --- |
| `attached_scaled_gripper_palm` | Gazebo 中夹具掌部，attach/contact 都以它为父坐标系 |
| `TCP` 近似点 | `palm + x=0.112 m`，用于判断末端是否到达目标 |
| D2 probe 近似点 | `palm + x=0.135 m`，用于判断力控接触 |
| 左手眼相机 | 挂在左夹具掌部，默认 `/left_camera/image_rect` |
| 右手眼相机 | 挂在右夹具掌部，默认 `/right_camera/image_rect` |
| `evidence_camera` | Gazebo 场景固定观察相机，不是手眼相机 |

## 1. 位置关系审查结论

以下数值来自当前 `generate_day_world.py` 与 `moveit_plan_client.py` 的 FK 对照。录制前先按这些关系判断物体有没有放错。

### Day 1 夹爪、TF、自碰撞矩阵、双臂运动能力

| 项目 | 世界坐标 |
| --- | --- |
| 左侧 TCP 参考柱 `d1_tcp_left_target` | `(-0.389, 0.123, 0.760)` |
| 右侧 TCP 参考柱 `d1_tcp_right_target` | `(0.519, 0.075, 0.760)` |
| 左臂同步运动参考线 `d1_sync_motion_left_lane` | `(-0.260, 0.110, 0.610)` |
| 右臂同步运动参考线 `d1_sync_motion_right_lane` | `(0.260, 0.110, 0.610)` |
| 中心安全间隔柱 `d1_center_clearance_marker` | `(0.000, 0.000, 0.650)` |
| 夹爪开合标尺 `d1_gripper_gap_gauge` | `(0.000, 0.045, 0.570)` |

结论：D1 不再做物块夹取放置，只展示四件事：夹爪开/合两个状态，`Link6 -> attached_scaled_gripper` 相对位姿，双臂自碰撞矩阵，双臂同步/交替运动能力。

### Day 2 动力学配置、阻抗/导纳力控接触

| 项目 | 世界坐标 |
| --- | --- |
| 力控目标板 `d2_force_target_panel` | `(0.542, -0.060, 0.760)` |
| 接触垫 `d2_contact_pad` | `(0.542, -0.130, 0.760)` |
| 动态接触探针 `d2_force_contact_probe` | `(0.542, -0.235, 0.760)` |
| 右臂 `d2_force_press` TCP | `(0.556, -0.099, 0.695)` |

结论：D2 是右臂动力学和力控接触实验。录制时必须同时展示 Gazebo 固体接触、受力 topic、导纳/阻抗状态、关节位置 topic。D2 报告见 `docs/day02_force_dynamics_impedance_admittance_report.md`。

### Day 3 手眼标定、OpenCV 识别和视觉伺服

| 项目 | 世界坐标 |
| --- | --- |
| 视觉靶标 `vision_target` | `(-0.509, -0.165, 0.531)` |
| 绿色识别块 `d3_marker_center` | `(-0.509, -0.206, 0.531)` |
| 左臂初始 TCP | `(-0.518, -0.051, 0.575)` |
| 左手眼相机初始位置 | `(-0.490, -0.114, 0.677)` |
| 左臂点靶标 TCP | `(-0.523, -0.166, 0.520)` |

结论：D3 必须按“左臂手眼相机一开始能看到绿色靶标，OpenCV 标注识别结果，然后视觉伺服驱动左臂点靶标”展示。不要说成固定场景相机跟随。D3 报告见 `docs/day03_hand_eye_opencv_visual_servo_report.md`，识别视频看 `/vision/debug_image`。

### Day 4 编织状态机和张力适配

| 项目 | 世界坐标 |
| --- | --- |
| 纱线 `weft_yarn` | `(0.564, -0.028, 0.689)` |
| 梭道 `d4_shuttle_lane` | `(0.088, 0.030, 0.790)` |
| 梭道动态探针 `d4_shuttle_contact_probe` | `(0.564, -0.055, 0.689)` |
| 张力探针 `d4_tension_contact_probe` | `(0.542, -0.120, 0.702)` |
| 勾线/挑线 marker | `d4_hook_target_marker`, `d4_lift_target_marker` |
| 右臂 `d4_pull_tight` TCP | `(0.556, -0.099, 0.695)` |

结论：D4 要按 BehaviorTree.CPP 状态机讲，五个 primitive 是勾线、挑线、拉紧、移位、换位；视频必须同时展示材质化编织场景、示教/回放轨迹证据、张力 PID/柔顺 topic。报告见 `docs/day04_weaving_state_machine_teach_tension_report.md`。

### Day 5 集成演示

D5 是视觉、力控、编织动作的组合场景。讲法应是“视觉状态 + 力控状态 + 张力状态 + 右臂纱线 attach/release + Gazebo 完整过程”，不要讲成所有物体都发生强 contact。

## 2. 录制现场窗口布局

推荐录屏时打开四类窗口：

| 窗口 | 用途 | 放屏幕位置建议 |
| --- | --- | --- |
| Gazebo GUI | 主画面，展示机械臂和物体真实关系 | 屏幕左侧大窗口 |
| RViz | 展示双臂 RobotModel、TF、关节运动 | 屏幕右上 |
| 手眼相机 | 展示 `/left_camera/image_rect` 或 `/right_camera/image_rect` | 屏幕右中 |
| 话题终端 | 展示状态、阶段、contact、vision、force、tension | 屏幕右下 |

录制软件由你自己控制。下面的脚本只负责启动仿真、生成当天场景、调用 MoveIt 规划、驱动 Gazebo/RViz 需要的 ROS 话题；不要把它理解为最终视频录制脚本。

## 3. 通用启动流程

所有命令都在 WSL Ubuntu/ROS2 终端执行。

先构建和 source：

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export OPS_ROOT=/mnt/e/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OPS_ROOT"
```

每一天按下面顺序启动。

### 3.1 终端 A：启动当天后端和动作

把 `DAY` 和 `DOMAIN` 换成当天编号。这个终端会启动 Gazebo server、MoveIt、桥接、视觉/力控/编织节点，并自动执行当天 MoveIt 轨迹。

```bash
DAY=day01
DOMAIN=211
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 \
SYNC_REPLAY_START_DELAY=8.0 \
MOVEIT_SEGMENT_DURATION=10.0 \
CAPTURE_TIMEOUT=360 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

说明：

| 参数 | 作用 |
| --- | --- |
| `RECORD_RVIZ=0` | 不让脚本录 RViz，你自己录屏 |
| `SYNC_REPLAY_START_DELAY=8.0` | 给你 8 秒时间切好 Gazebo/RViz/相机窗口后再开始动作 |
| `MOVEIT_SEGMENT_DURATION=10.0` | 放慢每段动作，方便老师看清接触过程 |
| `OUT` | 保存当天日志，现场讲解时可 `tail -f` 查看 |

如果要重录某一天，先在终端 A 按 `Ctrl+C` 停止，再重新执行当天命令。不要在同一个 Gazebo 进程里混录多个 day。

### 3.2 终端 B：打开 Gazebo GUI

终端 A 启动后，另开终端：

```bash
source /opt/ros/humble/setup.bash
source /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/install/setup.bash
gz sim -g
```

Gazebo GUI 操作要求：

| 操作 | 要求 |
| --- | --- |
| 等待模型出现 | Entity Tree 里应能看到 `left_rm65b`、`right_rm65b` 和当天物体 |
| 调视角 | 只允许旋转、平移、缩放相机视角 |
| 不拖物体 | 不要手动拖动物块、靶标、纱线、力控板 |
| 看局部 | 每天都要给目标物近景，不要全程远景 |
| 看整体 | 每天至少给一次双臂和完整工作台关系 |

若 Gazebo GUI 空白，先等 5-10 秒；仍然空白时在终端检查：

```bash
gz topic -l | grep rm65b_world
```

### 3.3 终端 C：打开 RViz

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/install/setup.bash
ros2 launch rm65b_dual_arm_moveit_config moveit_rviz.launch.py use_sim_time:=false
```

RViz 操作要求：

| 检查项 | 正常现象 |
| --- | --- |
| Fixed Frame | `world` |
| RobotModel | 左右机械臂都可见 |
| 关节运动 | 和 Gazebo 里机械臂动作同步 |
| 不手动 Plan | 当前轨迹由终端 A 的 MoveIt client 自动规划，录制时不要在 RViz 手动点 Plan/Execute |

如果 RViz 没加载机械臂，立即停止录制并检查：

```bash
ros2 topic echo /robot_description --once
ros2 topic hz /joint_states
ros2 run tf2_tools view_frames
```

判断标准：

| 检查项 | 正常现象 | 异常处理 |
| --- | --- | --- |
| `/robot_description` | 能 echo 出 URDF XML | 重新 build `rm65b_dual_arm_moveit_config` |
| `/joint_states` | 有频率，且包含 `left_joint1...right_joint6` | 说明 joint state replay 没启动或 ROS_DOMAIN_ID 不一致 |
| RViz Fixed Frame | `world` 不红 | 检查 `/tf_static` 是否有 `world -> left_base_link/right_base_link` |
| RobotModel | 双臂可见 | 关闭 RViz 后重新 launch |

### 3.4 终端 D：打开手眼相机和话题

手眼相机窗口：

```bash
source /opt/ros/humble/setup.bash
source /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中按当天选择：

| Day | 主要相机 |
| --- | --- |
| Day 1 | 可用 Gazebo 主视角和 RViz 为主，相机不是核心证据 |
| Day 2 | `/right_camera/image_rect`，左相机可作为观察补充 |
| Day 3 | `/vision/debug_image` 为主要证据，必要时补充 `/left_camera/image_rect` |
| Day 4 | `/right_camera/image_rect` |
| Day 5 | 左右相机都可以切换，但左相机用于视觉，右相机用于接触/纱线 |

通用话题终端：

```bash
ros2 topic echo /acceptance/day_status
ros2 topic echo /dual_arm_planning/phase
ros2 topic hz /joint_states
```

日志终端按当天 `OUT` 查看，例如：

```bash
tail -f "$OUT/logs/moveit_plan_client.log"
tail -f "$OUT/logs/physical_interaction_events.log"
```

## 4. Day 1 操作教程：夹爪、TF、自碰撞矩阵、双臂同步/交替运动

### 启动当天动作

```bash
DAY=day01
DOMAIN=211
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=360 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

### Gazebo 操作

1. 打开 `gz sim -g` 后，先给整体视角：左右机械臂、左右运动参考线、中心安全间隔柱、夹爪开合标尺都在画面中。
2. 拉近左/右夹爪，先展示开合标尺附近的打开状态。
3. 动作开始后，脚本会把夹爪从打开 `0.018 m` 切到闭合 `0.004 m`，录屏中必须能看出开和合两个状态。
4. 继续保持整体视角，观察两臂同时向外/向目标位运动，这是同步运动能力。
5. 后半段观察“左臂动、右臂保持”和“右臂动、左臂保持”，这是交替运动能力。
6. 全程不要拖动物体，D1 场景中不再有取放物块。

### RViz 操作

RViz 中必须看到双臂 RobotModel 和 TF 坐标系。重点显示：

| RViz 内容 | 作用 |
| --- | --- |
| `world`、`left_base_link`、`right_base_link` | 说明双臂固定基座 |
| `left_Link6`、`right_Link6` | 说明机械臂末端法兰 |
| `left_attached_scaled_gripper`、`right_attached_scaled_gripper` | 说明夹爪相对末端的固定安装关系 |
| 双臂关节运动 | 证明 Gazebo 和 RViz 使用同一份 MoveIt 轨迹 |

录制时不要在 RViz 里手动 Plan/Execute。当前 D1 轨迹由 `dual_moveit_plan_client` 调 MoveIt `/move_action` 规划，组名是 `dual_arms`。

### 话题和日志

```bash
tail -f "$OUT/logs/physical_interaction_events.log"
ros2 topic echo /dual_arm_planning/phase
ros2 topic echo /rm65b_gripper/upper_finger_cmd
ros2 topic echo /rm65b_gripper/lower_finger_cmd
```

现场应能看到这些日志关键词：

| 关键词 | 含义 |
| --- | --- |
| `d1_gripper_close_both` | 左右夹爪闭合 |
| `d1_gripper_open_both` | 左右夹爪打开 |
| `d1_gripper_close_during_dual_motion` | 双臂运动中再次展示闭合状态 |
| `d1_gripper_final_open` | 结束前恢复打开状态 |

`/dual_arm_planning/phase` 应出现：

| phase | 说明 |
| --- | --- |
| `d1_sync_spread_both_arms` | 双臂同步运动 |
| `d1_sync_shift_both_arms` | 双臂同步换位 |
| `d1_alternate_left_moves_right_holds` | 左臂动，右臂保持 |
| `d1_alternate_right_moves_left_holds` | 右臂动，左臂保持 |
| `d1_return_dual_ready` | 双臂回到准备位 |

### 合格标准

| 画面 | 必须成立 |
| --- | --- |
| Gazebo | 清楚看到夹爪打开和闭合两个状态 |
| Gazebo | 清楚看到双臂同步运动和交替运动 |
| RViz | 双臂 RobotModel、TF 坐标系、关节运动都可见 |
| 报告 | 写明夹爪配置、末端到夹爪转移矩阵、自碰撞矩阵、双臂规划配置 |
| 讲解 | 不再讲夹取放置，不出现物块 attach/detach 叙事 |

## 5. Day 2 操作教程：动力学配置、阻抗/导纳力控接触

### 启动当天动作

```bash
DAY=day02
DOMAIN=212
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=420 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

启动后先说明 D2 的动力学配置：

| 项目 | 说明 |
| --- | --- |
| 机械臂 link | D2 中 Gazebo SDF 保留官方 URDF inertial，并设置 `gravity=true` |
| 夹爪掌部 | `mass=0.270 kg`，使用辨识出的 palm/flange 聚合惯量 |
| 上/下手指 | 每个 `mass=0.045 kg`，使用 finger/hook 聚合惯量 |
| 总夹具 | `mass=0.360 kg`，质心 `[0.034,0,0] m` |
| 报告 | `docs/day02_force_dynamics_impedance_admittance_report.md` |

### Gazebo 操作

1. 先给整体视角，说明右臂靠近力控工位。
2. 拉近 `d2_force_target_panel` 和 `d2_contact_pad`，让老师看清目标板位置。
3. 动作开始后，视角跟随右臂末端，不要拍成远处空跑。
4. 接触阶段保持目标板、接触垫、右夹具末端同时在画面里。
5. 受力后继续看右臂末端，说明导纳控制把力误差转换成关节修正运动。

### RViz 操作

RViz 中主要看右臂轨迹。左臂只是观察/辅助，不要讲成左臂也在接触。

RViz 必须显示双臂 RobotModel；右臂动作要和 Gazebo 中接近目标板的动作一致。D2 录屏时不要在 RViz 手动 Plan/Execute。

### 相机和话题

相机窗口选择：

```text
/right_camera/image_rect
```

话题终端：

```bash
ros2 topic echo /right_rm_driver/rm_driver/udp_six_force
ros2 topic echo /joint_states
ros2 topic echo /force_control/state
ros2 topic echo /force_control/impedance_state
ros2 topic echo /force_control/admittance_offset
ros2 topic echo /force_control/corrected_right_joint_states
ros2 topic echo /force_control/gazebo_contact_force_state
ros2 topic echo /world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact
```

现场讲解顺序：

| 顺序 | 看什么 | 说明 |
| --- | --- | --- |
| 1 | Gazebo 右臂末端和固体板 | 机械臂碰固体 |
| 2 | `/right_rm_driver/rm_driver/udp_six_force` | 碰撞后 `force_fz` 变化 |
| 3 | `/force_control/state` | 导纳控制计算力误差和位移 |
| 4 | `/force_control/impedance_state` | 阻抗模型估计力-位移关系 |
| 5 | `/force_control/corrected_right_joint_states` | 受力后右臂关节位置修正 |
| 6 | `/joint_states` | RViz 使用的关节状态持续变化 |

### 合格标准

| 画面 | 必须成立 |
| --- | --- |
| Gazebo | 右臂末端靠近力控板/接触垫，目标物不偏离工位 |
| RViz | 右臂运动和 Gazebo 对齐 |
| 话题 | 右臂六维力、导纳状态、阻抗状态、修正关节位置都能看到变化 |
| 报告 | 写明夹具动力学辨识、Gazebo 重力/惯量配置、阻抗/导纳公式和程序位置 |
| 讲解 | 说“右臂阻抗/导纳力控接触”，不要说“双臂都接触目标板” |

## 6. Day 3 操作教程：手眼标定、OpenCV 标志物识别和视觉伺服

### 启动当天动作

```bash
DAY=day03
DOMAIN=213
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=380 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

配套报告：

```text
docs/day03_hand_eye_opencv_visual_servo_report.md
```

当天生成的视频重点看：

| 文件 | 用途 |
| --- | --- |
| `$OUT/videos/day03_opencv_marker_debug.mp4` | 标志物识别 OpenCV 视频 |
| `$OUT/videos/day03_left_gazebo_camera.mp4` | 左手眼相机原始视频 |
| `$OUT/videos/day03_moveit_harmonic_playback.mp4` | Gazebo 视觉伺服实验视频 |
| `$OUT/videos/day03_rviz.mp4` | RViz 视频，需 `RECORD_RVIZ=1` |

### Gazebo 操作

1. 先给整体视角，确认左臂、左夹具手眼相机、视觉板、绿色靶标在同一个工作区。
2. 拉近 `d3_vision_board`、`d3_marker_center`、`vision_target`，让老师知道靶标不是后期叠加。
3. 动作开始后，跟随左臂末端，看它从初始观察位移动到靶标附近。
4. 视觉伺服阶段保持左夹具末端和绿色靶标同时在画面里，不能拍成远处空跑。
5. 点靶标时，画面要能看到末端接近 `vision_target`；如果接触话题没有数据，不要讲成已经接触。

### 手眼相机操作

手眼原始相机窗口可以选择：

```text
/left_camera/image_rect
```

OpenCV 标志物识别窗口必须选择：

```text
/vision/debug_image
```

录屏一开始就要让老师看到 `/vision/debug_image` 里有绿色靶标、图像中心、靶标中心和像素误差标注。这里的“视觉跟随”是左臂夹具上的相机看靶标，不是 Gazebo 固定 `evidence_camera`。

可直接打开：

```bash
ros2 run rqt_image_view rqt_image_view /vision/debug_image
```

手眼矩阵讲解用报告中的三组矩阵：

| 矩阵 | 含义 |
| --- | --- |
| `^G T_C` | 夹具掌部到相机，平移 `[0.034,0,0.095] m` |
| `^C T_G` | 相机到夹具掌部，平移 `[-0.034,0,-0.095] m` |
| `^L6 T_C` | 机械臂 `Link6` 到相机，平移 `[0.052,0,0.095] m` |

### RViz 操作

RViz 中主要看左臂轨迹、双臂 RobotModel 和 TF。右臂可保持避让或静止。RViz 和 Gazebo 的左臂姿态要一致。

RViz 不要手动 Plan/Execute；它显示的是脚本启动后同一份 MoveIt/Gazebo 轨迹和视觉伺服修正证据。

### 话题

```bash
ros2 topic echo /vision/status
ros2 topic echo /vision/target_pose
ros2 topic echo /vision/metrics
ros2 topic echo /visual_servo/twist_cmd
ros2 topic echo /visual_servo/gazebo_adapter_state
ros2 topic echo /visual_servo/aligned
ros2 topic echo /visual_servo/left_corrected_joint_trajectory
ros2 topic echo /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact
```

现场讲解顺序：

| 顺序 | 看什么 | 说明 |
| --- | --- | --- |
| 1 | 报告矩阵 | 说明相机是眼在手上，安装在左夹具掌部 |
| 2 | `/vision/debug_image` | OpenCV 已识别靶标并画出像素误差 |
| 3 | `/vision/target_pose` | 像素偏差被转换成相机坐标系下的 `y/z` 偏差 |
| 4 | `/visual_servo/twist_cmd` | 视觉伺服根据偏差给出速度控制量 |
| 5 | `/visual_servo/gazebo_adapter_state` | 控制量被转成左臂 Gazebo 小步关节轨迹 |
| 6 | Gazebo + RViz | 左臂末端向靶标靠近，两个窗口姿态一致 |

### 合格标准

| 画面 | 必须成立 |
| --- | --- |
| 左手眼相机 | `/left_camera/image_rect` 一开始就能看到绿色靶标 |
| OpenCV 视频 | `/vision/debug_image` 有靶标中心、图像中心和像素误差标注 |
| Gazebo | 左臂末端接近靶标区域，不是机械臂不碰物体而靶标自己完成 |
| RViz | 左臂 RobotModel 运动和 Gazebo 对齐 |
| 话题 | `/vision/status` 出现 `salient_bright` 或 `aruco`，`/visual_servo/twist_cmd` 有控制量 |
| 报告 | 写明手眼标定方法、手眼矩阵、OpenCV 识别代码、视觉伺服原理和代码位置 |
| 讲解 | 强调“眼在手上”和“识别结果驱动控制”，不要说成固定场景相机跟随 |

## 7. Day 4 操作教程：编织状态机、示教复现和张力适配

### 启动当天动作

```bash
DAY=day04
DOMAIN=214
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=420 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

配套报告和图：

```text
docs/day04_weaving_state_machine_teach_tension_report.md
docs/diagrams/day04_weaving_state_machine.drawio
```

### Gazebo 操作

1. 先给整体视角，说明编织工位由织机导轨、经线、纬线、梭道、张力计和弹簧质量块组成。
2. 拉近 `weft_yarn`、`d4_warp_*`、`d4_shuttle_lane`、`d4_tension_scale`、`d4_tension_spring_mass_*`，让材质和张力模型可见。
3. 动作开始后按五个 primitive 讲解：勾线 `hook_yarn`、挑线 `lift_yarn`、拉紧 `pull_tight`、移位 `shift`、换位 `exchange`。
4. 拉紧阶段保持右夹具、纬线、张力计和弹簧质量块同时在画面内。
5. 移位/换位阶段切回整体视角，确认不是线材自己动、机械臂空跑。

### RViz 操作

RViz 中看双臂 RobotModel、关节轨迹和 `/joint_states` 到位确认。讲解重点是“BT 状态机驱动双臂 primitive”，不要只说右臂单独动作。

RViz 不要手动 Plan/Execute；录屏中的轨迹来自 MoveIt/primitive 回放链路。

### 相机和话题

相机窗口选择：

```text
/right_camera/image_rect
```

话题和日志：

```bash
ros2 topic echo /weaving/events
ros2 topic echo /weaving/tension_n
ros2 topic echo /weaving/tension_status
ros2 topic echo /weaving/tension_pid_state
ros2 topic echo /weaving/compliance_offset_m
ros2 topic echo /joint_states
ros2 topic echo /force_control/gazebo_contact_force_state
tail -f "$OUT/logs/physical_interaction_events.log"
```

现场应能看到这些日志关键词：

| 关键词 | 含义 |
| --- | --- |
| `primitive_start:hook_yarn` | 勾线动作开始 |
| `primitive_start:lift_yarn` | 挑线动作开始 |
| `primitive_start:pull_tight` | 拉紧动作开始 |
| `primitive_start:shift` | 移位动作开始 |
| `primitive_start:exchange` | 换位动作开始 |
| `arrival_confirmed` | 到位确认，不是纯延时 |
| `tension_confirmed` | 张力进入窗口 |
| `d4_hook_yarn_close` | 夹爪闭合保持线材 |
| `d4_yarn_attach` | 纱线 attach 到右夹具 |
| `d4_probe_attach` | 张力/接触探针 attach |
| `d4_yarn_release` | 纱线释放 |

示教和平滑回放命令：

```bash
python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py \
  --output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --primitive hook_yarn --primitive lift_yarn --primitive pull_tight --primitive shift --primitive exchange \
  --service-mode

python3 rm65b_dual_arm_ws/scripts/d4_smooth_teach_trajectory.py \
  --input outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --output outputs/on_site/day04/logs/day04_smoothed_primitives.yaml \
  --quality-json outputs/on_site/day04/logs/day04_trajectory_quality.json

DRY_RUN=false BT_RUNTIME=cpp bash rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh \
  outputs/on_site/day04/logs/day04_smoothed_primitives.yaml
```

### 合格标准

| 画面 | 必须成立 |
| --- | --- |
| Gazebo | 织机、经线、纬线、梭道、张力计、弹簧质量块和关键动作 marker 都可见 |
| RViz | 双臂模型可见，五个 primitive 的关节运动和 Gazebo 同步 |
| 状态机 | `/weaving/events` 依次出现 hook/lift/pull/shift/exchange、互锁、到位确认、张力确认 |
| 轨迹 | 示教 YAML、平滑 YAML、质量 JSON、回放脚本都有文件位置和命令 |
| 张力 | `/weaving/tension_n`、`/weaving/tension_pid_state`、`/weaving/compliance_offset_m` 有变化和参数记录 |
| 讲解 | 说“BT 状态机组合标准 primitive，在张力和到位信号下切换”，不要说成固定脚本延时动作 |

## 8. Day 5 操作教程：综合集成演示

### 启动当天动作

```bash
DAY=day05
DOMAIN=215
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=480 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

### Gazebo 操作

1. 先给整体视角：视觉板、力控板、编织/纱线区域都在同一个场景中。
2. 第一段看视觉区域：左臂和 `d3_marker_center`、`vision_target`。
3. 第二段看力控区域：右臂和 `d2_force_target_panel`、`d2_contact_pad`。
4. 第三段看编织区域：右臂和 `weft_yarn`、`d4_shuttle_lane`、`d4_tension_scale`。
5. 最后给整体视角，说明这是集成链路，不是单一动作。

### RViz 操作

RViz 中必须看到双臂 RobotModel 和整体轨迹。Gazebo 和 RViz 如果不同步，停止重来。

### 相机和话题

相机窗口：

| 阶段 | 相机 |
| --- | --- |
| 视觉阶段 | `/left_camera/image_rect` |
| 力控/纱线阶段 | `/right_camera/image_rect` |

话题终端：

```bash
ros2 topic echo /vision/status
ros2 topic echo /visual_servo/gazebo_adapter_state
ros2 topic echo /force_control/state
ros2 topic echo /force_control/gazebo_contact_force_state
ros2 topic echo /weaving/events
ros2 topic echo /weaving/tension_status
tail -f "$OUT/logs/physical_interaction_events.log"
```

### 合格标准

| 画面 | 必须成立 |
| --- | --- |
| Gazebo | 三个工位都在场景中，右臂接触/纱线动作和目标位置对上 |
| RViz | 双臂模型完整加载，运动和 Gazebo 同步 |
| 相机 | 左相机可支撑视觉，右相机可支撑接触/纱线 |
| 话题 | 视觉、力控、张力、attach/release 至少各有一个状态证据 |
| 讲解 | 讲“系统链路集成”，不要讲成所有模块同时强接触 |

## 9. 现场常见问题处理

| 问题 | 处理 |
| --- | --- |
| Gazebo 物体位置不对 | 停掉当天后端，重新跑当天命令；不要打开旧的 SDF 或旧输出目录 |
| Gazebo 有动作但 RViz 空白 | 检查 `/robot_description`、`/joint_states`、Fixed Frame；RViz 不显示机械臂就不要录 |
| RViz 和 Gazebo 不同步 | 确认 RViz 用 `use_sim_time:=false`，且没有启动旧的 planner/replay 节点 |
| D1 仍出现夹取放置叙事 | 停止重录，D1 只讲夹爪开合、TF、自碰撞矩阵、双臂同步/交替运动 |
| D3 相机看不到靶标 | 先看 `/left_camera/image_rect`，不要看 `evidence_camera`；若首帧没有绿色靶标，停止重来 |
| D2/D4 看起来像空跑 | Gazebo 视角拉近目标板/纱线，右相机和 force/tension 话题同时打开 |
| 老师问是不是 MoveIt 轨迹 | 回答：轨迹由 `moveit_plan_client` 调 `/move_action` 生成，Gazebo JointTrajectory 和 RViz `/joint_states` 都来自同一份 MoveIt plan |

录制前最后检查：

```bash
ros2 topic hz /joint_states
ros2 topic echo /acceptance/day_status --once
gz topic -l | grep rm65b_world
```

如果 RViz 没加载机械臂、Gazebo 目标物不在对应工位、手眼相机看不到当天目标，任何一个不满足都不要开始正式录屏。
