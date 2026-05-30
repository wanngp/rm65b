# Day 1 夹爪、末端转换、自碰撞矩阵与双臂运动规划报告

## 0. 证据状态与禁止夸大表述

本报告当前交付的是 Gazebo 夹爪模型/命令 topic、夹爪相对 `Link6` 的固定转换矩阵、MoveIt SRDF 自碰撞矩阵说明、以及双臂同步/交替运动规划链路。尚未完成真机夹爪安装后的开合延时实测、真实 TCP 重复定位误差实测、全行程真机无碰撞验证。

因此现场汇报时不要说“夹爪延时已经小于 100 ms”或“TCP 重复精度已经小于 +/-0.5 mm”。可以说“仿真和配置链路已具备，真机指标需要录制现场 topic 与测量数据后补入”。

## 1. 结论

Day 1 不再展示物块夹取放置，改为验证四项基础能力：

| 验证项 | 视频证据 | 报告证据 |
| --- | --- | --- |
| 夹爪开合能力 | Gazebo 中夹爪有打开和闭合两个状态 | 夹爪 SDF/控制 topic/行程参数 |
| 夹爪与机械臂末端相对位姿 | RViz 显示 `Link6` 与夹爪 TF，Gazebo 看到夹爪固定在末端 | `Link6 -> gripper -> TCP` 转移矩阵 |
| 双臂自碰撞矩阵 | RViz 中双臂模型完整，规划过程不穿模 | SRDF Allowed Collision Matrix |
| 双臂同步/交替运动 | Gazebo 与 RViz 中先双臂同步，再左右交替 | `dual_arms` MoveIt 规划组和 D1 stage 设计 |

## 2. 视频展示流程

| 顺序 | 画面 | 要说明的话 |
| --- | --- | --- |
| 1 | Gazebo 整体视角，左右臂和夹爪可见 | D1 是夹爪、TF、碰撞矩阵和双臂运动能力验证 |
| 2 | 拉近夹爪，展示打开状态 | 打开命令为 `0.018 m` |
| 3 | 夹爪闭合，展示闭合状态 | 闭合命令为 `0.004 m` |
| 4 | RViz 显示双臂 RobotModel 和 TF | 夹爪固定安装在 `Link6` 后方，转移矩阵写在报告中 |
| 5 | 双臂同时运动 | 对应 `d1_sync_spread_both_arms`、`d1_sync_shift_both_arms` |
| 6 | 左臂动右臂保持、右臂动左臂保持 | 对应 `d1_alternate_left_moves_right_holds`、`d1_alternate_right_moves_left_holds` |

## 3. 夹爪如何配置

Gazebo 中的夹爪由录制启动脚本在 RM65-B SDF 上动态补入，配置来源为 `scripts/play_harmonic_planned_record.sh`。

| 配置项 | 当前值 | 作用 |
| --- | --- | --- |
| 夹爪掌部 link | `attached_scaled_gripper_palm` | 固定到机械臂 `Link6` |
| 掌部相对 `Link6` 位姿 | `0.018 0 0 0 0 0` | 夹爪基准坐标系前移 18 mm |
| 上指 link | `attached_scaled_gripper_upper_finger` | 相对掌部初始 `0.080 0.024 0` |
| 下指 link | `attached_scaled_gripper_lower_finger` | 相对掌部初始 `0.080 -0.024 0` |
| 上指关节 | `gripper_upper_slide` | prismatic，轴向 `0 1 0` |
| 下指关节 | `gripper_lower_slide` | prismatic，轴向 `0 -1 0` |
| 行程范围 | `[0.000, 0.018] m` | 控制夹爪开合 |
| 控制器 | `gz::sim::systems::JointPositionController` | Gazebo 关节位置控制 |
| 命令 topic | `/rm65b_gripper/upper_finger_cmd`、`/rm65b_gripper/lower_finger_cmd` | ROS Float64 经 ros_gz_bridge 转 Gazebo Double |

D1 录制中，`scripts/physical_interaction_controller.py` 只做夹爪命令切换，不再发送物块 attach/detach：

| 状态 | 命令值 | 日志关键词 |
| --- | --- | --- |
| 打开 | `0.018 m` | `d1_gripper_open_both`、`d1_gripper_final_open` |
| 闭合 | `0.004 m` | `d1_gripper_close_both`、`d1_gripper_close_during_dual_motion` |

## 4. 夹爪和机械臂末端相对位置怎么求

当前仿真中的相对位姿来自 URDF/SDF 固定关节和夹爪几何定义。机械臂末端法兰使用 `Link6`，夹爪基准坐标系使用 `attached_scaled_gripper`/`attached_scaled_gripper_palm`。配置来源为 `scripts/generate_dual_rm65b_moveit_config.py` 和 `scripts/play_harmonic_planned_record.sh`。

从 `Link6` 到夹爪基准坐标系：

```text
T_Link6_gripper =
[ 1  0  0  0.018 ]
[ 0  1  0  0.000 ]
[ 0  0  1  0.000 ]
[ 0  0  0  1.000 ]
```

夹爪基准坐标系到 TCP 近似点，取夹爪前端 `yarn_hook` 中心：

```text
T_gripper_tcp =
[ 1  0  0  0.112 ]
[ 0  1  0  0.000 ]
[ 0  0  1  0.000 ]
[ 0  0  0  1.000 ]
```

因此 `Link6` 到 TCP 的组合变换为：

```text
T_Link6_tcp = T_Link6_gripper * T_gripper_tcp

T_Link6_tcp =
[ 1  0  0  0.130 ]
[ 0  1  0  0.000 ]
[ 0  0  1  0.000 ]
[ 0  0  0  1.000 ]
```

求解步骤：

1. 读取 URDF/SDF 中固定关节 `Link6 -> attached_scaled_gripper` 的 `origin/pose`。
2. 读取夹爪几何中前端 TCP 近似点相对夹爪基准坐标系的位置。
3. 将两个齐次变换矩阵相乘，得到 `Link6 -> TCP`。
4. 在 RViz 中打开 TF 显示，检查 `left_Link6/right_Link6` 与 `left_attached_scaled_gripper/right_attached_scaled_gripper` 是否同轴且相对位置固定。

## 5. 双臂自碰撞矩阵怎么求

自碰撞矩阵来自 MoveIt SRDF：`rm65b_dual_arm_ws/src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.srdf`。

生成方法：

1. 从官方 RM65 单臂 SRDF 读取 `disable_collisions`。
2. 分别给左臂和右臂 link 加 `left_`、`right_` 前缀，生成两套单臂自碰撞忽略项。
3. 对夹爪固定工具增加 `Link5/Link6` 与 `attached_scaled_gripper` 的相邻工具忽略项。
4. 不添加任何 left/right 跨臂 `disable_collisions`，因此双臂之间默认全部参与碰撞检查。
5. MoveIt 规划时由 PlanningScene/FCL 对未禁用的 link pair 做碰撞检测。

图例：

| 符号 | 含义 |
| --- | --- |
| `A` | Adjacent，相邻 link，禁用碰撞 |
| `N` | Never，采样中不发生有效碰撞，禁用碰撞 |
| `T` | AdjacentTool，工具与末端相邻，禁用碰撞 |
| `C` | Checked，保留碰撞检测 |

左臂和右臂各自使用同一套单臂矩阵：

| link | base_link | Link1 | Link2 | Link3 | Link4 | Link5 | Link6 | attached_scaled_gripper |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| base_link | - | A | N | N | C | C | C | C |
| Link1 | A | - | A | N | N | C | C | C |
| Link2 | N | A | - | A | N | C | C | C |
| Link3 | N | N | A | - | A | N | N | C |
| Link4 | C | N | N | A | - | A | N | C |
| Link5 | C | C | C | N | A | - | A | T |
| Link6 | C | C | C | N | N | A | - | T |
| attached_scaled_gripper | C | C | C | C | C | T | T | - |

跨臂矩阵没有禁用项，全部保留碰撞检测：

| left \ right | base_link | Link1 | Link2 | Link3 | Link4 | Link5 | Link6 | attached_scaled_gripper |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| base_link | C | C | C | C | C | C | C | C |
| Link1 | C | C | C | C | C | C | C | C |
| Link2 | C | C | C | C | C | C | C | C |
| Link3 | C | C | C | C | C | C | C | C |
| Link4 | C | C | C | C | C | C | C | C |
| Link5 | C | C | C | C | C | C | C | C |
| Link6 | C | C | C | C | C | C | C | C |
| attached_scaled_gripper | C | C | C | C | C | C | C | C |

报告中应说明：该矩阵是 MoveIt Allowed Collision Matrix，不是说机器人不会碰撞；它定义的是哪些 link pair 不需要检查，未禁用的 pair 仍由 MoveIt/FCL 检测。

## 6. 双臂运动规划怎么配置

MoveIt 配置文件：

| 文件 | 作用 |
| --- | --- |
| `config/rm65b_dual_arm.urdf` | 双臂 URDF，包含 `world`、左右固定基座、左右 RM65-B、固定夹爪 |
| `config/rm65b_dual_arm.srdf` | MoveIt 规划组、自碰撞矩阵、ready 状态 |
| `config/ompl_planning.yaml` | OMPL planner 配置 |
| `config/moveit_controllers.yaml` | 左右 FollowJointTrajectory controller 配置 |
| `launch/move_group.launch.py` | 启动 MoveIt move_group |
| `launch/moveit_rviz.launch.py` | 启动 RViz RobotModel/TF 显示 |

SRDF 中定义三个规划组：

| 规划组 | 内容 |
| --- | --- |
| `left_arm` | `left_base_link -> left_Link6` |
| `right_arm` | `right_base_link -> right_Link6` |
| `dual_arms` | `left_arm + right_arm` |

OMPL 配置使用：

| 项目 | 当前值 |
| --- | --- |
| planning plugin | `ompl_interface/OMPLPlanner` |
| planner | `RRTConnectkConfigDefault` |
| 双臂 projection | `joints(left_joint1,left_joint2,right_joint1,right_joint2)` |

D1 使用 `dual_moveit_plan_client` 对 `dual_arms` 组调用 MoveIt `/move_action`。轨迹阶段如下：

| stage | 左臂 | 右臂 | 视频含义 |
| --- | --- | --- | --- |
| `d1_sync_spread_both_arms` | 运动 | 运动 | 双臂同步运动 |
| `d1_sync_shift_both_arms` | 运动 | 运动 | 双臂同步换位 |
| `d1_alternate_left_moves_right_holds` | 运动 | 保持 | 左臂单独动作 |
| `d1_alternate_right_moves_left_holds` | 保持 | 运动 | 右臂单独动作 |
| `d1_return_dual_ready` | 运动 | 运动 | 双臂回准备位 |

规划完成后，脚本把同一份 MoveIt plan 同时用于：

| 输出 | 用途 |
| --- | --- |
| Gazebo `/model/left_rm65b/joint_trajectory`、`/model/right_rm65b/joint_trajectory` | 驱动 Gazebo 中左右机械臂 |
| ROS `/joint_states` | 驱动 RViz RobotModel |
| `/dual_arm_planning/phase` | 展示当前动作阶段 |

因此老师问“是不是 MoveIt 生成的轨迹”时，可以回答：D1 的双臂轨迹由 MoveIt2 `/move_action` 针对 `dual_arms` 规划组生成，Gazebo 与 RViz 使用同一份规划结果回放。

## 7. 复现实验命令

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

export OPS_ROOT=/mnt/e/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
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

另开 Gazebo GUI：

```bash
gz sim -g
```

另开 RViz：

```bash
ros2 launch rm65b_dual_arm_moveit_config moveit_rviz.launch.py use_sim_time:=false
```

现场检查：

```bash
ros2 topic echo /dual_arm_planning/phase
ros2 topic echo /rm65b_gripper/upper_finger_cmd
ros2 topic echo /rm65b_gripper/lower_finger_cmd
ros2 topic hz /joint_states
```
