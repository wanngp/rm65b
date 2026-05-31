# Day 3 手眼标定、标志物识别与视觉伺服实验报告

## 0. 证据状态与禁止夸大表述

本报告当前交付的是 Gazebo 眼在手上相机安装矩阵、可调用 OpenCV `calibrateHandEye` 的标定脚本、OpenCV 标志物识别节点、视觉伺服控制节点和离线 IBVS 收敛计算程序。`outputs/d3_hand_eye/hand_eye_matrix.json` 的 `source` 为 `gazebo_configured_eye_in_hand_pose`，不是 easy_handeye2 或真实相机采样得到的外参结果。

因此现场汇报时不要说“已经完成真实手眼标定”或“视觉定位误差已经小于 +/-1 mm”。可以说“仿真手眼安装矩阵和标定代码已完成；真实外参、相机内参、像素误差、世界误差和多初始位置成功率需要现场采样后补入”。

## 1. 实验目标

Day 3 展示左臂“眼在手上”的视觉闭环能力，不再使用固定场景相机冒充视觉跟随。实验需要证明三件事：

1. 左夹具上的手眼相机一开始能看到靶标。
2. OpenCV 节点能从手眼相机图像中识别靶标，并发布标注后的识别视频。
3. 视觉伺服节点根据靶标偏差生成控制量，使左臂末端向靶标靠近并点到靶标区域。

相关实现文件：

| 内容 | 文件 |
| --- | --- |
| 手眼矩阵计算 | `rm65b_dual_arm_ws/scripts/d3_hand_eye_calibration.py` |
| 标志物识别节点 | `rm65b_dual_arm_ws/src/rm65b_vision_guidance/rm65b_vision_guidance/aruco_target_node.py` |
| 视觉伺服控制节点 | `rm65b_dual_arm_ws/src/rm65b_vision_guidance/rm65b_vision_guidance/ibvs_controller.py` |
| Gazebo 视觉伺服适配 | `rm65b_dual_arm_ws/src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/gazebo_trajectory_relays.py` |
| 视觉伺服计算程序 | `rm65b_dual_arm_ws/scripts/d3_visual_servo_experiment.py` |
| 正式 MoveIt 可视化脚本 | `rm65b_dual_arm_ws/scripts/run_day_visual.sh day03` |
| 诊断可视化脚本 | `rm65b_dual_arm_ws/scripts/run_day03_vision_visual.sh`，默认转入正式 MoveIt 路径 |
| 启动和录制链路 | `rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh` |

## 2. 手眼相机安装关系

D3 使用左臂夹具掌部上的 Gazebo eye-in-hand camera，图像话题为：

```text
/left_camera/image_rect
```

录制脚本中 D3 的相机位姿配置为：

```bash
LEFT_EIH_SENSOR_POSE="0.034 0 0.095 0 0.35 0.38"
LEFT_EIH_VISUAL_POSE="0.032 0 0.090 0 0.35 0.38"
```

其中 `LEFT_EIH_SENSOR_POSE` 表示相机传感器坐标系相对左夹具掌部坐标系的位姿，单位为 m 和 rad。D1 报告中已给出机械臂末端 `Link6` 到夹具掌部的安装偏移，本实验沿用：

```text
Link6 -> gripper_palm: x=0.018 m, y=0, z=0, rpy=0,0,0
gripper_palm -> camera: x=0.034 m, y=0, z=0.095 m, rpy=0,0.35,0.38
```

## 3. 手眼标定方法

真实系统中，眼在手上的手眼标定求解的是：

```text
A_i X = X B_i
```

其中 `X = ^G T_C` 为相机坐标系 `C` 相对夹具/末端坐标系 `G` 的刚体变换。采集多组末端运动和标定板观测后，可用 OpenCV 的 `calibrateHandEye` 求解。Gazebo 实验中相机是按 SDF 固定安装到夹具掌部的，因此可直接用 SDF 安装位姿作为仿真基准矩阵；若切换到真实相机，只需要把采集样本写入 JSON 后调用同一脚本的 OpenCV 分支。

### 3.1 D3 手眼位置转移矩阵

夹具掌部到相机：

```text
^G T_C =
[[ 0.872362, -0.370920, 0.318437, 0.034],
 [ 0.348433,  0.928665, 0.127188, 0.000],
 [-0.342898,  0.000000, 0.939373, 0.095],
 [ 0.000000,  0.000000, 0.000000, 1.000]]
```

相机到夹具掌部：

```text
^C T_G =
[[ 0.872362,  0.348433, -0.342898,  0.002915],
 [-0.370920,  0.928665,  0.000000,  0.012611],
 [ 0.318437,  0.127188,  0.939373, -0.100067],
 [ 0.000000,  0.000000,  0.000000,  1.000000]]
```

机械臂 `Link6` 到相机：

```text
^L6 T_C =
[[ 0.872362, -0.370920, 0.318437, 0.052],
 [ 0.348433,  0.928665, 0.127188, 0.000],
 [-0.342898,  0.000000, 0.939373, 0.095],
 [ 0.000000,  0.000000, 0.000000, 1.000]]
```

### 3.2 手眼标定代码

完整程序位置：

```bash
python3 rm65b_dual_arm_ws/scripts/d3_hand_eye_calibration.py \
  --output rm65b_dual_arm_ws/outputs/d3_hand_eye/hand_eye_matrix.json
```

核心代码如下：

```python
def pose_to_matrix(values):
    x, y, z, roll, pitch, yaw = [float(item) for item in values]
    r = rpy_to_matrix(roll, pitch, yaw)
    return [
        [r[0][0], r[0][1], r[0][2], x],
        [r[1][0], r[1][1], r[1][2], y],
        [r[2][0], r[2][1], r[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]

def solve_with_opencv(samples_path):
    data = json.loads(samples_path.read_text(encoding="utf-8"))
    samples = data.get("samples") or []
    r_gripper2base, t_gripper2base = [], []
    r_target2cam, t_target2cam = [], []
    for sample in samples:
        t_base_gripper = np.array(sample["T_base_gripper"], dtype=float)
        t_camera_target = np.array(sample["T_camera_target"], dtype=float)
        r_gripper2base.append(t_base_gripper[:3, :3])
        t_gripper2base.append(t_base_gripper[:3, 3])
        r_target2cam.append(t_camera_target[:3, :3])
        t_target2cam.append(t_camera_target[:3, 3])
    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
```

真实相机标定时，样本 JSON 每一帧包含：

```json
{
  "samples": [
    {
      "T_base_gripper": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
      "T_camera_target": [[1,0,0,0.25],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    }
  ]
}
```

## 4. OpenCV 标志物识别视频

识别节点输入左手眼相机图像：

```text
/left_camera/image_rect
```

识别节点输出：

| 话题 | 含义 |
| --- | --- |
| `/vision/target_pose` | 靶标在相机坐标系下的估计位置 |
| `/vision/status` | 识别状态、像素误差、帧率 |
| `/vision/metrics` | JSON 格式识别指标 |
| `/vision/debug_image` | OpenCV 标注后的识别视频 |

`/vision/debug_image` 会在图像上画出图像中心、靶标中心、中心连线、像素误差、识别方法和帧率。录制时应打开：

```bash
ros2 run rqt_image_view rqt_image_view /vision/debug_image
```

自动录制脚本会保存：

```text
$OUT/videos/day03_opencv_marker_debug.mp4
$OUT/screenshots/07_opencv_marker_debug.png
$OUT/logs/vision_debug_capture_summary.txt
```

现场先看效果时，可不进入完整录制流程，直接运行：

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day03
```

该脚本生成高位视觉板和绿色靶标，启动左手眼相机、OpenCV 识别和 IBVS
topic 链路，并让左臂做 search-align-approach 循环。它用于快速确认
“相机看见目标”和“左臂在对准目标运动”这两个画面是否清楚。

## 5. 视觉伺服原理

识别节点把像素偏差转换为相机坐标系下的靶标位置：

```text
Z = target_depth_m
y = -(u - cx) * Z / fx
z = -(v - cy) * Z / fy
```

视觉伺服节点使用图像式视觉伺服的简化控制律：

```text
v_y = sat(-lambda_y * y)
v_z = sat(-lambda_z * z)
```

当前参数：

```text
lambda_y = 0.8
lambda_z = 0.5
max_linear_mps = 0.03
target_tolerance_m = 0.001
```

在 Gazebo 验收环境中，`visual_servo_gazebo_adapter` 将 `/visual_servo/twist_cmd` 转成左臂小步关节轨迹并发布到：

```text
/model/left_rm65b/joint_trajectory
/visual_servo/left_corrected_joint_trajectory
```

这条链路不是预先编排物体运动，而是由手眼相机识别结果驱动：

```text
/left_camera/image_rect
  -> aruco_target_node
  -> /vision/target_pose
  -> ibvs_controller
  -> /visual_servo/twist_cmd
  -> visual_servo_gazebo_adapter
  -> /model/left_rm65b/joint_trajectory
```

### 5.1 视觉伺服实验代码

控制节点核心代码：

```python
error_y = msg.pose.position.y
error_z = msg.pose.position.z
norm = math.sqrt(error_y * error_y + error_z * error_z)
aligned = norm <= self.tolerance

twist.twist.linear.y = self._clamp(-self.gain_xy * error_y)
twist.twist.linear.z = self._clamp(-self.gain_z * error_z)
if aligned:
    twist.twist.linear.y = 0.0
    twist.twist.linear.z = 0.0
```

Gazebo 适配代码：

```python
dy = float(self.latest_twist.twist.linear.y)
dz = float(self.latest_twist.twist.linear.z)
positions = list(self.latest_left)
positions[0] += self._clamp(dy * self.gain_j1)
positions[1] += self._clamp(dz * self.gain_j2)
self.pub.publish(traj)
self.corrected_pub.publish(ros_traj)
```

离线计算程序：

```bash
python3 rm65b_dual_arm_ws/scripts/d3_visual_servo_experiment.py \
  --output rm65b_dual_arm_ws/outputs/d3_visual_servo/ibvs_trace.csv
```

该程序输出像素误差、相机坐标误差、`twist_cmd` 和关节修正量的时间序列，用于报告中解释视觉伺服收敛过程。

## 6. D3 录制操作要求

启动：

```bash
DAY=day03
DOMAIN=213
OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 SYNC_REPLAY_START_DELAY=8.0 MOVEIT_SEGMENT_DURATION=10.0 CAPTURE_TIMEOUT=380 \
bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

脚本会生成的 D3 视频文件：

| 文件 | 用途 |
| --- | --- |
| `$OUT/videos/day03_opencv_marker_debug.mp4` | OpenCV 标志物识别视频 |
| `$OUT/videos/day03_left_gazebo_camera.mp4` | 左手眼原始相机视频 |
| `$OUT/videos/day03_moveit_harmonic_playback.mp4` | Gazebo 视觉伺服实验视频 |
| `$OUT/videos/day03_rviz.mp4` | RViz 视觉伺服实验视频，需 `RECORD_RVIZ=1` |

录屏窗口建议：

| 窗口 | 内容 |
| --- | --- |
| Gazebo | 左臂、左夹具、视觉板、`vision_target` |
| RViz | 双臂 RobotModel、左臂轨迹、TF |
| rqt_image_view | `/vision/debug_image` |
| 终端 1 | `ros2 topic echo /vision/status` |
| 终端 2 | `ros2 topic echo /visual_servo/twist_cmd` |
| 终端 3 | `ros2 topic echo /visual_servo/gazebo_adapter_state` |

必须录到的证据：

1. `/vision/debug_image` 首帧可见靶标，不允许一开始黑屏或看不到目标。
2. OpenCV 标注画面中有靶标中心和像素误差。
3. `/vision/target_pose` 有非零 `y/z` 偏差。
4. `/visual_servo/twist_cmd` 随偏差产生控制量。
5. Gazebo 中左臂末端靠近靶标，RViz 中左臂运动与 Gazebo 对齐。
6. 若要证明点到靶标，查看 `vision_target` 接触话题：

```bash
ros2 topic echo /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact
```

## 7. 验收判断

合格视频应能回答老师的四个问题：

| 老师可能问 | 视频和报告回答 |
| --- | --- |
| 相机在哪里？ | 左夹具掌部 eye-in-hand camera，矩阵见第 3 节 |
| 标定矩阵怎么来的？ | Gazebo 用安装位姿，真实系统用 `calibrateHandEye`，代码见第 3.2 节 |
| 有没有真的识别靶标？ | `/vision/debug_image` OpenCV 标注视频和 `/vision/status` |
| 机械臂为什么会动？ | `/vision/target_pose -> /visual_servo/twist_cmd -> /model/left_rm65b/joint_trajectory` 闭环链路 |
