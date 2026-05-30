# 双臂五天实验录制修复交接记录（暂停点，历史记录）

> 状态说明：本文是 2026-05-29 针对旧版“夹取、交接、放置”视频问题的交接记录。后续 D1 验收内容已调整为“夹爪开合能力、夹爪/末端 TF、自碰撞矩阵、双臂同步/交替运动”，不再要求 D1 夹取放置。因此本文只能作为历史问题排查记录，不能作为当前 D1-D5 技术实验清单的最终验收口径。当前交付状态以 `docs/technical_experiment_delivery_audit.md` 为准。

记录时间：2026-05-29  
当前状态：暂停后续录制，方便下一轮直接继续。

## 用户硬性要求

- D1-D5 每一天都不能再使用“演示编排”或动态 `set_pose` 摆拍来驱动物体。
- D1 如果要讲真实抓取交接，必须有 attach/contact 机制，不能出现“机械臂没夹住，物体自己动”。
- D2 要修接触可见性，并补充 topic 证据采样。
- D3 要优先修相机/视觉目标，尤其眼在手上的相机视角不能奇怪。
- 重录后必须自己抽帧/跳看检查，严格按“给老师看会被问什么”来验收。
- RViz 和 Gazebo 必须对得上：机器人运动、物体运动、接触/抓取逻辑都要能解释。

## 已完成的主要修改

### 1. 运行时 attach 机制

新增文件：

- `rm65b_dual_arm_ws/scripts/rm65b_runtime_link_attacher.cc`

作用：

- Gazebo Sim System plugin。
- 通过 `/rm65b/physical/<name>/attach` 和 `/rm65b/physical/<name>/detach` 控制固定关节。
- 使用 `gz::sim::components::DetachableJoint` 在机械臂手爪 palm link 和物体 link 之间建立/移除固定连接。
- 不调用 `/world/*/set_pose`。

已验证编译命令：

```bash
wsl -d Ubuntu-22.04-ROS2 bash -lc 'cd "/mnt/e/1-项目/睿尔曼" && mkdir -p /tmp/rm65b_attacher_test && g++ -std=c++17 -fPIC -shared rm65b_dual_arm_ws/scripts/rm65b_runtime_link_attacher.cc -o /tmp/rm65b_attacher_test/librm65b_runtime_link_attacher.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2 gz-transport13 gz-msgs10)'
```

### 2. 物理交互控制器

新增文件：

- `rm65b_dual_arm_ws/scripts/physical_interaction_controller.py`

作用：

- 控制夹爪目标文件。
- 发布 attach/detach 指令。
- 明确不调用 `/world/*/set_pose`。
- D1：左爪夹持方块、左爪释放、右爪接管、右爪释放。
- D2：围绕接触动作控制夹爪开合。
- D4/D5：右爪 attach/detach `weft_yarn` 和 `d4_shuttle_contact_probe`。
- D3：只处理夹爪状态，不驱动物体。

### 3. Gazebo 场景生成

修改文件：

- `rm65b_dual_arm_ws/scripts/generate_day_world.py`

已做修改：

- `add_box_model` 支持动态模型、质量、摩擦、重力开关和 contact sensor。
- D1 方块改为 dynamic SDF body，并放到左臂可接触的高位区域。
- D2 接触目标、接触垫、接触探针移动到右臂 TCP 可接触区域，`d2_contact_pad` 加 contact sensor。
- D3 视觉板、marker、`vision_target` 移到左臂眼在手相机可观察区域。
- D3 最新源代码里 `vision_target`/board 已上移到 `z=0.740`，但这个最新改动尚未同步到 staging workspace，也尚未复测。
- D4/D5 织布场景高位化，`weft_yarn` 改为 dynamic 且关闭重力，准备由 attach 机制带动。

### 4. D2 topic 证据

修改文件：

- `rm65b_dual_arm_ws/src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/gazebo_contact_force_estimator.py`
- `rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh`

已加入 D2 contact sensor topic：

```text
/world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact
```

录制脚本中已加入 bridge / record / once sampling 的相关条目。

### 5. 主录制脚本禁用旧摆拍路径

修改文件：

- `rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh`

已做修改：

- 增加 `ALLOW_LEGACY_CHOREOGRAPHY=0` 默认值。
- 如果不是 MoveIt/Gazebo planned trajectory，且未显式打开 legacy，则直接退出。
- 编译并注入 `rm65b::RuntimeLinkAttacher` plugin。
- D1 注入 `d1_left_block`、`d1_right_block` attach 配置。
- D4/D5 注入 `d4_right_yarn`、`d4_right_probe` attach 配置。
- 主路径用 `physical_interaction_controller.py` 替代 `sync_moveit_scene_choreography.py`。
- rosbag 停止后增加等待，避免缺失 `metadata.yaml`。
- evidence sampling 改为播放期间采样，不再等所有 topic 关闭后才采。
- day02/day03/day05 左侧眼在手相机 yaw 调整为 `0.75`。

注意：

- 主路径已禁用动态 `set_pose`。
- 但 `play_harmonic_planned_record.sh` 里仍残留旧的不可达 legacy 函数文本，包含 `set_pose_xyz` 调用。
- `sync_moveit_scene_choreography.py` 旧文件仍存在，里面大量 `run_gz_set_pose`，主录制路径不再调用它。
- `play_harmonic_record.sh` 和 `harmonic_fk_player.py` 也属于旧/其他路径，仍有 `set_pose`。最终交付前要明确不使用这些路径，最好清理或隔离。

## 已做验证

Python / Bash 检查已通过：

```bash
python3 -m py_compile rm65b_dual_arm_ws/scripts/generate_day_world.py rm65b_dual_arm_ws/scripts/physical_interaction_controller.py rm65b_dual_arm_ws/scripts/replay_moveit_plan_joint_states.py
bash -n rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh rm65b_dual_arm_ws/scripts/record_moveit_days_harmonic.sh
```

staging workspace：

```text
/mnt/e/rm65b_ws_recording
```

已同步并构建过：

```bash
wsl -d Ubuntu-22.04-ROS2 bash -lc 'cd /mnt/e/rm65b_ws_recording && source /opt/ros/humble/setup.bash && colcon build --base-paths src --packages-select rm_ros_interfaces rm65b_dual_arm_planning rm65b_vision_guidance --symlink-install'
```

说明：

- 该构建已经完成 3 个 package，但命令曾因 120s timeout 在 summary 后退出。
- 后续通过 `ros2 pkg prefix` 验证 staging install 可用。
- 默认 WSL distro 不是正确环境，必须使用 `Ubuntu-22.04-ROS2`。

## D3 快速试录结果

旧 D3 试录输出目录：

```text
E:\rm65b_physical_test_day03_20260529_011909\day03
```

结果：

- `day03_moveit_harmonic_playback.mp4` 已生成。
- `day03_left_gazebo_camera.mp4` 已生成。
- `day03_right_gazebo_camera.mp4` 已生成。
- 该次试录没有录 RViz，因为用了 `RECORD_RVIZ=0`。
- rosbag 已正常生成 `metadata.yaml`。

关键证据：

```text
vision_status_once.txt:
data: target method=salient_dark u=392.4 v=416.5 pixel_error_x=72.4 pixel_error_y=176.5 fps=14.94
```

抽帧结论：

- 左眼在手相机能看到目标，但目标偏低偏左，画面不够理想。
- 因此后续已把 D3 目标/板上移到 `z=0.740`，但尚未重新同步 staging 和复测。

## 未完成事项

1. 同步最新源代码到 staging workspace  
   最新 D3 `z=0.740` 修改还没有同步到 `/mnt/e/rm65b_ws_recording`。

2. 清理 legacy 摆拍代码  
   主路径已经禁用，但文件里仍能 grep 到旧的 `set_pose_xyz` 调用。建议直接删除 `play_harmonic_planned_record.sh` 中不可达 legacy 编排函数，或把旧脚本移入明确的 deprecated 目录，避免老师/验收时看到“还有 set_pose”。

3. 增加 RuntimeLinkAttacher 状态采样  
   目前有 `physical_interaction_events.log` 和 Gazebo plugin log，但还没有把 `/rm65b/physical/<name>/state` 持续采样到日志。建议在 `start_evidence_sampling` 中增加：
   - D1：`d1_left_block`、`d1_right_block`
   - D4/D5：`d4_right_yarn`、`d4_right_probe`

4. 重新试录 D3  
   目标：确认左眼在手相机中视觉目标位于合理区域，`vision_status_once.txt` 有稳定输出，不再贴边或视角奇怪。

5. 重录 D1-D5 全量视频  
   每一天都要录 Gazebo、RViz、左/右眼在手相机和 rosbag。

6. 严格抽帧验收  
   不能只看文件是否生成。必须检查关键帧/contact sheet：
   - D1 是否真的夹起、交接、放下。
   - D2 是否真的发生可见接触，topic 是否有 contact/force 证据。
   - D3 视觉目标是否在眼在手相机画面中，且 RViz/Gazebo 对齐。
   - D4/D5 纱线/探针是否由机械臂 attach/contact 带动，不是自己飞。
   - 所有天数 RViz 机器人姿态和 Gazebo 机器人姿态是否一致。

7. 更新当时版本的视频审计报告  
   建议新建或更新：
   - `docs/strict_video_audit_20260529.md`
   - 或新增最终版 `docs/final_recording_audit_20260529.md`

## 下一轮建议命令

### 1. 同步最新源到 staging

```bash
wsl -d Ubuntu-22.04-ROS2 bash -lc 'mkdir -p /mnt/e/rm65b_ws_recording && rsync -a --exclude build --exclude install --exclude log "/mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/" /mnt/e/rm65b_ws_recording/'
```

如果只改脚本和 world generator，通常不需要重编 ROS package；如果改了 package 内 Python 节点，建议重新 build。

### 2. 重新做基本检查

```bash
wsl -d Ubuntu-22.04-ROS2 bash -lc 'cd "/mnt/e/1-项目/睿尔曼" && python3 -m py_compile rm65b_dual_arm_ws/scripts/generate_day_world.py rm65b_dual_arm_ws/scripts/physical_interaction_controller.py && bash -n rm65b_dual_arm_ws/scripts/play_harmonic_planned_record.sh rm65b_dual_arm_ws/scripts/record_moveit_days_harmonic.sh'
```

### 3. D3 快速试录

PowerShell 中建议使用单字符串 `ArgumentList`，避免 `bash -lc` 参数被拆坏：

```powershell
$root = "/mnt/e/rm65b_physical_test_day03_retry_$(Get-Date -Format yyyyMMdd_HHmmss)"
$cmd = "mkdir -p $root && source /opt/ros/humble/setup.bash && source install/setup.bash && export DISPLAY=:0 && RECORD_RVIZ=0 CAPTURE_TIMEOUT=130 bash scripts/play_harmonic_planned_record.sh day03 $root/day03 220"
$args = "-d Ubuntu-22.04-ROS2 --cd /mnt/e/rm65b_ws_recording bash -lc `"$cmd`""
Start-Process -FilePath "wsl.exe" -ArgumentList $args -WindowStyle Hidden
```

### 4. 全量重录

建议输出到 ASCII 路径，避免中文路径导致 WSL/Gazebo 日志或脚本编码问题：

```powershell
$stamp = Get-Date -Format yyyyMMdd_HHmmss
$rootWsl = "/mnt/e/rm65b_physical_no_choreography_$stamp"
$cmd = "mkdir -p $rootWsl && source /opt/ros/humble/setup.bash && source install/setup.bash && export DISPLAY=:0 && RECORD_RVIZ=1 CAPTURE_TIMEOUT=190 bash scripts/record_moveit_days_harmonic.sh $rootWsl /mnt/e/rm65b_ws_recording 240 > $rootWsl/record_all.stdout.log 2> $rootWsl/record_all.stderr.log"
$args = "-d Ubuntu-22.04-ROS2 --cd /mnt/e/rm65b_ws_recording bash -lc `"$cmd`""
Start-Process -FilePath "wsl.exe" -ArgumentList $args -WindowStyle Hidden
```

## 当时版本的视频验收标准（历史）

### D1

- 方块一开始在左爪可达位置。
- 左爪闭合后方块跟随左臂运动。
- 交接时不能出现“两臂不动，方块自己飞到右手”的情况。
- 右爪接管后方块跟随右臂运动并放下。
- attach/detach 日志和画面时序一致。

### D2

- Gazebo 画面中必须看到机械臂和接触目标发生接触或压靠。
- `d2_contact_pad_contact_once.txt` 不能是空证据。
- force/control/status topic 要能说明正在做接触/柔顺/力控相关实验。

### D3

- 左/右机械臂相机是眼在手上，不是漂在场景里的奇怪 camera。
- 左眼在手相机中能看到视觉目标，目标不能贴边、不能只露一小块。
- `/vision/target_pose`、`/vision/status` 要有有效输出。

### D4/D5

- 纱线、探针等物体不能自己运动。
- 物体移动必须能解释为 Gazebo collision/contact 或 RuntimeLinkAttacher 固定关节带动。
- 如果展示双臂协同，画面中要能看到双臂参与，而不是一个机械臂空动。

### 全部天数

- RViz 和 Gazebo 中的机器人姿态要大体同步。
- 动作不能僵硬到像手写关键帧摆拍；要能说明机器人轨迹来自 MoveIt/Gazebo planned playback。
- rosbag 每天都要有 `metadata.yaml`。
- Gazebo、RViz、左右 eye-in-hand camera 视频都要完整生成并抽帧检查。

## 已知风险

- Runtime attach 是工程化抓取近似：用固定关节表达夹持，不等同于纯摩擦自然抓取。讲解时应表述为“Gazebo 物理场景中通过夹爪闭合触发 attach 机制模拟稳定抓取/交接”，不要说成完全依靠摩擦自发抓起。
- 物体和 TCP 的相对位置仍可能需要根据重录视频微调，尤其 D1 交接和 D2 接触。
- rosbag 文件很大，D3 试录单天 db3 已接近 GB 级，重录前确认磁盘空间。
- 如果继续使用旧脚本路径，仍可能出现 `set_pose` 摆拍；最终录制必须限定为 `play_harmonic_planned_record.sh` + `record_moveit_days_harmonic.sh` 的新主路径。
