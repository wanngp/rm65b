# D1-D5 录制结果严格检查结论（2026-05-29 历史版）

> 状态说明：本文记录的是旧版 D1-D5 录制视频的严格抽帧审计，保留用于说明“为什么不能把演示编排当成验收证据”。后续 D1 验收内容已调整为夹爪开合、TF、自碰撞矩阵、双臂同步/交替运动，不再要求物块夹取交接。当前交付状态、缺口和正式汇报口径以 `docs/technical_experiment_delivery_audit.md` 为准。

检查时间：2026-05-29  
检查对象：

- D1 修正版：`outputs/rm65b_day01_center_handoff_fix_20260529_001929/day01`
- D2-D5：`outputs/rm65b_five_day_sync_moveit_recording_20260528_225304/day02` 至 `day05`
- 抽帧对照：`E:/rm65b_strict_audit_20260529/contact_sheets`

## 总结论

这版不建议整体直接交给老师作为最终展示版。机器人关节轨迹确实来自 MoveIt2 `/move_action`，但 Gazebo 中的物体、组件、纱线、视觉目标主要由场景编排脚本用 `gz set_pose` 跟随或摆放，不是 Gazebo 物理抓取、接触或 MoveIt 规划出的物体运动。RViz 只证明双臂关节姿态同步，不能证明 Gazebo 物体运动与机械臂接触真实一致。

## 逐日判定

| 实验 | 严格判定 | 主要问题 |
| --- | --- | --- |
| D1 物体交接（旧版范围，当前 D1 不采用） | 有改进，但只能条件通过 | 机械臂会按 MoveIt 轨迹运动，物体从左侧到右侧的过程比旧版合理；但方块仍是脚本移动，不是 Gazebo 物理夹取/吸附。中间交接阶段仍可能被老师质疑“物体为什么跟着走”。 |
| D2 力控接触 | 不建议作为最终版 | 主 Gazebo 视角中接触点被板件遮挡，不能清楚证明机械臂压到目标；右手眼在手相机看到夹爪和红色结构，但仍不足以证明力控闭环。一次性 topic 证据文件多数无效。 |
| D3 视觉伺服 | 不通过，必须重录 | 左手眼在手相机没有清楚看到视觉目标/标靶，画面主要是地面、夹爪和红色结构。老师问“相机看到的目标在哪里”时无法回答。视觉相关一次性证据文件为空。 |
| D4 示教/编织 | 相对可用，但仍需说明边界 | 编织架、导轨、纱线和双臂动作可见，`day04_recorded_primitives_from_live_joint_states.yaml` 能证明来自 live joint states 的示教记录；但纱线/接触探针仍是脚本跟随，不是物理拉紧。 |
| D5 综合演示 | 条件可用，展示效果一般 | 综合场景元素较全，但主视角拥挤，细节遮挡明显；眼在手相机能看到部分导轨/纱线/结构，但无法清楚支撑“视觉+力控+编织”全部闭环。 |

## 证据链问题

- 五天 `dual_moveit_summary.json` 均显示 `source: MoveIt2 /move_action true dual_arms group`，说明双臂关节目标是 MoveIt2 规划得到的。
- `sync_moveit_scene_choreography.py` 中大量使用 `run_gz_set_pose(...)` 驱动物体，例如 D1 方块、D2 接触探针、D3 视觉目标、D4/D5 纱线和探针，因此这些物体不是被 Gazebo 物理夹持或接触自然带动。
- 五天 rosbag 目录下均有 `.db3`，但缺少 `metadata.yaml`。当前不能稳定用 `ros2 bag info` 作为验收证据。
- 很多 `*_once.txt` 是空文件或 `rcl context invalid` 错误，不能作为 topic 证据提交。
- RViz 当前主要展示 RobotModel/TF，同步的是同一份 MoveIt 关节回放；RViz 里没有 Gazebo 道具，因此不能用 RViz 证明物体接触、夹取、力控或视觉目标同步。

## 建议修复优先级

1. 先修 D3：重新摆放视觉标靶和眼在手相机外参，确保左/右相机视频中能清楚看到目标，并同步保留视觉 topic 证据。
2. 再修 D2：调整主摄像机或力控工位，使“接近-接触-压下-释放”在主 Gazebo 视频中可见；同步修复 force/contact topic 采样时机。
3. 旧版 D1 如需作为“真实抓取交接”展示，需要 Gazebo attach/contact 机制或至少明确标注为可视化跟随；当前 D1 已改为夹爪开合、TF、自碰撞矩阵、双臂同步/交替运动，不采用该旧范围。
4. 修复 rosbag 结束流程，确保 `metadata.yaml` 正常生成，或对已有 bag 执行 reindex 后再提交。
5. 修复 `*_once.txt` 采样：必须在节点仍运行、topic 正在发布时采样，不能在关闭阶段采样。
