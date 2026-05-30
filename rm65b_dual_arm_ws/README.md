# RM65-B dual-arm ROS 2 workspace

This workspace is built around the official RealMan Humble package and a
small layer of experiment packages.

## Packages

- `ros2_rm_robot`: official RealMan ROS 2 Humble source, version V1.6.0 from
  the downloaded humble branch archive.
- `rm65b_dual_arm_bringup`: launch files and unified parameters.
- `rm65b_gripper_control`: `control_msgs/action/GripperCommand` compatible
  gripper server with simulation and protocol placeholders.
- `rm65b_safety`: watchdog, force limit, joint limit, and stop-topic bridge.
- `rm65b_vision_guidance`: ArUco/AprilTag-style target pose and IBVS scaffold.
- `rm65b_weaving_primitives`: weaving primitive trajectories and coordinator.

## Notes

The official RealMan driver publishes topics under `rm_driver/...`. The dual
arm launch starts each driver in a namespace, yielding:

- `/left_rm_driver/rm_driver/...`
- `/right_rm_driver/rm_driver/...`

The official force-position API topic name contains the upstream spelling
`set_force_postion_cmd`; this workspace keeps that spelling for compatibility.

The repository path contains Chinese characters. The custom Python packages
build there, but official `rm_ros_interfaces` can fail in `rosidl` path
handling. Use an ASCII-only workspace path for full official-driver builds.

## VMware Ubuntu bootstrap

Run the bootstrap script inside the Ubuntu VM:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh
```

The script checks for the official RealMan `ros2_rm_robot` source, downloads it
only when it is missing, copies it to `src/ros2_rm_robot`, and checks the Ubuntu
ROS2 environment.

To let the script install missing system dependencies through apt, add the
explicit install flag:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh --install-apt-deps
```

This runs `sudo apt-get update` and installs available base, ROS2 Humble,
MoveIt2, Gazebo/ros_gz, colcon, Python YAML, and ffmpeg packages. It does not
rewrite apt sources. If ROS2 or Gazebo packages are unavailable from the VM's
current apt sources, the script reports those packages and leaves the source
configuration to the VM owner.

If the VM already has RealMan packages in another workspace, reuse them instead
of downloading again:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh \
  --official-source-dir ~/old_ros2_ws/src/ros2_rm_robot \
  --existing-setup ~/old_ros2_ws/install/setup.bash
```

`--official-source-dir` reuses the old source tree. `--existing-setup` sources
the old build as an underlay for environment checks and optional builds. The
current workspace still keeps its own `src/ros2_rm_robot`, because the recording
and playback scripts expect RealMan package files under this workspace after
build.
