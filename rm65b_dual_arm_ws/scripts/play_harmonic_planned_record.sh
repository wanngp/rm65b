#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:?output directory required}"
WORKSPACE="${2:-/tmp/rm65b_dual_arm_ws_ascii_verify2}"
DOMAIN_ID="${3:-68}"
DAY_ID="${4:-day05}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

normalize_day_id() {
  case "${1,,}" in
    d1|day1|day01) echo "day01" ;;
    d2|day2|day02) echo "day02" ;;
    d3|day3|day03) echo "day03" ;;
    d4|day4|day04) echo "day04" ;;
    d5|day5|day05) echo "day05" ;;
    *) echo "$1" ;;
  esac
}

DAY_NORM="$(normalize_day_id "$DAY_ID")"

case "$DAY_NORM" in
  day01) FRAME_COUNT="${FRAME_COUNT:-500}" ;;
  day02) FRAME_COUNT="${FRAME_COUNT:-500}" ;;
  day03) FRAME_COUNT="${FRAME_COUNT:-460}" ;;
  day04) FRAME_COUNT="${FRAME_COUNT:-520}" ;;
  *) FRAME_COUNT="${FRAME_COUNT:-560}" ;;
esac
CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-120}"
RECORD_RVIZ="${RECORD_RVIZ:-0}"
case "$DAY_NORM" in
  day01|day02) DEFAULT_RVIZ_RECORD_SECONDS="52" ;;
  day03) DEFAULT_RVIZ_RECORD_SECONDS="48" ;;
  day04) DEFAULT_RVIZ_RECORD_SECONDS="54" ;;
  *) DEFAULT_RVIZ_RECORD_SECONDS="58" ;;
esac
RVIZ_RECORD_SECONDS="${RVIZ_RECORD_SECONDS:-$DEFAULT_RVIZ_RECORD_SECONDS}"
RVIZ_VIDEO_SIZE="${RVIZ_VIDEO_SIZE:-1280x720}"
RVIZ_FPS="${RVIZ_FPS:-10}"
RVIZ_STARTUP_SLEEP="${RVIZ_STARTUP_SLEEP:-12}"
RVIZ_CAPTURE_OFFSET="${RVIZ_CAPTURE_OFFSET:-+0,0}"
RVIZ_WINDOW_WAIT_SECONDS="${RVIZ_WINDOW_WAIT_SECONDS:-35}"
RVIZ_WINDOW_PATTERN="${RVIZ_WINDOW_PATTERN:-dual_arm_acceptance.rviz}"
RVIZ_MIN_WINDOW_WIDTH="${RVIZ_MIN_WINDOW_WIDTH:-640}"
RVIZ_MIN_WINDOW_HEIGHT="${RVIZ_MIN_WINDOW_HEIGHT:-480}"

mkdir -p "$OUT_DIR/logs" "$OUT_DIR/screenshots" "$OUT_DIR/videos" "$OUT_DIR/rosbags" "$OUT_DIR/camera_frames" "$OUT_DIR/left_camera_frames" "$OUT_DIR/right_camera_frames" "$OUT_DIR/vision_debug_frames"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_PARTITION="${GZ_PARTITION:-rm65b_planned_${DOMAIN_ID}_$$}"
export GZ_SIM_RESOURCE_PATH="$WORKSPACE/install/rm_description/share/rm_description:${GZ_SIM_RESOURCE_PATH:-}"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

WORLD="${HARMONIC_WORLD:-$WORKSPACE/harmonic/rm65b_empty_world.sdf}"
WORLD_RUNTIME="$OUT_DIR/logs/rm65b_planned_world_runtime.sdf"
DAY_WORLD_GENERATOR="${DAY_WORLD_GENERATOR:-$SCRIPT_DIR/generate_day_world.py}"
URDF="$WORKSPACE/install/rm_description/share/rm_description/urdf/rm_65.urdf"
URDF_HARMONIC="$OUT_DIR/logs/rm_65_harmonic_file_meshes.urdf"
SDF_DYNAMIC="$OUT_DIR/logs/rm_65_harmonic_dynamic.sdf"
SDF_CONTROLLED_BASE="$OUT_DIR/logs/rm_65_harmonic_fixed_base_controller_base.sdf"
SDF_CONTROLLED_LEFT="$OUT_DIR/logs/rm_65_harmonic_fixed_base_controller_left_eye_in_hand.sdf"
SDF_CONTROLLED_RIGHT="$OUT_DIR/logs/rm_65_harmonic_fixed_base_controller_right_eye_in_hand.sdf"
MESH_ROOT="$WORKSPACE/install/rm_description/share/rm_description/meshes"
TRAJECTORY_FILE="$WORKSPACE/install/rm65b_weaving_primitives/share/rm65b_weaving_primitives/trajectories/weaving_primitives.yaml"
SYNC_JOINT_STATE_REPLAY="${SYNC_JOINT_STATE_REPLAY:-$SCRIPT_DIR/replay_moveit_plan_joint_states.py}"
GAZEBO_TRAJECTORY_PUBLISHER="${GAZEBO_TRAJECTORY_PUBLISHER:-$SCRIPT_DIR/publish_gazebo_trajectory_from_plan.py}"
PHYSICAL_INTERACTION_CONTROLLER="${PHYSICAL_INTERACTION_CONTROLLER:-$SCRIPT_DIR/physical_interaction_controller.py}"
RUNTIME_LINK_ATTACHER_SOURCE="${RUNTIME_LINK_ATTACHER_SOURCE:-$SCRIPT_DIR/rm65b_runtime_link_attacher.cc}"
RUNTIME_LINK_ATTACHER_SO="$OUT_DIR/logs/librm65b_runtime_link_attacher.so"
USE_RUNTIME_LINK_ATTACHER_COMPONENT="${USE_RUNTIME_LINK_ATTACHER_COMPONENT:-0}"
SCENE_SYNC_URDF="${SCENE_SYNC_URDF:-$WORKSPACE/src/ros2_rm_robot/rm_description/urdf/rm_65.urdf}"
if [ ! -s "$SCENE_SYNC_URDF" ]; then
  SCENE_SYNC_URDF="$URDF"
fi
USE_MOVEIT_GAZEBO_TRAJECTORY="${USE_MOVEIT_GAZEBO_TRAJECTORY:-1}"
RUN_LEGACY_DUAL_ARM_PLANNER="${RUN_LEGACY_DUAL_ARM_PLANNER:-0}"
MOVEIT_SEGMENT_DURATION="${MOVEIT_SEGMENT_DURATION:-8.0}"
SYNC_REPLAY_START_DELAY="${SYNC_REPLAY_START_DELAY:-0.25}"
SYNC_SCENE_RATE_HZ="${SYNC_SCENE_RATE_HZ:-8.0}"
TRUE_DUAL_MOVEIT="${TRUE_DUAL_MOVEIT:-1}"
LEFT_EIH_VISUAL_POSE="0.032 0 0.090 0 0.35 0"
LEFT_EIH_SENSOR_POSE="0.034 0 0.095 0 0.35 0"
LEFT_EIH_HORIZONTAL_FOV="1.65"
RIGHT_EIH_HORIZONTAL_FOV="1.65"
if [[ "$DAY_NORM" == "day03" ]]; then
  LEFT_EIH_VISUAL_POSE="0.032 0 0.090 0 0.35 0.38"
  LEFT_EIH_SENSOR_POSE="0.034 0 0.095 0 0.35 0.38"
  LEFT_EIH_HORIZONTAL_FOV="2.20"
elif [[ "$DAY_NORM" == "day02" || "$DAY_NORM" == "day05" ]]; then
  LEFT_EIH_VISUAL_POSE="0.032 0 0.090 0 0.35 0.75"
  LEFT_EIH_SENSOR_POSE="0.034 0 0.095 0 0.35 0.75"
  if [[ "$DAY_NORM" == "day05" ]]; then
    LEFT_EIH_HORIZONTAL_FOV="2.20"
  fi
fi
RIGHT_EIH_VISUAL_POSE="0.032 0 0.090 0 0.35 0"
RIGHT_EIH_SENSOR_POSE="0.034 0 0.095 0 0.35 0"
LEFT_INITIAL_POSITIONS="0.000000 -0.350000 0.650000 0.000000 0.900000 0.000000"
RIGHT_INITIAL_POSITIONS="$LEFT_INITIAL_POSITIONS"
if [[ "$DAY_NORM" == "day03" ]]; then
  LEFT_INITIAL_POSITIONS="0.046000 -0.840000 1.161000 -0.120000 0.865000 -0.383000"
  RIGHT_INITIAL_POSITIONS="0.000000 -0.350000 0.650000 0.000000 0.900000 0.000000"
fi
if [ "$TRUE_DUAL_MOVEIT" = "1" ]; then
  MOVEIT_CONFIG_PACKAGE="rm65b_dual_arm_moveit_config"
  MOVEIT_PLAN_EXECUTABLE="dual_moveit_plan_client"
  MOVEIT_PLAN_FILE="$OUT_DIR/logs/dual_moveit_plans.yaml"
  MOVEIT_SUMMARY_FILE="$OUT_DIR/logs/dual_moveit_summary.json"
  MOVEIT_LEFT_GZ_TRAJECTORY="$OUT_DIR/logs/dual_moveit_left_gz_trajectory.pbtxt"
  MOVEIT_RIGHT_GZ_TRAJECTORY="$OUT_DIR/logs/dual_moveit_right_gz_trajectory.pbtxt"
else
  MOVEIT_CONFIG_PACKAGE="rm_65_config"
  MOVEIT_PLAN_EXECUTABLE="moveit_plan_client"
  MOVEIT_PLAN_FILE="$OUT_DIR/logs/moveit_plans.yaml"
  MOVEIT_SUMMARY_FILE="$OUT_DIR/logs/moveit_summary.json"
  MOVEIT_LEFT_GZ_TRAJECTORY="$OUT_DIR/logs/moveit_left_gz_trajectory.pbtxt"
  MOVEIT_RIGHT_GZ_TRAJECTORY="$OUT_DIR/logs/moveit_right_gz_trajectory.pbtxt"
fi

if [ "$USE_MOVEIT_GAZEBO_TRAJECTORY" != "1" ]; then
  echo "legacy scripted trajectory/choreography is permanently disabled for acceptance recordings." >&2
  exit 2
fi

sed "s#package://rm_description/meshes#file://$MESH_ROOT#g" "$URDF" > "$URDF_HARMONIC"
gz sdf -p "$URDF_HARMONIC" > "$SDF_DYNAMIC"

python3 - "$SDF_DYNAMIC" "$SDF_CONTROLLED_BASE" "$DAY_NORM" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst, day = sys.argv[1:4]
enable_d2_dynamics = day in {"day02", "d2"}
tree = ET.parse(src)
root = tree.getroot()
model = root.find("model")
if model is None:
    raise SystemExit("converted SDF has no model element")

model.set("name", "rm65b_fixed_base_joint_control")
static = model.find("static")
if static is None:
    static = ET.Element("static")
    model.insert(0, static)
static.text = "false"

for link in model.findall("link"):
    gravity = link.find("gravity")
    if gravity is None:
        gravity = ET.SubElement(link, "gravity")
    gravity.text = "true" if enable_d2_dynamics else "false"
    for collision in list(link.findall("collision")):
        link.remove(collision)

world_joint = ET.Element("joint", {"name": "world_fixed_base", "type": "fixed"})
ET.SubElement(world_joint, "parent").text = "world"
ET.SubElement(world_joint, "child").text = "base_link"
model.insert(1, world_joint)

def add_inertial(parent, mass="0.050", pose="0 0 0 0 0 0", inertia=None):
    inertial = ET.SubElement(parent, "inertial")
    ET.SubElement(inertial, "pose").text = pose
    ET.SubElement(inertial, "mass").text = mass
    inertia = ET.SubElement(inertial, "inertia")
    values = inertia or {
        "ixx": "0.000020", "ixy": "0", "ixz": "0",
        "iyy": "0.000020", "iyz": "0", "izz": "0.000020",
    }
    for tag, value in values.items():
        ET.SubElement(inertia, tag).text = value

def add_box(parent, tag, name, pose_text, size_text, color_text=None):
    element = ET.SubElement(parent, tag, {"name": name})
    ET.SubElement(element, "pose").text = pose_text
    geometry = ET.SubElement(element, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size_text
    if color_text is not None:
        material = ET.SubElement(element, "material")
        ET.SubElement(material, "ambient").text = color_text
        ET.SubElement(material, "diffuse").text = color_text
    return element

def add_box_visual(parent, name, pose_text, size_text, color_text, collision=False):
    visual = ET.SubElement(parent, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = pose_text
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size_text
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = color_text
    ET.SubElement(material, "diffuse").text = color_text
    if collision:
        add_box(parent, "collision", f"{name}_collision", pose_text, size_text)

def add_position_controller(parent, joint_name, topic):
    plugin = ET.SubElement(
        parent,
        "plugin",
        {
            "filename": "gz-sim-joint-position-controller-system",
            "name": "gz::sim::systems::JointPositionController",
        },
    )
    ET.SubElement(plugin, "joint_name").text = joint_name
    ET.SubElement(plugin, "topic").text = topic
    ET.SubElement(plugin, "p_gain").text = "80"
    ET.SubElement(plugin, "i_gain").text = "0.1"
    ET.SubElement(plugin, "d_gain").text = "2.0"

if model.find("link[@name='Link6']") is not None:
    tool_color = "0.16 0.22 0.28 1"
    finger_color = "0.05 0.06 0.07 1"
    gripper = ET.SubElement(model, "link", {"name": "attached_scaled_gripper_palm"})
    ET.SubElement(gripper, "pose", {"relative_to": "Link6"}).text = "0.018 0 0 0 0 0"
    ET.SubElement(gripper, "gravity").text = "true" if enable_d2_dynamics else "false"
    if enable_d2_dynamics:
        add_inertial(
            gripper,
            mass="0.270",
            pose="0.016296 0 0 0 0 0",
            inertia={
                "ixx": "0.00007002", "ixy": "0", "ixz": "0",
                "iyy": "0.000194813", "iyz": "0", "izz": "0.000243800",
            },
        )
    else:
        add_inertial(gripper)
    add_box_visual(gripper, "flange_adapter", "0 0 0 0 0 0", "0.038 0.038 0.012", tool_color, True)
    add_box_visual(gripper, "palm", "0.030 0 0 0 0 0", "0.040 0.026 0.018", tool_color, True)
    for name, y in (("attached_scaled_gripper_upper_finger", 0.006), ("attached_scaled_gripper_lower_finger", -0.006)):
        finger = ET.SubElement(model, "link", {"name": name})
        ET.SubElement(finger, "pose", {"relative_to": "attached_scaled_gripper_palm"}).text = f"0.070 {y:.3f} 0 0 0 0"
        ET.SubElement(finger, "gravity").text = "true" if enable_d2_dynamics else "false"
        if enable_d2_dynamics:
            add_inertial(
                finger,
                mass="0.045",
                pose="0.007111 0 0 0 0 0",
                inertia={
                    "ixx": "0.00000129", "ixy": "0", "ixz": "0",
                    "iyy": "0.000018649", "iyz": "0", "izz": "0.000018859",
                },
            )
        else:
            add_inertial(finger)
        add_box_visual(finger, "finger", "0 0 0 0 0 0", "0.052 0.006 0.010", finger_color, True)
        add_box_visual(finger, "yarn_hook", "0.028 0 0 0 0 0", "0.014 0.014 0.010", finger_color, True)
    fixed = ET.SubElement(model, "joint", {"name": "attached_scaled_gripper_fixed", "type": "fixed"})
    ET.SubElement(fixed, "parent").text = "Link6"
    ET.SubElement(fixed, "child").text = "attached_scaled_gripper_palm"
    upper = ET.SubElement(model, "joint", {"name": "gripper_upper_slide", "type": "prismatic"})
    ET.SubElement(upper, "parent").text = "attached_scaled_gripper_palm"
    ET.SubElement(upper, "child").text = "attached_scaled_gripper_upper_finger"
    ET.SubElement(upper, "axis").append(ET.Element("xyz"))
    upper.find("axis/xyz").text = "0 1 0"
    ET.SubElement(upper.find("axis"), "limit").append(ET.Element("lower"))
    upper.find("axis/limit/lower").text = "0.000"
    ET.SubElement(upper.find("axis/limit"), "upper").text = "0.018"
    lower = ET.SubElement(model, "joint", {"name": "gripper_lower_slide", "type": "prismatic"})
    ET.SubElement(lower, "parent").text = "attached_scaled_gripper_palm"
    ET.SubElement(lower, "child").text = "attached_scaled_gripper_lower_finger"
    ET.SubElement(lower, "axis").append(ET.Element("xyz"))
    lower.find("axis/xyz").text = "0 -1 0"
    ET.SubElement(lower.find("axis"), "limit").append(ET.Element("lower"))
    lower.find("axis/limit/lower").text = "0.000"
    ET.SubElement(lower.find("axis/limit"), "upper").text = "0.018"
    add_position_controller(model, "gripper_upper_slide", "/rm65b_gripper/upper_finger_cmd")
    add_position_controller(model, "gripper_lower_slide", "/rm65b_gripper/lower_finger_cmd")

plugin = ET.SubElement(
    model,
    "plugin",
    {
        "filename": "gz-sim-joint-trajectory-controller-system",
        "name": "gz::sim::systems::JointTrajectoryController",
    },
)
initial_positions = [0.0, -0.35, 0.65, 0.0, 0.90, 0.0]
for idx, initial in enumerate(initial_positions, start=1):
    ET.SubElement(plugin, "joint_name").text = f"joint{idx}"
    ET.SubElement(plugin, "initial_position").text = f"{initial:.6f}"
    ET.SubElement(plugin, "position_p_gain").text = "45"
    ET.SubElement(plugin, "position_i_gain").text = "0.1"
    ET.SubElement(plugin, "position_d_gain").text = "1.5"
    ET.SubElement(plugin, "position_i_min").text = "-1"
    ET.SubElement(plugin, "position_i_max").text = "1"
    ET.SubElement(plugin, "position_cmd_min").text = "-80"
    ET.SubElement(plugin, "position_cmd_max").text = "80"

tree.write(dst, encoding="unicode", xml_declaration=True)
PY

python3 - "$SDF_CONTROLLED_BASE" "$SDF_CONTROLLED_LEFT" "/left_camera/image_rect" "left_eye_in_hand_camera" "0.16 0.36 0.68 1" "$LEFT_EIH_VISUAL_POSE" "$LEFT_EIH_SENSOR_POSE" "$LEFT_INITIAL_POSITIONS" "$LEFT_EIH_HORIZONTAL_FOV" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst, topic, sensor_name, color, visual_pose, sensor_pose, initial_positions, horizontal_fov = sys.argv[1:10]
tree = ET.parse(src)
root = tree.getroot()
model = root.find("model")
if model is None:
    raise SystemExit("arm SDF has no model element")
palm = model.find("link[@name='attached_scaled_gripper_palm']")
if palm is None:
    raise SystemExit("arm SDF has no attached_scaled_gripper_palm link for eye-in-hand camera")

visual = ET.SubElement(palm, "visual", {"name": f"{sensor_name}_body"})
ET.SubElement(visual, "pose").text = visual_pose
geometry = ET.SubElement(visual, "geometry")
box = ET.SubElement(geometry, "box")
ET.SubElement(box, "size").text = "0.024 0.018 0.014"
material = ET.SubElement(visual, "material")
ET.SubElement(material, "ambient").text = color
ET.SubElement(material, "diffuse").text = color

sensor = ET.SubElement(palm, "sensor", {"name": sensor_name, "type": "camera"})
ET.SubElement(sensor, "pose").text = sensor_pose
ET.SubElement(sensor, "topic").text = topic
ET.SubElement(sensor, "always_on").text = "true"
ET.SubElement(sensor, "update_rate").text = "30"
camera = ET.SubElement(sensor, "camera")
ET.SubElement(camera, "horizontal_fov").text = horizontal_fov
image = ET.SubElement(camera, "image")
ET.SubElement(image, "width").text = "640"
ET.SubElement(image, "height").text = "480"
ET.SubElement(image, "format").text = "R8G8B8"
clip = ET.SubElement(camera, "clip")
ET.SubElement(clip, "near").text = "0.02"
ET.SubElement(clip, "far").text = "6"

values = [float(value) for value in initial_positions.split()]
for plugin in model.findall("plugin"):
    if plugin.get("filename") != "gz-sim-joint-trajectory-controller-system":
        continue
    initial_elems = plugin.findall("initial_position")
    for elem, value in zip(initial_elems, values):
        elem.text = f"{value:.6f}"

tree.write(dst, encoding="unicode", xml_declaration=True)
PY

python3 - "$SDF_CONTROLLED_BASE" "$SDF_CONTROLLED_RIGHT" "/right_camera/image_rect" "right_eye_in_hand_camera" "0.68 0.24 0.18 1" "$RIGHT_EIH_VISUAL_POSE" "$RIGHT_EIH_SENSOR_POSE" "$RIGHT_INITIAL_POSITIONS" "$RIGHT_EIH_HORIZONTAL_FOV" <<'PY'
import sys
import xml.etree.ElementTree as ET

src, dst, topic, sensor_name, color, visual_pose, sensor_pose, initial_positions, horizontal_fov = sys.argv[1:10]
tree = ET.parse(src)
root = tree.getroot()
model = root.find("model")
if model is None:
    raise SystemExit("arm SDF has no model element")
palm = model.find("link[@name='attached_scaled_gripper_palm']")
if palm is None:
    raise SystemExit("arm SDF has no attached_scaled_gripper_palm link for eye-in-hand camera")

visual = ET.SubElement(palm, "visual", {"name": f"{sensor_name}_body"})
ET.SubElement(visual, "pose").text = visual_pose
geometry = ET.SubElement(visual, "geometry")
box = ET.SubElement(geometry, "box")
ET.SubElement(box, "size").text = "0.024 0.018 0.014"
material = ET.SubElement(visual, "material")
ET.SubElement(material, "ambient").text = color
ET.SubElement(material, "diffuse").text = color

sensor = ET.SubElement(palm, "sensor", {"name": sensor_name, "type": "camera"})
ET.SubElement(sensor, "pose").text = sensor_pose
ET.SubElement(sensor, "topic").text = topic
ET.SubElement(sensor, "always_on").text = "true"
ET.SubElement(sensor, "update_rate").text = "30"
camera = ET.SubElement(sensor, "camera")
ET.SubElement(camera, "horizontal_fov").text = horizontal_fov
image = ET.SubElement(camera, "image")
ET.SubElement(image, "width").text = "640"
ET.SubElement(image, "height").text = "480"
ET.SubElement(image, "format").text = "R8G8B8"
clip = ET.SubElement(camera, "clip")
ET.SubElement(clip, "near").text = "0.02"
ET.SubElement(clip, "far").text = "6"

values = [float(value) for value in initial_positions.split()]
for plugin in model.findall("plugin"):
    if plugin.get("filename") != "gz-sim-joint-trajectory-controller-system":
        continue
    initial_elems = plugin.findall("initial_position")
    for elem, value in zip(initial_elems, values):
        elem.text = f"{value:.6f}"

tree.write(dst, encoding="unicode", xml_declaration=True)
PY

python3 - "$SDF_CONTROLLED_LEFT" "$SDF_CONTROLLED_RIGHT" "$DAY_NORM" "$USE_RUNTIME_LINK_ATTACHER_COMPONENT" <<'PY'
import sys
import xml.etree.ElementTree as ET

left_sdf, right_sdf, day, use_runtime_attacher = sys.argv[1:5]


def add_detachable_plugin(
    sdf_file: str,
    *,
    name: str,
    parent_link: str,
    child_model: str,
    child_link: str,
) -> None:
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise SystemExit(f"{sdf_file} has no model element")
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-detachable-joint-system",
            "name": "gz::sim::systems::DetachableJoint",
        },
    )
    for tag, value in (
        ("parent_link", parent_link),
        ("child_model", child_model),
        ("child_link", child_link),
        ("detach_topic", f"/rm65b/physical/{name}/detach"),
        ("attach_topic", f"/rm65b/physical/{name}/attach"),
        ("output_topic", f"/rm65b/physical/{name}/state"),
    ):
        ET.SubElement(plugin, tag).text = value
    tree.write(sdf_file, encoding="unicode", xml_declaration=True)


def add_force_probe_tool(sdf_file: str) -> None:
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise SystemExit(f"{sdf_file} has no model element")
    if model.find("link[@name='d2_force_probe_tip']") is not None:
        return

    def add_probe_link(name: str, parent: str, pose: str, size: str) -> None:
        link = ET.SubElement(model, "link", {"name": name})
        ET.SubElement(link, "pose", {"relative_to": parent}).text = pose
        ET.SubElement(link, "gravity").text = "false"
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = "0.035"
        inertia = ET.SubElement(inertial, "inertia")
        for tag, value in {
            "ixx": "0.000010",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.000010",
            "iyz": "0",
            "izz": "0.000010",
        }.items():
            ET.SubElement(inertia, tag).text = value
        for tag, rgba in (("visual", "0.95 0.82 0.18 1"), ("collision", None)):
            elem = ET.SubElement(link, tag, {"name": "probe_tip" if tag == "visual" else "probe_collision"})
            ET.SubElement(elem, "pose").text = "0 0 0 0 0 0"
            geometry = ET.SubElement(elem, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = size
            if rgba is not None:
                material = ET.SubElement(elem, "material")
                ET.SubElement(material, "ambient").text = rgba
                ET.SubElement(material, "diffuse").text = rgba
        joint = ET.SubElement(model, "joint", {"name": f"{name}_fixed", "type": "fixed"})
        ET.SubElement(joint, "parent").text = parent
        ET.SubElement(joint, "child").text = name

    add_probe_link("d2_force_probe_tip", "attached_scaled_gripper_palm", "0.135 0 0 0 0 0", "0.045 0.045 0.045")
    add_probe_link("d2_forearm_contact_bumper", "Link5", "0 0 0 0 0 0", "0.140 0.140 0.140")
    tree.write(sdf_file, encoding="unicode", xml_declaration=True)


if day == "day01":
    pass
elif day == "day02":
    add_force_probe_tool(right_sdf)
elif day in {"day04", "day05"}:
    if day == "day05":
        add_force_probe_tool(right_sdf)
    add_detachable_plugin(
        right_sdf,
        name="d4_right_yarn",
        parent_link="attached_scaled_gripper_palm",
        child_model="weft_yarn",
        child_link="link",
    )
    add_detachable_plugin(
        right_sdf,
        name="d4_right_probe",
        parent_link="attached_scaled_gripper_palm",
        child_model="d4_shuttle_contact_probe",
        child_link="link",
    )
PY

python3 "$DAY_WORLD_GENERATOR" \
  --base-world "$WORLD" \
  --day-id "$DAY_ID" \
  --output "$WORLD_RUNTIME" \
  --notes "$OUT_DIR/logs/day_environment_notes.txt"

if [ "$USE_RUNTIME_LINK_ATTACHER_COMPONENT" = "1" ]; then
  g++ -std=c++17 -fPIC -shared "$RUNTIME_LINK_ATTACHER_SOURCE" \
    -o "$RUNTIME_LINK_ATTACHER_SO" \
    $(pkg-config --cflags --libs gz-sim8 gz-plugin2 gz-transport13 gz-msgs10) \
    > "$OUT_DIR/logs/runtime_link_attacher_build.log" 2>&1

  python3 - "$WORLD_RUNTIME" "$RUNTIME_LINK_ATTACHER_SO" "$DAY_NORM" <<'PY'
import sys
import xml.etree.ElementTree as ET

world_file, plugin_so, day = sys.argv[1:4]
tree = ET.parse(world_file)
root = tree.getroot()
world = root.find("world")
if world is None:
    raise SystemExit("runtime SDF has no world")

plugin = ET.Element(
    "plugin",
    {
        "filename": plugin_so,
        "name": "rm65b::RuntimeLinkAttacher",
    },
)


def text(parent, tag, value):
    elem = ET.SubElement(parent, tag)
    elem.text = value
    return elem


def attachment(name, parent, child):
    elem = ET.SubElement(plugin, "attachment")
    text(elem, "name", name)
    text(elem, "parent_link", parent)
    text(elem, "child_link", child)
    text(elem, "attach_topic", f"/rm65b/physical/{name}/attach")
    text(elem, "detach_topic", f"/rm65b/physical/{name}/detach")
    text(elem, "output_topic", f"/rm65b/physical/{name}/state")


if day == "day01":
    pass
elif day == "day04":
    attachment("d4_right_yarn", "right_rm65b::attached_scaled_gripper_palm", "weft_yarn::link")
    attachment("d4_right_probe", "right_rm65b::attached_scaled_gripper_palm", "d4_shuttle_contact_probe::link")
elif day == "day05":
    attachment("d4_right_yarn", "right_rm65b::attached_scaled_gripper_palm", "weft_yarn::link")
    attachment("d4_right_probe", "right_rm65b::attached_scaled_gripper_palm", "d4_shuttle_contact_probe::link")

if list(plugin):
    world.append(plugin)
tree.write(world_file, encoding="unicode", xml_declaration=True)
PY
else
  echo "runtime component attacher skipped; using model-scoped gz-sim-detachable-joint-system plugins" \
    > "$OUT_DIR/logs/runtime_link_attacher_build.log"
fi

{
  echo "OUT_DIR=$OUT_DIR"
  echo "WORKSPACE=$WORKSPACE"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "GZ_PARTITION=$GZ_PARTITION"
  echo "BASE_WORLD=$WORLD"
  echo "WORLD=$WORLD_RUNTIME"
  echo "DAY_ID=$DAY_ID"
  echo "DAY_NORM=$DAY_NORM"
  echo "FRAME_COUNT=$FRAME_COUNT"
  echo "CAPTURE_TIMEOUT=$CAPTURE_TIMEOUT"
  echo "RECORD_RVIZ=$RECORD_RVIZ"
  echo "RVIZ_RECORD_SECONDS=$RVIZ_RECORD_SECONDS"
  echo "RVIZ_VIDEO_SIZE=$RVIZ_VIDEO_SIZE"
  echo "DISPLAY=${DISPLAY:-}"
  echo "URDF=$URDF"
  echo "SDF_CONTROLLED_BASE=$SDF_CONTROLLED_BASE"
  echo "SDF_CONTROLLED_LEFT=$SDF_CONTROLLED_LEFT"
  echo "SDF_CONTROLLED_RIGHT=$SDF_CONTROLLED_RIGHT"
  echo "TRAJECTORY_FILE=$TRAJECTORY_FILE"
  echo "MOVEIT_PLAN_FILE=$MOVEIT_PLAN_FILE"
  echo "MOVEIT_SUMMARY_FILE=$MOVEIT_SUMMARY_FILE"
  echo "RUNTIME_LINK_ATTACHER_SO=$RUNTIME_LINK_ATTACHER_SO"
  echo "PHYSICAL_INTERACTION_CONTROLLER=$PHYSICAL_INTERACTION_CONTROLLER"
  echo "TRUE_DUAL_MOVEIT=$TRUE_DUAL_MOVEIT"
  echo "MOVEIT_CONFIG_PACKAGE=$MOVEIT_CONFIG_PACKAGE"
  command -v gz
  command -v rviz2 || true
  command -v ffmpeg || true
  echo "gz_cli=available"
} | tee "$OUT_DIR/logs/harmonic_planned_env.log"

PIDS=()
CAPTURE_PID=""
LEFT_CAPTURE_PID=""
RIGHT_CAPTURE_PID=""
D4_RECORDER_PID=""
RVIZ_RECORD_PID=""
ROSBAG_PID=""
PHYSICAL_CONTROLLER_PID=""
GRIPPER_STREAM_PID=""
SAMPLE_PIDS=()

CONTACT_RECORD_TOPICS=()
case "$DAY_NORM" in
  day01)
    CONTACT_RECORD_TOPICS=()
    ;;
  day02)
    CONTACT_RECORD_TOPICS=(
      /world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact
      /world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact
    )
    ;;
  day03)
    CONTACT_RECORD_TOPICS=(
      /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact
    )
    ;;
  day04)
    CONTACT_RECORD_TOPICS=(
      /world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact
      /world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact
    )
    ;;
  *)
    CONTACT_RECORD_TOPICS=(
      /world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact
      /world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact
      /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact
      /world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact
      /world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact
    )
    ;;
esac
terminate_tree() {
  local root="$1"
  local child
  for child in $(pgrep -P "$root" 2>/dev/null || true); do
    terminate_tree "$child"
  done
  if kill -0 "$root" 2>/dev/null; then
    kill -INT "$root" 2>/dev/null || kill -TERM "$root" 2>/dev/null || true
  fi
}

wait_or_kill() {
  local pid="$1"
  local attempt
  for attempt in $(seq 1 120); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.5
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  if [ -n "${GRIPPER_STOP_FILE:-}" ]; then
    touch "$GRIPPER_STOP_FILE" 2>/dev/null || true
  fi
  if [ -n "${CAPTURE_PID:-}" ] && kill -0 "$CAPTURE_PID" 2>/dev/null; then
    terminate_tree "$CAPTURE_PID"
  fi
  if [ -n "${LEFT_CAPTURE_PID:-}" ] && kill -0 "$LEFT_CAPTURE_PID" 2>/dev/null; then
    terminate_tree "$LEFT_CAPTURE_PID"
  fi
  if [ -n "${RIGHT_CAPTURE_PID:-}" ] && kill -0 "$RIGHT_CAPTURE_PID" 2>/dev/null; then
    terminate_tree "$RIGHT_CAPTURE_PID"
  fi
  if [ -n "${VISION_DEBUG_CAPTURE_PID:-}" ] && kill -0 "$VISION_DEBUG_CAPTURE_PID" 2>/dev/null; then
    terminate_tree "$VISION_DEBUG_CAPTURE_PID"
  fi
  for pid in "${PIDS[@]:-}"; do
    terminate_tree "$pid"
  done
  sleep 2
  if [ -n "${CAPTURE_PID:-}" ] && kill -0 "$CAPTURE_PID" 2>/dev/null; then
    kill -TERM "$CAPTURE_PID" 2>/dev/null || true
    wait_or_kill "$CAPTURE_PID"
  fi
  if [ -n "${LEFT_CAPTURE_PID:-}" ] && kill -0 "$LEFT_CAPTURE_PID" 2>/dev/null; then
    kill -TERM "$LEFT_CAPTURE_PID" 2>/dev/null || true
    wait_or_kill "$LEFT_CAPTURE_PID"
  fi
  if [ -n "${RIGHT_CAPTURE_PID:-}" ] && kill -0 "$RIGHT_CAPTURE_PID" 2>/dev/null; then
    kill -TERM "$RIGHT_CAPTURE_PID" 2>/dev/null || true
    wait_or_kill "$RIGHT_CAPTURE_PID"
  fi
  if [ -n "${VISION_DEBUG_CAPTURE_PID:-}" ] && kill -0 "$VISION_DEBUG_CAPTURE_PID" 2>/dev/null; then
    kill -TERM "$VISION_DEBUG_CAPTURE_PID" 2>/dev/null || true
    wait_or_kill "$VISION_DEBUG_CAPTURE_PID"
  fi
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      :
    fi
    wait_or_kill "$pid"
  done
}
trap cleanup EXIT

if [ "${RECORD_ALL_TOPICS:-0}" = "1" ]; then
  ros2 bag record \
  --max-cache-size 1073741824 \
  -a \
  -o "$OUT_DIR/rosbags/harmonic_planned_rosbag" \
  > "$OUT_DIR/logs/rosbag_record.log" 2>&1 &
else
  ros2 bag record \
  --max-cache-size 1073741824 \
  -o "$OUT_DIR/rosbags/harmonic_planned_rosbag" \
  /clock /tf /tf_static /parameter_events \
  /joint_states /dual_arm_planning/phase \
  /dual_arm_planning/left_joint_trajectory /dual_arm_planning/right_joint_trajectory \
  /move_action/_action/status /move_action/_action/feedback /display_planned_path \
  /left_gripper_controller/joint_states /right_gripper_controller/joint_states \
  /left_rm_driver/rm_driver/udp_six_force /right_rm_driver/rm_driver/udp_six_force \
  /weaving/tension_n /weaving/tension_status /weaving/tension_pid_state \
  /weaving/tension_pid_params /weaving/compliance_offset_m \
  /force_control/target_wrench /force_control/wrench_error /force_control/admittance_offset \
  /force_control/state /force_control/corrected_right_joint_trajectory /force_control/gazebo_relay_state \
  /force_control/gazebo_contact_force_state \
  /force_control/impedance_wrench /force_control/impedance_state \
  /force_control/admittance_displacement_m /force_control/admittance_velocity_mps \
  /force_control/corrected_right_joint_states \
  /left_camera/camera_info /vision/target_pose /vision/status /vision/metrics /vision/debug_image \
  /visual_servo/twist_cmd /visual_servo/aligned /visual_servo/gazebo_adapter_state \
  /visual_servo/left_corrected_joint_trajectory \
  /model/left_rm65b/joint_trajectory /model/right_rm65b/joint_trajectory \
  /rm65b_gripper/upper_finger_cmd /rm65b_gripper/lower_finger_cmd \
  "${CONTACT_RECORD_TOPICS[@]}" \
  /weaving/events /safety/state /safety/fault /safety/estop /safety/reset_required \
  /safety/recovery_command /system/launch_audit /acceptance/day_status \
  > "$OUT_DIR/logs/rosbag_record.log" 2>&1 &
fi
ROSBAG_PID="$!"
PIDS+=("$ROSBAG_PID")

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /left_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image \
  /right_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image \
  /rm65b/evidence_camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /model/left_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /model/right_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /rm65b_gripper/upper_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /rm65b_gripper/lower_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /contacts/d2_force_target_panel@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d2_contact_pad@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/vision_target@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d2_left_compliance_bar@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d2_right_compliance_bar@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d4_upper_loom_rail@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d4_lower_loom_rail@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d4_shuttle_lane@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /contacts/d4_tension_scale@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d2_left_compliance_bar/link/link/sensor/d2_left_compliance_bar_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d2_right_compliance_bar/link/link/sensor/d2_right_compliance_bar_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d4_upper_loom_rail/link/link/sensor/d4_upper_loom_rail_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d4_lower_loom_rail/link/link/sensor/d4_lower_loom_rail_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  /world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts \
  > "$OUT_DIR/logs/ros_gz_bridge.log" 2>&1 &
PIDS+=("$!")

GRIPPER_TARGET_FILE="$OUT_DIR/logs/gripper_target_position.txt"
GRIPPER_STOP_FILE="$OUT_DIR/logs/stop_gripper_command_stream"
rm -f "$GRIPPER_STOP_FILE"
if [[ "$DAY_NORM" == "day02" || "$DAY_NORM" == "day03" ]]; then
  printf "0.000\n" > "$GRIPPER_TARGET_FILE"
else
  printf "0.018\n" > "$GRIPPER_TARGET_FILE"
fi
gripper_command_stream() {
  while [ ! -f "$GRIPPER_STOP_FILE" ]; do
    local position
    position="$(cat "$GRIPPER_TARGET_FILE" 2>/dev/null || printf "0.018")"
    timeout 1.0s ros2 topic pub --once /rm65b_gripper/upper_finger_cmd std_msgs/msg/Float64 "{data: $position}" >/dev/null 2>&1 || true
    timeout 1.0s ros2 topic pub --once /rm65b_gripper/lower_finger_cmd std_msgs/msg/Float64 "{data: $position}" >/dev/null 2>&1 || true
    sleep 0.25
  done
}
gripper_command_stream > "$OUT_DIR/logs/ros_gripper_command_stream.log" 2>&1 &
GRIPPER_STREAM_PID="$!"
PIDS+=("$GRIPPER_STREAM_PID")

if [[ "$DAY_NORM" == "day03" || "$DAY_NORM" == "day05" ]]; then
  ros2 run rm65b_vision_guidance gazebo_camera_info_publisher \
    > "$OUT_DIR/logs/gazebo_camera_info_publisher.log" 2>&1 &
  PIDS+=("$!")

  ros2 run rm65b_vision_guidance aruco_target_node \
    --ros-args \
    -p image_topic:=/left_camera/image_rect \
    -p camera_info_topic:=/left_camera/camera_info \
    -p debug_image_topic:=/vision/debug_image \
    -p fallback_publish_center:=false \
    > "$OUT_DIR/logs/aruco_target_node.log" 2>&1 &
  PIDS+=("$!")

  ros2 run rm65b_vision_guidance ibvs_controller \
    > "$OUT_DIR/logs/ibvs_controller.log" 2>&1 &
  PIDS+=("$!")
else
  echo "vision stack intentionally skipped for $DAY_NORM" > "$OUT_DIR/logs/vision_stack_skipped.log"
fi

ros2 launch "$MOVEIT_CONFIG_PACKAGE" move_group.launch.py allow_trajectory_execution:=false \
  > "$OUT_DIR/logs/move_group.log" 2>&1 &
PIDS+=("$!")

gz sim -v 4 -r -s "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_sim.log" 2>&1 &
PIDS+=("$!")

sleep 10
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_CONTROLLED_LEFT" -name left_rm65b -x -0.45 -y 0 -z 0.02 -Y 1.5708 \
  > "$OUT_DIR/logs/spawn_left_rm65b.log" 2>&1 || true
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_CONTROLLED_RIGHT" -name right_rm65b -x 0.45 -y 0 -z 0.02 -Y -1.5708 \
  > "$OUT_DIR/logs/spawn_right_rm65b.log" 2>&1 || true
sleep 3

gz topic -l > "$OUT_DIR/logs/gz_topics_after_spawn.txt" 2>&1 || true

sleep 4
ros2 run rm65b_dual_arm_planning "$MOVEIT_PLAN_EXECUTABLE" \
  --day-id "$DAY_ID" \
  --output-dir "$OUT_DIR/logs" \
  --segment-duration "$MOVEIT_SEGMENT_DURATION" \
  > "$OUT_DIR/logs/moveit_plan_client.log" 2>&1

if [ "$RUN_LEGACY_DUAL_ARM_PLANNER" = "1" ]; then
  echo "RUN_LEGACY_DUAL_ARM_PLANNER is disabled: RViz joint states must replay from the same MoveIt plan sent to Gazebo." >&2
  exit 2
fi
echo "legacy dual_arm_planner skipped; RViz joint states are replayed from the same MoveIt plan sent to Gazebo" > "$OUT_DIR/logs/dual_arm_planner_skipped.log"

if [[ "$DAY_NORM" == "day04" || "$DAY_NORM" == "day05" ]]; then
  ros2 run rm65b_weaving_primitives tension_simulator \
    --ros-args \
    -p config_file:="$WORKSPACE/install/rm65b_weaving_primitives/share/rm65b_weaving_primitives/config/d4_tension_pid_compliance.yaml" \
    > "$OUT_DIR/logs/tension_simulator.log" 2>&1 &
  PIDS+=("$!")
else
  echo "tension simulator intentionally skipped for $DAY_NORM" > "$OUT_DIR/logs/tension_simulator_skipped.log"
fi

if [[ "$DAY_NORM" == "day02" || "$DAY_NORM" == "day04" || "$DAY_NORM" == "day05" ]]; then
  ros2 run rm65b_dual_arm_planning force_admittance_controller \
    --ros-args -p day_id:="$DAY_ID" \
    > "$OUT_DIR/logs/force_admittance_controller.log" 2>&1 &
  PIDS+=("$!")

  ros2 run rm65b_dual_arm_planning gazebo_contact_force_estimator \
    --ros-args -p day_id:="$DAY_ID" \
    > "$OUT_DIR/logs/gazebo_contact_force_estimator.log" 2>&1 &
  PIDS+=("$!")

  ros2 run rm65b_dual_arm_planning force_gazebo_trajectory_relay \
    > "$OUT_DIR/logs/force_gazebo_trajectory_relay.log" 2>&1 &
  PIDS+=("$!")
else
  echo "force stack intentionally skipped for $DAY_NORM" > "$OUT_DIR/logs/force_stack_skipped.log"
fi

if [[ "$DAY_NORM" == "day03" || "$DAY_NORM" == "day05" ]]; then
  ros2 run rm65b_dual_arm_planning visual_servo_gazebo_adapter \
    > "$OUT_DIR/logs/visual_servo_gazebo_adapter.log" 2>&1 &
  PIDS+=("$!")
else
  echo "visual servo adapter intentionally skipped for $DAY_NORM" > "$OUT_DIR/logs/visual_servo_adapter_skipped.log"
fi

if [[ "$DAY_NORM" == "day04" || "$DAY_NORM" == "day05" ]]; then
  ros2 run rm65b_weaving_primitives weaving_bt_runner \
    --ros-args -p dry_run:=true \
    > "$OUT_DIR/logs/weaving_bt_runner.log" 2>&1 &
  PIDS+=("$!")
else
  echo "weaving BT intentionally skipped for $DAY_NORM" > "$OUT_DIR/logs/weaving_bt_skipped.log"
fi

if [[ "$DAY_NORM" == "day04" && "$USE_MOVEIT_GAZEBO_TRAJECTORY" != "1" ]]; then
  python3 "$WORKSPACE/scripts/record_d4_real_teach_replay.py" \
    --output-file "$OUT_DIR/logs/day04_recorded_primitives_from_live_joint_states.yaml" \
    --duration 2.0 \
    --wait-joint-state 25.0 \
    > "$OUT_DIR/logs/d4_teach_replay_recorder.log" 2>&1 &
  D4_RECORDER_PID="$!"
  PIDS+=("$D4_RECORDER_PID")
elif [[ "$DAY_NORM" == "day04" ]]; then
  echo "D4 recorder deferred until synchronized joint-state replay starts." > "$OUT_DIR/logs/d4_teach_replay_recorder_deferred.log"
fi

IMAGE_SAVER="${IMAGE_SAVER:-$WORKSPACE/scripts/save_ros_images.py}"
start_camera_recording() {
  timeout "${CAPTURE_TIMEOUT}s" python3 "$IMAGE_SAVER" \
    --topic /rm65b/evidence_camera/image \
    --output-dir "$OUT_DIR/camera_frames" \
    --count "$FRAME_COUNT" \
    > "$OUT_DIR/logs/save_ros_images.log" 2>&1 &
  CAPTURE_PID="$!"

  timeout "${CAPTURE_TIMEOUT}s" python3 "$IMAGE_SAVER" \
    --topic /left_camera/image_rect \
    --output-dir "$OUT_DIR/left_camera_frames" \
    --count "$FRAME_COUNT" \
    > "$OUT_DIR/logs/save_left_gazebo_camera_images.log" 2>&1 &
  LEFT_CAPTURE_PID="$!"

  timeout "${CAPTURE_TIMEOUT}s" python3 "$IMAGE_SAVER" \
    --topic /right_camera/image_rect \
    --output-dir "$OUT_DIR/right_camera_frames" \
    --count "$FRAME_COUNT" \
    > "$OUT_DIR/logs/save_right_gazebo_camera_images.log" 2>&1 &
  RIGHT_CAPTURE_PID="$!"

  if [[ "$DAY_NORM" == "day03" || "$DAY_NORM" == "day05" ]]; then
    timeout "${CAPTURE_TIMEOUT}s" python3 "$IMAGE_SAVER" \
      --topic /vision/debug_image \
      --output-dir "$OUT_DIR/vision_debug_frames" \
      --count "$FRAME_COUNT" \
      > "$OUT_DIR/logs/save_vision_debug_images.log" 2>&1 &
    VISION_DEBUG_CAPTURE_PID="$!"
  fi
}

start_rviz_recording() {
  if [ "$RECORD_RVIZ" != "1" ]; then
    echo "rviz recording skipped: RECORD_RVIZ=$RECORD_RVIZ" > "$OUT_DIR/logs/rviz_recording.log"
    return
  fi
  if [ -z "${DISPLAY:-}" ]; then
    echo "rviz recording skipped: DISPLAY is not set" > "$OUT_DIR/logs/rviz_recording.log"
    return
  fi
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "rviz recording skipped: rviz2 not found" > "$OUT_DIR/logs/rviz_recording.log"
    return
  fi

  LIBGL_ALWAYS_SOFTWARE=1 \
  QT_XCB_GL_INTEGRATION=none \
  QT_OPENGL=software \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  __GLX_VENDOR_LIBRARY_NAME=mesa \
  ros2 launch "$MOVEIT_CONFIG_PACKAGE" moveit_rviz.launch.py use_sim_time:=false \
    > "$OUT_DIR/logs/rviz_launch.log" 2>&1 &
  local rviz_launch_pid="$!"
  PIDS+=("$rviz_launch_pid")

  sleep "$RVIZ_STARTUP_SLEEP"

  local rviz_window_id=""
  if command -v xdotool >/dev/null 2>&1; then
    local deadline=$((SECONDS + RVIZ_WINDOW_WAIT_SECONDS))
    while [ "$SECONDS" -lt "$deadline" ]; do
      rviz_window_id="$(xdotool search --onlyvisible --name "$RVIZ_WINDOW_PATTERN" 2>/dev/null | head -n 1 || true)"
      if [ -n "$rviz_window_id" ]; then
        break
      fi
      sleep 1
    done
  fi
  if [ -z "$rviz_window_id" ] && command -v xwininfo >/dev/null 2>&1; then
    local deadline=$((SECONDS + RVIZ_WINDOW_WAIT_SECONDS))
    while [ "$SECONDS" -lt "$deadline" ]; do
      xwininfo -root -tree > "$OUT_DIR/logs/rviz_xwininfo_tree.txt" 2>&1 || true
      rviz_window_id="$(
        RVIZ_MIN_WINDOW_WIDTH="$RVIZ_MIN_WINDOW_WIDTH" \
        RVIZ_MIN_WINDOW_HEIGHT="$RVIZ_MIN_WINDOW_HEIGHT" \
        awk '
              /dual_arm_acceptance\.rviz|rm65b_dual_arm_acceptance_rviz/ && $0 !~ /Selection Owner|Initializing/ {
                for (i = 1; i <= NF; ++i) {
                  if ($i ~ /^[0-9]+x[0-9]+[+-]/) {
                    split($i, a, "x")
                    split(a[2], b, /[+-]/)
                    width = a[1] + 0
                    height = b[1] + 0
                    if (width < ENVIRON["RVIZ_MIN_WINDOW_WIDTH"] || height < ENVIRON["RVIZ_MIN_WINDOW_HEIGHT"]) {
                      continue
                    }
                    area = width * height
                    if ($0 ~ / - RViz"/) {
                      area += 1000000000
                    }
                    if (area > best) {
                      best = area
                      id = $1
                    }
                  }
                }
              }
              END {print id}
            ' "$OUT_DIR/logs/rviz_xwininfo_tree.txt"
      )"
      if [ -n "$rviz_window_id" ]; then
        break
      fi
      sleep 1
    done
  fi

  if [ -n "$rviz_window_id" ] && command -v xwininfo >/dev/null 2>&1; then
    xwininfo -id "$rviz_window_id" > "$OUT_DIR/logs/rviz_selected_window_info.txt" 2>&1 || true
  fi

  {
    echo "record_rviz=1"
    echo "rviz_launch_pid=$rviz_launch_pid"
    echo "rviz_window_id=${rviz_window_id:-not_found}"
    echo "rviz_video_size=$RVIZ_VIDEO_SIZE"
    echo "rviz_seconds=$RVIZ_RECORD_SECONDS"
  } > "$OUT_DIR/logs/rviz_recording.log"

  if [ -n "$rviz_window_id" ]; then
    local rviz_video_width="${RVIZ_VIDEO_SIZE%x*}"
    local rviz_video_height="${RVIZ_VIDEO_SIZE#*x}"
    ffmpeg -y \
      -f x11grab \
      -draw_mouse 0 \
      -window_id "$rviz_window_id" \
      -framerate "$RVIZ_FPS" \
      -i "$DISPLAY" \
      -t "$RVIZ_RECORD_SECONDS" \
      -vf "scale=${rviz_video_width}:${rviz_video_height}:force_original_aspect_ratio=decrease,pad=${rviz_video_width}:${rviz_video_height}:(ow-iw)/2:(oh-ih)/2" \
      -pix_fmt yuv420p \
      "$OUT_DIR/videos/${DAY_NORM}_rviz.mp4" \
      > "$OUT_DIR/logs/ffmpeg_rviz_video.log" 2>&1 &
  else
    ffmpeg -y \
      -f x11grab \
      -draw_mouse 0 \
      -video_size "$RVIZ_VIDEO_SIZE" \
      -framerate "$RVIZ_FPS" \
      -i "${DISPLAY}${RVIZ_CAPTURE_OFFSET}" \
      -t "$RVIZ_RECORD_SECONDS" \
      -pix_fmt yuv420p \
      "$OUT_DIR/videos/${DAY_NORM}_rviz.mp4" \
      > "$OUT_DIR/logs/ffmpeg_rviz_video.log" 2>&1 &
  fi
  RVIZ_RECORD_PID="$!"
  PIDS+=("$RVIZ_RECORD_PID")
}

sample_topic_once() {
  local topic="$1"
  local file="$2"
  local field="${3:-}"
  (
    local deadline=$((SECONDS + 20))
    while [ "$SECONDS" -lt "$deadline" ]; do
      if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
        break
      fi
      sleep 0.5
    done
    if [ -n "$field" ]; then
      timeout 45s ros2 topic echo "$topic" --once --field "$field" > "$file" 2>&1 || true
    else
      timeout 45s ros2 topic echo "$topic" --once > "$file" 2>&1 || true
    fi
  ) &
  SAMPLE_PIDS+=("$!")
}

sample_topic_window() {
  local topic="$1"
  local file="$2"
  local duration="${3:-45s}"
  (
    local deadline=$((SECONDS + 20))
    while [ "$SECONDS" -lt "$deadline" ]; do
      if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
        break
      fi
      sleep 0.5
    done
    timeout "$duration" ros2 topic echo "$topic" > "$file" 2>&1 || true
  ) &
  SAMPLE_PIDS+=("$!")
}

sample_gz_topic_window() {
  local topic="$1"
  local file="$2"
  local duration="${3:-45s}"
  timeout "$duration" gz topic -e -t "$topic" > "$file" 2>&1 &
  SAMPLE_PIDS+=("$!")
}

start_evidence_sampling() {
  sample_topic_once /clock "$OUT_DIR/logs/clock_once.txt"
  sample_topic_once /dual_arm_planning/phase "$OUT_DIR/logs/planning_phase_once.txt"
  sample_topic_once /dual_arm_planning/left_joint_trajectory "$OUT_DIR/logs/left_planned_trajectory_once.txt"
  sample_topic_once /dual_arm_planning/right_joint_trajectory "$OUT_DIR/logs/right_planned_trajectory_once.txt"
  sample_topic_once /move_action/_action/status "$OUT_DIR/logs/moveit_action_status_once.txt"
  sample_topic_once /left_camera/image_rect "$OUT_DIR/logs/left_gazebo_camera_image_once.txt" header
  sample_topic_once /right_camera/image_rect "$OUT_DIR/logs/right_gazebo_camera_image_once.txt" header

  case "$DAY_NORM" in
    day01)
      sample_topic_window /rm65b_gripper/upper_finger_cmd "$OUT_DIR/logs/d1_upper_finger_cmd_window.txt" 70s
      sample_topic_window /rm65b_gripper/lower_finger_cmd "$OUT_DIR/logs/d1_lower_finger_cmd_window.txt" 70s
      ;;
    day02)
      sample_topic_window /left_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/left_force_window.txt" 60s
      sample_topic_window /right_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/right_force_window.txt" 60s
      sample_topic_window /force_control/state "$OUT_DIR/logs/force_control_state_window.txt" 60s
      sample_topic_window /force_control/wrench_error "$OUT_DIR/logs/force_control_wrench_error_window.txt" 60s
      sample_topic_window /force_control/impedance_state "$OUT_DIR/logs/force_control_impedance_state_window.txt" 60s
      sample_topic_window /force_control/admittance_offset "$OUT_DIR/logs/force_control_admittance_offset_window.txt" 60s
      sample_topic_window /force_control/corrected_right_joint_states "$OUT_DIR/logs/force_control_corrected_right_joint_states_window.txt" 60s
      sample_topic_window /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_window.txt" 60s
      sample_topic_window /world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact "$OUT_DIR/logs/d2_contact_pad_contact_window.txt" 60s
      sample_topic_window /world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact "$OUT_DIR/logs/d2_force_target_panel_contact_window.txt" 60s
      ;;
    day03)
      sample_topic_once /vision/target_pose "$OUT_DIR/logs/vision_target_once.txt"
      sample_topic_once /vision/debug_image "$OUT_DIR/logs/vision_debug_image_once.txt" header
      sample_topic_window /vision/status "$OUT_DIR/logs/vision_status_once.txt" 55s
      sample_topic_window /vision/metrics "$OUT_DIR/logs/vision_metrics_once.txt" 55s
      sample_topic_window /visual_servo/twist_cmd "$OUT_DIR/logs/visual_servo_twist_window.txt" 55s
      sample_topic_window /visual_servo/aligned "$OUT_DIR/logs/visual_servo_aligned_window.txt" 55s
      sample_topic_window /visual_servo/gazebo_adapter_state "$OUT_DIR/logs/visual_servo_gazebo_adapter_once.txt" 55s
      sample_gz_topic_window /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact "$OUT_DIR/logs/d3_vision_target_contact_gz_window.txt" 55s
      ;;
    day04)
      sample_topic_window /weaving/events "$OUT_DIR/logs/weaving_event_window.txt" 70s
      sample_topic_window /weaving/tension_n "$OUT_DIR/logs/tension_window.txt" 70s
      sample_topic_window /weaving/tension_pid_state "$OUT_DIR/logs/tension_pid_state_window.txt" 70s
      sample_topic_window /weaving/compliance_offset_m "$OUT_DIR/logs/compliance_offset_window.txt" 70s
      sample_topic_window /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_window.txt" 70s
      sample_topic_window /world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact "$OUT_DIR/logs/d4_shuttle_lane_contact_window.txt" 70s
      sample_gz_topic_window /rm65b/physical/d4_right_yarn/state "$OUT_DIR/logs/d4_right_yarn_attach_state.txt" 70s
      sample_gz_topic_window /rm65b/physical/d4_right_probe/state "$OUT_DIR/logs/d4_right_probe_attach_state.txt" 70s
      ;;
    *)
      sample_topic_once /vision/target_pose "$OUT_DIR/logs/vision_target_once.txt"
      sample_topic_once /vision/debug_image "$OUT_DIR/logs/vision_debug_image_once.txt" header
      sample_topic_window /vision/status "$OUT_DIR/logs/vision_status_once.txt" 55s
      sample_topic_window /vision/metrics "$OUT_DIR/logs/vision_metrics_once.txt" 55s
      sample_topic_window /visual_servo/twist_cmd "$OUT_DIR/logs/visual_servo_twist_window.txt" 55s
      sample_topic_window /visual_servo/aligned "$OUT_DIR/logs/visual_servo_aligned_window.txt" 55s
      sample_topic_window /visual_servo/gazebo_adapter_state "$OUT_DIR/logs/visual_servo_gazebo_adapter_once.txt" 55s
      sample_gz_topic_window /world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact "$OUT_DIR/logs/d3_vision_target_contact_gz_window.txt" 55s
      sample_topic_window /force_control/state "$OUT_DIR/logs/force_control_state_window.txt" 70s
      sample_topic_window /force_control/wrench_error "$OUT_DIR/logs/force_control_wrench_error_window.txt" 70s
      sample_topic_window /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_window.txt" 70s
      sample_topic_window /world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact "$OUT_DIR/logs/d2_force_target_panel_contact_window.txt" 70s
      sample_topic_window /world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact "$OUT_DIR/logs/d4_shuttle_lane_contact_window.txt" 70s
      sample_topic_window /weaving/events "$OUT_DIR/logs/weaving_event_window.txt" 70s
      sample_topic_window /weaving/tension_n "$OUT_DIR/logs/tension_window.txt" 70s
      sample_topic_window /weaving/tension_pid_state "$OUT_DIR/logs/tension_pid_state_window.txt" 70s
      sample_topic_window /weaving/compliance_offset_m "$OUT_DIR/logs/compliance_offset_window.txt" 70s
      sample_gz_topic_window /rm65b/physical/d4_right_yarn/state "$OUT_DIR/logs/d4_right_yarn_attach_state.txt" 70s
      sample_gz_topic_window /rm65b/physical/d4_right_probe/state "$OUT_DIR/logs/d4_right_probe_attach_state.txt" 70s
      ;;
  esac
}

wait_evidence_sampling() {
  local pid
  for pid in "${SAMPLE_PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}

stop_gripper_command_stream() {
  if [ -n "${GRIPPER_STOP_FILE:-}" ]; then
    touch "$GRIPPER_STOP_FILE" 2>/dev/null || true
  fi
  if [ -n "${GRIPPER_STREAM_PID:-}" ] && kill -0 "$GRIPPER_STREAM_PID" 2>/dev/null; then
    wait_or_kill "$GRIPPER_STREAM_PID"
  fi
  GRIPPER_STREAM_PID=""
}

send_joint_trajectory() {
  local topic="$1"
  local trajectory_file="$2"
  gz topic -t "$topic" -m gz.msgs.JointTrajectory -p "$(cat "$trajectory_file")" \
    >> "$OUT_DIR/logs/gz_joint_trajectory_command.log" 2>&1 || true
}

publish_gazebo_trajectories_ros() {
  python3 "$GAZEBO_TRAJECTORY_PUBLISHER" \
    --plan-file "$MOVEIT_PLAN_FILE" \
    --repeats 1 \
    --time-offset 0.25 \
    --settle 0.0 \
    > "$OUT_DIR/logs/gazebo_trajectory_publisher.log" 2>&1 || true
}

send_gripper_position() {
  local position="$1"
  printf "%s\n" "$position" > "$GRIPPER_TARGET_FILE"
  echo "$(date --iso-8601=seconds) gripper_target_position=$position" >> "$OUT_DIR/logs/ros_gripper_command.log"
}

moveit_plan_duration() {
  python3 - "$MOVEIT_PLAN_FILE" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}
points = data.get("points") or []
duration = 0.0
if points:
    duration = float(points[-1].get("time_from_start", 0.0) or 0.0)
print(f"{duration:.3f}")
PY
}

prepare_moveit_physical_scene() {
  if [[ "$DAY_NORM" == "day02" ]]; then
    send_gripper_position 0.000
  else
    send_gripper_position 0.018
  fi
  case "$DAY_NORM" in
    day01)
      sleep 1
      ;;
    day04|day05)
      timeout 2s gz topic -t /rm65b/physical/d4_right_yarn/detach -m gz.msgs.Empty -p "" -d 0.2 >/dev/null 2>&1 || true
      timeout 2s gz topic -t /rm65b/physical/d4_right_probe/detach -m gz.msgs.Empty -p "" -d 0.2 >/dev/null 2>&1 || true
      sleep 2
      ;;
  esac
  {
    echo "dynamic_scene_set_pose: disabled"
    echo "prop_motion_policy: props are static or dynamic SDF bodies; runtime motion may only come from Gazebo contacts or model-scoped DetachableJoint fixed joints"
    if [[ "$DAY_NORM" == "day01" ]]; then
      echo "physical_attach: disabled for D1 gripper/TF/dual-arm motion check"
    else
      echo "physical_attach: model-scoped gz-sim-detachable-joint-system"
    fi
    echo "pre_record_detach: completed before camera capture starts for initially attached DetachableJoint props"
  } > "$OUT_DIR/logs/physical_scene_prepare.log"
}

run_moveit_synchronized_playback() {
  local planned_duration
  local playback_seconds
  planned_duration="$(moveit_plan_duration)"
  playback_seconds="$(python3 - "$planned_duration" "$RVIZ_RECORD_SECONDS" <<'PY'
import sys

plan_duration = float(sys.argv[1])
record_seconds = float(sys.argv[2])
print(f"{max(record_seconds, plan_duration + 6.0):.3f}")
PY
)"

  python3 "$SYNC_JOINT_STATE_REPLAY" \
    --plan-file "$MOVEIT_PLAN_FILE" \
    --day-id "$DAY_NORM" \
    --duration "$playback_seconds" \
    --start-delay "$SYNC_REPLAY_START_DELAY" \
    > "$OUT_DIR/logs/sync_joint_state_replay.log" 2>&1 &
  SYNC_JOINT_REPLAY_PID="$!"
  PIDS+=("$SYNC_JOINT_REPLAY_PID")

  if [[ "$DAY_NORM" == "day04" ]]; then
    python3 "$WORKSPACE/scripts/record_d4_real_teach_replay.py" \
      --output-file "$OUT_DIR/logs/day04_recorded_primitives_from_live_joint_states.yaml" \
      --duration 6.0 \
      --wait-joint-state 8.0 \
      > "$OUT_DIR/logs/d4_teach_replay_recorder.log" 2>&1 &
    D4_RECORDER_PID="$!"
    PIDS+=("$D4_RECORDER_PID")
  fi

  python3 "$PHYSICAL_INTERACTION_CONTROLLER" \
    --plan-file "$MOVEIT_PLAN_FILE" \
    --day-id "$DAY_NORM" \
    --duration "$playback_seconds" \
    --rate-hz "$SYNC_SCENE_RATE_HZ" \
    --start-delay "$SYNC_REPLAY_START_DELAY" \
    --gripper-target-file "$GRIPPER_TARGET_FILE" \
    --log-file "$OUT_DIR/logs/physical_interaction_events.log" \
    > "$OUT_DIR/logs/physical_interaction_controller.log" 2>&1 &
  PHYSICAL_CONTROLLER_PID="$!"
  PIDS+=("$PHYSICAL_CONTROLLER_PID")

  sleep "$SYNC_REPLAY_START_DELAY"
  publish_gazebo_trajectories_ros
  send_joint_trajectory "/model/left_rm65b/joint_trajectory" "$MOVEIT_LEFT_GZ_TRAJECTORY"
  send_joint_trajectory "/model/right_rm65b/joint_trajectory" "$MOVEIT_RIGHT_GZ_TRAJECTORY"
  sleep "$playback_seconds"
}

sleep 2
prepare_moveit_physical_scene
start_rviz_recording

{
  echo "day_id: $DAY_ID"
  echo "day_normalized: $DAY_NORM"
  echo "playback_mode: physical synchronized MoveIt replay; Gazebo joint trajectories and RViz joint_states come from $MOVEIT_PLAN_FILE"
  echo "dynamic_prop_set_pose: disabled for all days"
  echo "prop_motion_policy: SDF static/dynamic props may move only by Gazebo collision/contact or model-scoped DetachableJoint fixed joints"
  echo "legacy_handwritten_trajectory: removed from this acceptance recording path"
  echo "base_world: empty world; day-specific props are generated into rm65b_planned_world_runtime.sdf"
  if [ "$TRUE_DUAL_MOVEIT" = "1" ]; then
    echo "trajectory_source: MoveIt2 /move_action via rm65b_dual_arm_moveit_config dual_arms group"
  else
    echo "trajectory_source: MoveIt2 /move_action via rm_65_config move_group"
  fi
  echo "left_base_fixed: left_rm65b spawned once at x=-0.45 y=0 z=0.02 yaw=+1.5708"
  echo "right_base_fixed: right_rm65b spawned once at x=0.45 y=0 z=0.02 yaw=-1.5708"
  echo "robot_motion: Harmonic JointTrajectoryController commands joint1..joint6; no set_pose is issued for left_rm65b/right_rm65b"
  echo "gripper_scale: TCP approximation is x=0.105 m from gripper palm and x=0.123 m from Link6"
  echo "gazebo_gripper: attached gripper has prismatic finger joints gripper_upper_slide/gripper_lower_slide and JointPositionController commands"
  if [[ "$DAY_NORM" == "day02" ]]; then
    echo "d2_dynamics: gravity enabled on arm and gripper links; gripper inertial parameters use identified palm/finger aggregate masses and box inertias"
    echo "d2_gripper_identified_mass_total_kg: 0.360"
    echo "d2_gripper_identified_com_in_gripper_m: 0.028222 0 0"
    echo "d2_gripper_identified_inertia_about_com_kgm2: ixx=0.0000607867 iyy=0.0004030022 izz=0.0004525089"
  fi
  echo "gazebo_gripper_command_topics: /rm65b_gripper/upper_finger_cmd /rm65b_gripper/lower_finger_cmd bridged from ROS Float64 to Gazebo Double"
  if [[ "$DAY_NORM" == "day01" ]]; then
    echo "physical_attach: disabled for D1 gripper/TF/dual-arm motion check"
  else
    echo "physical_attach: model-scoped gz-sim-detachable-joint-system"
  fi
  echo "contact_sensor_topics: /contacts/d2_force_target_panel /contacts/d2_contact_pad /contacts/vision_target /contacts/d2_left_compliance_bar /contacts/d2_right_compliance_bar /contacts/d4_upper_loom_rail /contacts/d4_lower_loom_rail /contacts/d4_shuttle_lane /contacts/d4_tension_scale"
  echo "contact_sensor_scoped_topics: /world/rm65b_world/model/*/link/link/sensor/*_contact/contact bridged from Gazebo Harmonic Contact system"
  echo "gazebo_left_camera: /left_camera/image_rect is produced by an eye-in-hand camera sensor mounted on the left arm gripper palm"
  echo "gazebo_right_camera: /right_camera/image_rect is produced by an eye-in-hand camera sensor mounted on the right arm gripper palm"
  echo "vision_loop: left-arm eye-in-hand /left_camera/image_rect -> aruco_target_node -> ibvs_controller -> visual_servo_gazebo_adapter -> /model/left_rm65b/joint_trajectory"
  echo "vision_debug_image: /vision/debug_image is the OpenCV annotated marker-recognition video source"
  echo "force_control_loop: /right_rm_driver/rm_driver/udp_six_force -> /force_control/wrench_error -> /force_control/admittance_offset -> /force_control/corrected_right_joint_trajectory -> /model/right_rm65b/joint_trajectory"
  echo "force_feedback_source: Gazebo Harmonic contact topics -> gazebo_contact_force_estimator -> /right_rm_driver/rm_driver/udp_six_force"
  echo "d4_tension_model: spring-mass-damper yarn model with PID and compliance parameters from d4_tension_pid_compliance.yaml"
  case "$DAY_NORM" in
    day01)
      echo "video_acceptance_design: show gripper open/close states, Link6-to-gripper transform, RViz TF frames, and true dual-arm synchronous/alternating MoveIt motion"
      ;;
    day02)
      echo "video_acceptance_design: show D2 dynamics-enabled gripper, solid contact force change, admittance/impedance state, corrected right-arm joint motion, and /joint_states plus force topics"
      echo "day02_left_camera_role: left eye-in-hand camera is yawed toward the force station and used as an observer view of the right-arm contact task"
      ;;
    day03)
      echo "video_acceptance_design: left eye-in-hand camera sees the fixed target at the first frame, /vision/debug_image overlays marker recognition, then visual-servo commands correct the left arm toward the target"
      echo "hand_eye_transform_gripper_to_camera_m: translation=0.034 0 0.095 rpy=0 0.35 0.38 matrix=[[0.872362,-0.370920,0.318437,0.034],[0.348433,0.928665,0.127188,0],[-0.342898,0,0.939373,0.095],[0,0,0,1]]"
      ;;
    day04)
      echo "video_acceptance_design: show loom rails with material-coded warp/weft yarn, BT states hook/lift/pull/shift/exchange, live joint-state teach recorder, replay YAML, tension PID/compliance telemetry, and contact feedback"
      echo "teach_replay_recorder_output: $OUT_DIR/logs/day04_recorded_primitives_from_live_joint_states.yaml"
      ;;
    *)
      echo "video_acceptance_design: show integrated vision -> gripper -> MoveIt/BT weaving -> force/contact/tension -> final cloth strip workflow"
      ;;
  esac
} > "$OUT_DIR/logs/planning_playback_notes.txt"

start_camera_recording
sleep 2
start_evidence_sampling
run_moveit_synchronized_playback
wait_evidence_sampling
stop_gripper_command_stream

if [ -n "${RVIZ_RECORD_PID:-}" ]; then
  wait "$RVIZ_RECORD_PID" 2>/dev/null || true
  RVIZ_RECORD_PID=""
  if [ -s "$OUT_DIR/videos/${DAY_NORM}_rviz.mp4" ]; then
    ffmpeg -y -ss 2 -i "$OUT_DIR/videos/${DAY_NORM}_rviz.mp4" -frames:v 1 \
      "$OUT_DIR/screenshots/05_rviz_overview.png" \
      > "$OUT_DIR/logs/ffmpeg_screenshot_05_rviz.log" 2>&1 || true
  fi
fi

timeout 4s gz topic -e -t /model/left_rm65b/joint_trajectory_progress \
  > "$OUT_DIR/logs/left_joint_trajectory_progress.txt" 2>&1 || true
timeout 4s gz topic -e -t /model/right_rm65b/joint_trajectory_progress \
  > "$OUT_DIR/logs/right_joint_trajectory_progress.txt" 2>&1 || true

wait "$CAPTURE_PID" || true
CAPTURE_PID=""
wait "$LEFT_CAPTURE_PID" || true
LEFT_CAPTURE_PID=""
wait "$RIGHT_CAPTURE_PID" || true
RIGHT_CAPTURE_PID=""
if [ -n "${VISION_DEBUG_CAPTURE_PID:-}" ]; then
  wait "$VISION_DEBUG_CAPTURE_PID" || true
  VISION_DEBUG_CAPTURE_PID=""
fi

if [ -n "${D4_RECORDER_PID:-}" ]; then
  wait "$D4_RECORDER_PID" 2>/dev/null || true
  D4_RECORDER_PID=""
fi

mapfile -t FRAME_FILES < <(find "$OUT_DIR/camera_frames" -maxdepth 1 -name 'camera_*.ppm' | sort)
ACTUAL_FRAME_COUNT="${#FRAME_FILES[@]}"
{
  echo "requested_frame_count: $FRAME_COUNT"
  echo "actual_frame_count: $ACTUAL_FRAME_COUNT"
  echo "capture_timeout_s: $CAPTURE_TIMEOUT"
} > "$OUT_DIR/logs/capture_summary.txt"

mapfile -t LEFT_FRAME_FILES < <(find "$OUT_DIR/left_camera_frames" -maxdepth 1 -name 'camera_*.ppm' | sort)
LEFT_ACTUAL_FRAME_COUNT="${#LEFT_FRAME_FILES[@]}"
{
  echo "requested_frame_count: $FRAME_COUNT"
  echo "actual_frame_count: $LEFT_ACTUAL_FRAME_COUNT"
  echo "source_topic: /left_camera/image_rect"
  echo "source: Gazebo Harmonic eye-in-hand camera sensor on left arm gripper palm"
} > "$OUT_DIR/logs/left_gazebo_camera_capture_summary.txt"

mapfile -t RIGHT_FRAME_FILES < <(find "$OUT_DIR/right_camera_frames" -maxdepth 1 -name 'camera_*.ppm' | sort)
RIGHT_ACTUAL_FRAME_COUNT="${#RIGHT_FRAME_FILES[@]}"
{
  echo "requested_frame_count: $FRAME_COUNT"
  echo "actual_frame_count: $RIGHT_ACTUAL_FRAME_COUNT"
  echo "source_topic: /right_camera/image_rect"
  echo "source: Gazebo Harmonic eye-in-hand camera sensor on right arm gripper palm"
} > "$OUT_DIR/logs/right_gazebo_camera_capture_summary.txt"

mapfile -t VISION_DEBUG_FRAME_FILES < <(find "$OUT_DIR/vision_debug_frames" -maxdepth 1 -name 'camera_*.ppm' | sort)
VISION_DEBUG_ACTUAL_FRAME_COUNT="${#VISION_DEBUG_FRAME_FILES[@]}"
{
  echo "requested_frame_count: $FRAME_COUNT"
  echo "actual_frame_count: $VISION_DEBUG_ACTUAL_FRAME_COUNT"
  echo "source_topic: /vision/debug_image"
  echo "source: OpenCV annotated marker-recognition output from aruco_target_node"
} > "$OUT_DIR/logs/vision_debug_capture_summary.txt"

ffmpeg -y -framerate 10 -i "$OUT_DIR/camera_frames/camera_%04d.ppm" \
  -pix_fmt yuv420p "$OUT_DIR/videos/${DAY_ID}_moveit_harmonic_playback.mp4" \
  > "$OUT_DIR/logs/ffmpeg_camera_video.log" 2>&1 || true

ffmpeg -y -framerate 10 -i "$OUT_DIR/left_camera_frames/camera_%04d.ppm" \
  -pix_fmt yuv420p "$OUT_DIR/videos/${DAY_ID}_left_gazebo_camera.mp4" \
  > "$OUT_DIR/logs/ffmpeg_left_gazebo_camera_video.log" 2>&1 || true

ffmpeg -y -framerate 10 -i "$OUT_DIR/right_camera_frames/camera_%04d.ppm" \
  -pix_fmt yuv420p "$OUT_DIR/videos/${DAY_ID}_right_gazebo_camera.mp4" \
  > "$OUT_DIR/logs/ffmpeg_right_gazebo_camera_video.log" 2>&1 || true

ffmpeg -y -framerate 10 -i "$OUT_DIR/vision_debug_frames/camera_%04d.ppm" \
  -pix_fmt yuv420p "$OUT_DIR/videos/${DAY_ID}_opencv_marker_debug.mp4" \
  > "$OUT_DIR/logs/ffmpeg_vision_debug_video.log" 2>&1 || true

if [ "$ACTUAL_FRAME_COUNT" -gt 0 ]; then
  FIRST_INDEX=15
  if [ "$FIRST_INDEX" -ge "$ACTUAL_FRAME_COUNT" ]; then
    FIRST_INDEX=0
  fi
  MID_INDEX=$((ACTUAL_FRAME_COUNT / 2))
  LAST_INDEX=$((ACTUAL_FRAME_COUNT - 1))

  ffmpeg -y -i "${FRAME_FILES[$FIRST_INDEX]}" "$OUT_DIR/screenshots/01_fixed_bases_loaded.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_01.log" 2>&1 || true
  ffmpeg -y -i "${FRAME_FILES[$MID_INDEX]}" "$OUT_DIR/screenshots/02_moveit_joint_planning.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_02.log" 2>&1 || true
  ffmpeg -y -i "${FRAME_FILES[$LAST_INDEX]}" "$OUT_DIR/screenshots/03_day_result.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_03.log" 2>&1 || true
fi

if [ "$LEFT_ACTUAL_FRAME_COUNT" -gt 0 ]; then
  LEFT_MID_INDEX=$((LEFT_ACTUAL_FRAME_COUNT / 2))
  ffmpeg -y -i "${LEFT_FRAME_FILES[$LEFT_MID_INDEX]}" "$OUT_DIR/screenshots/04_left_gazebo_camera_view.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_04_left_camera.log" 2>&1 || true
fi

if [ "$RIGHT_ACTUAL_FRAME_COUNT" -gt 0 ]; then
  RIGHT_MID_INDEX=$((RIGHT_ACTUAL_FRAME_COUNT / 2))
  ffmpeg -y -i "${RIGHT_FRAME_FILES[$RIGHT_MID_INDEX]}" "$OUT_DIR/screenshots/06_right_gazebo_camera_view.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_06_right_camera.log" 2>&1 || true
fi

if [ "$VISION_DEBUG_ACTUAL_FRAME_COUNT" -gt 0 ]; then
  VISION_DEBUG_MID_INDEX=$((VISION_DEBUG_ACTUAL_FRAME_COUNT / 2))
  ffmpeg -y -i "${VISION_DEBUG_FRAME_FILES[$VISION_DEBUG_MID_INDEX]}" "$OUT_DIR/screenshots/07_opencv_marker_debug.png" \
    > "$OUT_DIR/logs/ffmpeg_screenshot_07_vision_debug.log" 2>&1 || true
fi

late_topic_once() {
  local topic="$1"
  local file="$2"
  if [ -s "$file" ]; then
    return
  fi
  timeout 5s ros2 topic echo "$topic" --once > "$file" 2>&1 || true
}

late_topic_once /dual_arm_planning/phase "$OUT_DIR/logs/planning_phase_once.txt"
late_topic_once /dual_arm_planning/left_joint_trajectory "$OUT_DIR/logs/left_planned_trajectory_once.txt"
late_topic_once /move_action/_action/status "$OUT_DIR/logs/moveit_action_status_once.txt"

case "$DAY_NORM" in
  day02)
    late_topic_once /left_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/left_force_once.txt"
    late_topic_once /right_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/right_force_once.txt"
    late_topic_once /force_control/state "$OUT_DIR/logs/force_control_state_once.txt"
    late_topic_once /force_control/wrench_error "$OUT_DIR/logs/force_control_wrench_error_once.txt"
    late_topic_once /force_control/gazebo_relay_state "$OUT_DIR/logs/force_gazebo_relay_state_once.txt"
    late_topic_once /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_once.txt"
    ;;
  day03)
    late_topic_once /vision/target_pose "$OUT_DIR/logs/vision_target_once.txt"
    late_topic_once /vision/status "$OUT_DIR/logs/vision_status_once.txt"
    late_topic_once /vision/metrics "$OUT_DIR/logs/vision_metrics_once.txt"
    late_topic_once /vision/debug_image "$OUT_DIR/logs/vision_debug_image_once.txt"
    late_topic_once /visual_servo/gazebo_adapter_state "$OUT_DIR/logs/visual_servo_gazebo_adapter_once.txt"
    ;;
  day04)
    late_topic_once /weaving/events "$OUT_DIR/logs/weaving_event_once.txt"
    late_topic_once /weaving/tension_n "$OUT_DIR/logs/tension_once.txt"
    late_topic_once /weaving/tension_pid_state "$OUT_DIR/logs/tension_pid_state_once.txt"
    late_topic_once /weaving/compliance_offset_m "$OUT_DIR/logs/compliance_offset_once.txt"
    late_topic_once /force_control/state "$OUT_DIR/logs/force_control_state_once.txt"
    late_topic_once /force_control/wrench_error "$OUT_DIR/logs/force_control_wrench_error_once.txt"
    late_topic_once /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_once.txt"
    ;;
  day05)
    late_topic_once /weaving/events "$OUT_DIR/logs/weaving_event_once.txt"
    late_topic_once /weaving/tension_n "$OUT_DIR/logs/tension_once.txt"
    late_topic_once /weaving/tension_pid_state "$OUT_DIR/logs/tension_pid_state_once.txt"
    late_topic_once /weaving/compliance_offset_m "$OUT_DIR/logs/compliance_offset_once.txt"
    late_topic_once /left_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/left_force_once.txt"
    late_topic_once /right_rm_driver/rm_driver/udp_six_force "$OUT_DIR/logs/right_force_once.txt"
    late_topic_once /vision/target_pose "$OUT_DIR/logs/vision_target_once.txt"
    late_topic_once /vision/status "$OUT_DIR/logs/vision_status_once.txt"
    late_topic_once /vision/debug_image "$OUT_DIR/logs/vision_debug_image_once.txt"
    late_topic_once /vision/metrics "$OUT_DIR/logs/vision_metrics_once.txt"
    late_topic_once /force_control/state "$OUT_DIR/logs/force_control_state_once.txt"
    late_topic_once /force_control/wrench_error "$OUT_DIR/logs/force_control_wrench_error_once.txt"
    late_topic_once /force_control/gazebo_relay_state "$OUT_DIR/logs/force_gazebo_relay_state_once.txt"
    late_topic_once /force_control/gazebo_contact_force_state "$OUT_DIR/logs/gazebo_contact_force_state_once.txt"
    late_topic_once /visual_servo/gazebo_adapter_state "$OUT_DIR/logs/visual_servo_gazebo_adapter_once.txt"
    ;;
esac

late_topic_once /left_camera/image_rect "$OUT_DIR/logs/left_gazebo_camera_image_once.txt"
late_topic_once /right_camera/image_rect "$OUT_DIR/logs/right_gazebo_camera_image_once.txt"
late_topic_once /clock "$OUT_DIR/logs/clock_once.txt"

echo "${DAY_ID}_moveit_harmonic_playback_complete" | tee "$OUT_DIR/logs/status.txt"
