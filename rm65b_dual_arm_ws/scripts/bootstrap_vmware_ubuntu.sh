#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$WORKSPACE/.." && pwd)"

SOURCE_URL="https://github.com/RealManRobot/ros2_rm_robot/archive/refs/heads/humble.zip"
ARCHIVE_PATH="$REPO_ROOT/downloads/ros2_rm_robot_humble.zip"
EXTRACT_DIR="$REPO_ROOT/downloads/ros2_rm_robot_humble"
TARGET_DIR="$WORKSPACE/src/ros2_rm_robot"
OFFICIAL_SOURCE_DIR=""
EXISTING_SETUP=""
SKIP_DOWNLOAD=0
FORCE_REFRESH_TARGET=0
SKIP_PROJECT_VERIFY=0
SETUP_APT_SOURCES=0
INSTALL_APT_DEPS=0
RUN_BUILD=0
CLEAN_BUILD=0

official_required=(
  README.md
  rm_bringup/package.xml
  rm_description/package.xml
  rm_driver/package.xml
  rm_moveit2_config/rm_65_config/package.xml
  rm_ros_interfaces/package.xml
)

workspace_required=(
  scripts/verify_project.py
  src/rm65b_dual_arm_bringup/launch/full_system.launch.py
  src/rm65b_dual_arm_bringup/config/dual_arm_frames.yaml
  src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.urdf
  src/rm65b_dual_arm_planning/package.xml
  src/rm65b_gripper_control/package.xml
  src/rm65b_safety/package.xml
  src/rm65b_vision_guidance/package.xml
  src/rm65b_weaving_primitives/config/weaving_tree.xml
)

usage() {
  cat <<EOF
Usage: bash scripts/bootstrap_vmware_ubuntu.sh [options]

Options:
  --root PATH                 Repository root. Default: parent of this workspace.
  --source-url URL            Official ros2_rm_robot humble zip URL.
  --official-source-dir PATH  Existing ros2_rm_robot source tree to reuse.
  --existing-setup PATH       Existing workspace install/setup.bash to source as an underlay.
  --archive PATH              Downloaded archive path.
  --extract-dir PATH          Extract destination.
  --skip-download             Fail instead of downloading a missing archive.
  --force-refresh-target      Replace an incomplete src/ros2_rm_robot target.
  --skip-project-verify       Skip scripts/verify_project.py.
  --setup-apt-sources         Configure official ROS2 and Gazebo apt sources.
  --install-apt-deps          Install missing apt dependencies with sudo apt-get.
  --build                     Run colcon build after checks.
  --clean-build               Remove this workspace's build/install/log before building.
  --all                       Install apt dependencies, clean generated state, and build.
  -h, --help                  Show this help.
EOF
}

log() {
  printf '[%s] %s\n' "$1" "$2"
}

die() {
  log FAIL "$1" >&2
  exit 1
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

source_setup_file() {
  local setup_file="$1"
  # ROS/ament setup files may read unset variables internally; keep this
  # bootstrap script strict while sourcing those external files.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

remove_generated_package_dir() {
  local path="$1"
  [[ -e "$path" ]] || return 0
  case "$path" in
    "$WORKSPACE"/build/*|"$WORKSPACE"/install/*|"$WORKSPACE"/log/*)
      rm -rf "$path"
      ;;
    *)
      die "Refusing to remove generated path outside workspace build/install/log: $path"
      ;;
  esac
}

clean_package_build_state() {
  local package_name="$1"
  log INFO "Cleaning generated colcon state for $package_name."
  remove_generated_package_dir "$WORKSPACE/build/$package_name"
  remove_generated_package_dir "$WORKSPACE/install/$package_name"
  remove_generated_package_dir "$WORKSPACE/log/latest_build/$package_name"
}

clean_workspace_build_state() {
  log WARN "Cleaning generated build/install/log under $WORKSPACE."
  local path
  for path in "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"; do
    [[ -e "$path" ]] || continue
    case "$path" in
      "$WORKSPACE"/build|"$WORKSPACE"/install|"$WORKSPACE"/log)
        rm -rf "$path"
        ;;
      *)
        die "Refusing to remove generated path outside workspace: $path"
        ;;
    esac
  done
}

sudo_cmd() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

apt_package_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'
}

apt_package_available() {
  apt-cache show "$1" >/dev/null 2>&1
}

missing_paths() {
  local base="$1"
  shift
  local missing=()
  local rel
  for rel in "$@"; do
    if [[ ! -e "$base/$rel" ]]; then
      missing+=("$rel")
    fi
  done
  printf '%s\n' "${missing[@]}"
}

tree_is_valid() {
  local base="$1"
  shift
  [[ -d "$base" ]] || return 1
  [[ -z "$(missing_paths "$base" "$@")" ]]
}

find_official_source() {
  local candidates=(
    "$OFFICIAL_SOURCE_DIR"
    "$TARGET_DIR"
    "$EXTRACT_DIR/ros2_rm_robot-humble"
    "$EXTRACT_DIR/ros2_rm_robot"
    "$REPO_ROOT/downloads/ros2_rm_robot-humble"
    "$REPO_ROOT/downloads/ros2_rm_robot"
    "$HOME/ros2_ws/src/ros2_rm_robot"
    "$HOME/rm65b_dual_arm_ws/src/ros2_rm_robot"
    "$HOME/rm65b_ws_recording/src/ros2_rm_robot"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    [[ -n "$candidate" ]] || continue
    if tree_is_valid "$candidate" "${official_required[@]}"; then
      (cd "$candidate" && pwd)
      return 0
    fi
  done
  return 1
}

suggest_existing_workspaces() {
  local candidates=(
    "$HOME/ros2_ws"
    "$HOME/rm65b_dual_arm_ws"
    "$HOME/rm65b_ws_recording"
  )
  local ws
  for ws in "${candidates[@]}"; do
    if tree_is_valid "$ws/src/ros2_rm_robot" "${official_required[@]}"; then
      log INFO "Reusable official source candidate: $ws/src/ros2_rm_robot"
    fi
    if [[ -f "$ws/install/setup.bash" ]]; then
      log INFO "Reusable built underlay candidate: $ws/install/setup.bash"
    fi
  done
}

download_archive() {
  mkdir -p "$(dirname "$ARCHIVE_PATH")"
  if [[ -f "$ARCHIVE_PATH" ]]; then
    log OK "Archive already exists: $ARCHIVE_PATH"
    return 0
  fi
  if [[ "$SKIP_DOWNLOAD" -eq 1 ]]; then
    die "Official source archive is missing and --skip-download was specified: $ARCHIVE_PATH"
  fi

  log INFO "Downloading official ros2_rm_robot humble source."
  local partial="${ARCHIVE_PATH}.partial"
  rm -f "$partial"
  if have_cmd curl; then
    curl -L --fail --retry 3 -o "$partial" "$SOURCE_URL"
  elif have_cmd wget; then
    wget -O "$partial" "$SOURCE_URL"
  else
    die "Neither curl nor wget is available; cannot download $SOURCE_URL"
  fi
  mv "$partial" "$ARCHIVE_PATH"
  log OK "Downloaded: $ARCHIVE_PATH"
}

extract_archive() {
  mkdir -p "$EXTRACT_DIR"
  log INFO "Extracting official source archive."
  if have_cmd unzip; then
    unzip -q -o "$ARCHIVE_PATH" -d "$EXTRACT_DIR"
  elif have_cmd python3; then
    python3 -m zipfile -e "$ARCHIVE_PATH" "$EXTRACT_DIR"
  else
    die "Neither unzip nor python3 is available; cannot extract $ARCHIVE_PATH"
  fi
}

copy_official_source() {
  local source_dir="$1"
  mkdir -p "$WORKSPACE/src"

  if [[ -d "$TARGET_DIR" ]]; then
    if tree_is_valid "$TARGET_DIR" "${official_required[@]}"; then
      log OK "Workspace official source already exists: $TARGET_DIR"
      return 0
    fi
    if [[ "$FORCE_REFRESH_TARGET" -ne 1 ]]; then
      log FAIL "Workspace official source exists but is incomplete: $TARGET_DIR"
      missing_paths "$TARGET_DIR" "${official_required[@]}" | sed 's/^/  missing: /' >&2
      die "Re-run with --force-refresh-target to replace it."
    fi

    case "$TARGET_DIR" in
      "$WORKSPACE/src/ros2_rm_robot") ;;
      *) die "Refusing to remove unexpected target path: $TARGET_DIR" ;;
    esac
    log WARN "Refreshing incomplete official source target: $TARGET_DIR"
    rm -rf "$TARGET_DIR"
  fi

  cp -a "$source_dir" "$TARGET_DIR"
  log OK "Copied official source to workspace: $TARGET_DIR"
}

check_workspace_files() {
  local missing
  missing="$(missing_paths "$WORKSPACE" "${workspace_required[@]}")"
  if [[ -n "$missing" ]]; then
    printf '%s\n' "$missing" | sed 's/^/  missing: /' >&2
    die "Workspace required files are missing."
  fi
  log OK "Workspace required files are present."
}

install_available_apt_packages() {
  local label="$1"
  shift
  local installable=()
  local unavailable=()
  local package
  for package in "$@"; do
    if apt_package_installed "$package"; then
      log OK "Already installed: $package"
    elif apt_package_available "$package"; then
      installable+=("$package")
    else
      unavailable+=("$package")
    fi
  done

  if [[ "${#installable[@]}" -gt 0 ]]; then
    log INFO "Installing $label packages: ${installable[*]}"
    sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get \
      -o DPkg::Lock::Timeout=120 \
      install -y "${installable[@]}"
  fi

  for package in "${unavailable[@]}"; do
    log WARN "apt package is not available from current sources: $package"
  done
}

setup_apt_sources_if_requested() {
  if [[ "$SETUP_APT_SOURCES" -ne 1 ]]; then
    log INFO "Apt source setup not requested. Add --setup-apt-sources for fresh Ubuntu systems."
    return 0
  fi
  have_cmd apt-get || die "apt-get is not available on this system."
  if [[ "$(id -u)" -ne 0 ]] && ! have_cmd sudo; then
    die "sudo is required for --setup-apt-sources when not running as root."
  fi

  local codename=""
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
  fi
  if [[ -z "$codename" ]] && have_cmd lsb_release; then
    codename="$(lsb_release -cs)"
  fi
  [[ -n "$codename" ]] || die "Could not determine Ubuntu codename for apt sources."

  log INFO "Installing apt source helper packages."
  sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 update
  sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 install -y \
    ca-certificates curl gnupg lsb-release software-properties-common

  log INFO "Enabling Ubuntu universe repository."
  sudo_cmd add-apt-repository -y universe

  log INFO "Configuring ROS2 apt source for Ubuntu $codename."
  local ros_source_version=""
  ros_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}' | head -n 1 || true)"
  if [[ -n "$ros_source_version" ]]; then
    curl -fsSL \
      -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.${codename}_all.deb"
    sudo_cmd dpkg -i /tmp/ros2-apt-source.deb
  else
    log WARN "Could not resolve latest ros2-apt-source release; using legacy ros2.list setup."
    sudo_cmd curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $codename main" \
      | sudo_cmd tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi

  log INFO "Configuring Gazebo Harmonic apt source for Ubuntu $codename."
  sudo_cmd curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $codename main" \
    | sudo_cmd tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

  log INFO "Refreshing apt package index after source setup."
  sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 update
  log OK "Apt source setup complete."
}

install_apt_deps_if_requested() {
  if [[ "$INSTALL_APT_DEPS" -ne 1 ]]; then
    log INFO "Apt dependency installation not requested. Add --install-apt-deps to install missing packages."
    return 0
  fi
  have_cmd apt-get || die "apt-get is not available on this system."
  have_cmd dpkg-query || die "dpkg-query is not available on this system."
  if [[ "$(id -u)" -ne 0 ]] && ! have_cmd sudo; then
    die "sudo is required for --install-apt-deps when not running as root."
  fi

  log INFO "Updating apt package index."
  sudo_cmd env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 update

  local base_packages=(
    build-essential
    ca-certificates
    cmake
    curl
    ffmpeg
    git
    gnupg
    lsb-release
    python3-colcon-common-extensions
    python3-numpy
    python3-opencv
    python3-pip
    python3-yaml
    unzip
    wget
    x11-utils
    libgl1-mesa-dri
    libxkbcommon-x11-0
    libxcb-xinerama0
    mesa-utils
    xdotool
  )
  install_available_apt_packages "base" "${base_packages[@]}"

  local ros_packages=(
    ros-dev-tools
    python3-rosdep
    ros-humble-ros-base
    ros-humble-rviz2
    ros-humble-xacro
    ros-humble-robot-state-publisher
    ros-humble-joint-state-publisher
    ros-humble-joint-state-publisher-gui
    ros-humble-tf2-tools
    ros-humble-control-msgs
    ros-humble-behaviortree-cpp-v3
    ros-humble-ros2-control
    ros-humble-ros2-controllers
    ros-humble-moveit
    ros-humble-ros-gzharmonic
    ros-humble-ros-gzharmonic-bridge
    ros-humble-ros-gzharmonic-image
    ros-humble-ros-gzharmonic-sim
    ros-humble-ros-gzharmonic-sim-demos
  )
  install_available_apt_packages "ROS2/Gazebo" "${ros_packages[@]}"

  local gazebo_packages=(
    gz-harmonic
    gz-sim8-cli
    gz-tools2
    libgz-sim8-dev
    libsdformat14-dev
    libyaml-cpp-dev
  )
  install_available_apt_packages "Gazebo Harmonic" "${gazebo_packages[@]}"

  log OK "Apt dependency installation pass complete."
}

check_ubuntu_environment() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    log INFO "OS: ${PRETTY_NAME:-unknown}"
    if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
      log WARN "ROS2 Humble target is Ubuntu 22.04; current VERSION_ID=${VERSION_ID:-unknown}"
    fi
  fi

  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source_setup_file /opt/ros/humble/setup.bash
    log OK "ROS2 Humble setup found: /opt/ros/humble/setup.bash"
  else
    log WARN "ROS2 Humble setup is missing: /opt/ros/humble/setup.bash"
    log INFO "Official installer is available after bootstrap at: $TARGET_DIR/rm_install/scripts/ros2_install.sh"
  fi

  if [[ -n "$EXISTING_SETUP" ]]; then
    if [[ ! -f "$EXISTING_SETUP" ]]; then
      die "Existing setup file does not exist: $EXISTING_SETUP"
    fi
    source_setup_file "$EXISTING_SETUP"
    log OK "Existing RealMan/ROS workspace underlay sourced: $EXISTING_SETUP"
  else
    suggest_existing_workspaces
  fi
  if [[ "$RUN_BUILD" -ne 1 && -f "$WORKSPACE/install/setup.bash" ]]; then
    source_setup_file "$WORKSPACE/install/setup.bash"
    log OK "Current workspace setup found: $WORKSPACE/install/setup.bash"
  fi

  local required_commands=(python3 ros2 colcon)
  local optional_commands=(git cmake unzip curl wget gz ffmpeg xdotool xwininfo)
  local cmd
  for cmd in "${required_commands[@]}"; do
    if have_cmd "$cmd"; then
      log OK "$cmd: $(command -v "$cmd")"
    else
      log WARN "$cmd is missing"
    fi
  done
  for cmd in "${optional_commands[@]}"; do
    if have_cmd "$cmd"; then
      log OK "$cmd: $(command -v "$cmd")"
    else
      log WARN "$cmd is missing"
    fi
  done

  if have_cmd python3; then
    if python3 - <<'PY' >/dev/null 2>&1
import yaml
PY
    then
      log OK "python3 PyYAML is available."
    else
      log WARN "python3 PyYAML is missing; install python3-yaml."
    fi
    if python3 - <<'PY' >/dev/null 2>&1
import numpy
PY
    then
      log OK "python3 NumPy is available."
    else
      log WARN "python3 NumPy is missing; install python3-numpy."
    fi
    if python3 - <<'PY' >/dev/null 2>&1
import cv2
PY
    then
      log OK "python3 OpenCV is available."
    else
      log WARN "python3 OpenCV is missing; install python3-opencv for D3 vision checks."
    fi
  fi

  if have_cmd ros2; then
    ros2 pkg prefix rm_driver >/dev/null 2>&1 \
      && log OK "Existing rm_driver package is visible to ROS." \
      || log WARN "rm_driver is not visible yet; build this workspace or source an existing setup."
    ros2 pkg prefix rm_ros_interfaces >/dev/null 2>&1 \
      && log OK "Existing rm_ros_interfaces package is visible to ROS." \
      || log WARN "rm_ros_interfaces is not visible yet; build this workspace or source an existing setup."
    ros2 pkg prefix rm_description >/dev/null 2>&1 \
      && log OK "Existing rm_description package is visible to ROS." \
      || log WARN "rm_description is not visible yet; build this workspace or source an existing setup."
    ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1 \
      && log OK "MoveIt2 package moveit_ros_move_group is available." \
      || log WARN "MoveIt2 package moveit_ros_move_group is missing."
    ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 \
      && log OK "ros_gz_bridge is available." \
      || log WARN "ros_gz_bridge is missing."
    ros2 pkg prefix behaviortree_cpp_v3 >/dev/null 2>&1 \
      && log OK "BehaviorTree.CPP v3 package is available." \
      || log WARN "BehaviorTree.CPP v3 package is missing."
  fi
}

run_project_verify() {
  if [[ "$SKIP_PROJECT_VERIFY" -eq 1 ]]; then
    log WARN "Skipped offline project verification."
    return 0
  fi
  if [[ ! -d "$REPO_ROOT/docs" ]]; then
    log WARN "Skipped offline project verification because repo docs are not present at $REPO_ROOT/docs."
    log WARN "This is expected when only rm65b_dual_arm_ws was copied into the Ubuntu workspace."
    return 0
  fi
  if ! have_cmd python3; then
    log WARN "Skipped offline project verification because python3 is missing."
    return 0
  fi
  log INFO "Running offline project verification."
  (cd "$WORKSPACE" && python3 scripts/verify_project.py)
}

run_build_if_requested() {
  if [[ "$RUN_BUILD" -ne 1 ]]; then
    log INFO "Build not requested. Add --build to run colcon build."
    return 0
  fi
  [[ -f /opt/ros/humble/setup.bash ]] || die "Cannot build because /opt/ros/humble/setup.bash is missing."
  have_cmd colcon || die "Cannot build because colcon is missing."

  log INFO "Building workspace."
  (
    cd "$WORKSPACE"
    source_setup_file /opt/ros/humble/setup.bash
    if [[ -n "$EXISTING_SETUP" ]]; then
      source_setup_file "$EXISTING_SETUP"
    fi
    if [[ "$CLEAN_BUILD" -eq 1 ]]; then
      clean_workspace_build_state
    fi
    clean_package_build_state rm_ros_interfaces
    colcon build --symlink-install --packages-select rm_ros_interfaces
    source_setup_file install/setup.bash
    colcon build --symlink-install --packages-skip rm_ros_interfaces
  )
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --root)
      REPO_ROOT="$(cd "$2" && pwd)"
      WORKSPACE="$REPO_ROOT/rm65b_dual_arm_ws"
      SCRIPT_DIR="$WORKSPACE/scripts"
      TARGET_DIR="$WORKSPACE/src/ros2_rm_robot"
      ARCHIVE_PATH="$REPO_ROOT/downloads/ros2_rm_robot_humble.zip"
      EXTRACT_DIR="$REPO_ROOT/downloads/ros2_rm_robot_humble"
      shift 2
      ;;
    --source-url)
      SOURCE_URL="$2"
      shift 2
      ;;
    --official-source-dir)
      OFFICIAL_SOURCE_DIR="$(cd "$2" && pwd)"
      shift 2
      ;;
    --existing-setup)
      EXISTING_SETUP="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
      shift 2
      ;;
    --archive)
      ARCHIVE_PATH="$2"
      shift 2
      ;;
    --extract-dir)
      EXTRACT_DIR="$2"
      shift 2
      ;;
    --skip-download)
      SKIP_DOWNLOAD=1
      shift
      ;;
    --force-refresh-target)
      FORCE_REFRESH_TARGET=1
      shift
      ;;
    --skip-project-verify)
      SKIP_PROJECT_VERIFY=1
      shift
      ;;
    --setup-apt-sources)
      SETUP_APT_SOURCES=1
      shift
      ;;
    --install-apt-deps)
      INSTALL_APT_DEPS=1
      shift
      ;;
    --build)
      RUN_BUILD=1
      shift
      ;;
    --clean-build)
      CLEAN_BUILD=1
      RUN_BUILD=1
      shift
      ;;
    --all)
      SETUP_APT_SOURCES=1
      INSTALL_APT_DEPS=1
      RUN_BUILD=1
      CLEAN_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

log INFO "Repository root: $REPO_ROOT"
log INFO "ROS2 workspace: $WORKSPACE"
log INFO "Official source URL: $SOURCE_URL"
if [[ -n "$OFFICIAL_SOURCE_DIR" ]]; then
  log INFO "Preferred existing official source: $OFFICIAL_SOURCE_DIR"
fi
if [[ -n "$EXISTING_SETUP" ]]; then
  log INFO "Existing underlay setup: $EXISTING_SETUP"
fi

mkdir -p "$REPO_ROOT/downloads" "$WORKSPACE/src"

check_workspace_files
setup_apt_sources_if_requested
install_apt_deps_if_requested

official_source="$(find_official_source || true)"
if [[ -z "$official_source" ]]; then
  download_archive
  extract_archive
  official_source="$(find_official_source || true)"
fi
[[ -n "$official_source" ]] || die "Could not find a valid official ros2_rm_robot source tree."
log OK "Official source tree: $official_source"

copy_official_source "$official_source"
tree_is_valid "$TARGET_DIR" "${official_required[@]}" || die "Official source verification failed under $TARGET_DIR"
log OK "Official source verification passed."

check_ubuntu_environment
run_project_verify
run_build_if_requested

log OK "Bootstrap complete."
if [[ "$RUN_BUILD" -eq 1 ]]; then
  log INFO "Next command: cd \"$WORKSPACE\" && bash scripts/run_day_visual.sh day01"
else
  log INFO "Next command: cd \"$WORKSPACE\" && bash scripts/bootstrap_vmware_ubuntu.sh --build"
fi
