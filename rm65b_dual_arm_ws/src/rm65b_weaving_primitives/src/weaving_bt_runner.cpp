#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

namespace rm65b_weaving_primitives
{
namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using FollowJointTrajectoryClient = rclcpp_action::Client<FollowJointTrajectory>;
using GripperCommandClient = rclcpp_action::Client<GripperCommand>;

struct TrajectoryPointSpec
{
  double time_from_start{0.0};
  std::vector<double> positions;
};

struct WeavingContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub;
  bool dry_run{true};
  std::string playback_stage{"D4_D5"};
  std::map<std::string, std::string> controllers;
  std::map<std::string, std::vector<std::string>> joint_names;
  std::map<std::string, std::map<std::string, std::vector<TrajectoryPointSpec>>> primitives;
  std::map<std::string, FollowJointTrajectoryClient::SharedPtr> arm_clients;
  std::map<std::string, GripperCommandClient::SharedPtr> gripper_clients;
  mutable std::mutex telemetry_mutex;
  bool has_tension{false};
  double latest_tension_n{0.0};
  std::string latest_tension_status{"UNKNOWN"};
  rclcpp::Time latest_tension_stamp;
  std::map<std::string, double> latest_joint_positions;
  rclcpp::Time latest_joint_stamp;
  std::string primitive_lock_owner;

  void publish_event(const std::string & text) const
  {
    RCLCPP_INFO(node->get_logger(), "%s", text.c_str());
    std_msgs::msg::String msg;
    msg.data = text;
    event_pub->publish(msg);
  }
};

using WeavingContextPtr = std::shared_ptr<WeavingContext>;

WeavingContextPtr get_context(const BT::NodeConfiguration & config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError("missing weaving context on BT blackboard");
  }
  WeavingContextPtr context;
  try {
    context = config.blackboard->get<WeavingContextPtr>("context");
  } catch (const std::exception & error) {
    throw BT::RuntimeError(std::string("missing weaving context on BT blackboard: ") + error.what());
  }
  if (!context) {
    throw BT::RuntimeError("empty weaving context on BT blackboard");
  }
  return context;
}

std::string read_text_file(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open " + path);
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string btcpp_v3_xml_path(const std::string & source_path, bool & used_compat_xml)
{
  std::string xml = read_text_file(source_path);
  const std::string original_xml = xml;
  xml = std::regex_replace(xml, std::regex("BTCPP_format=\"4\""), "BTCPP_format=\"3\"");
  xml = std::regex_replace(xml, std::regex("success_count="), "success_threshold=");
  xml = std::regex_replace(xml, std::regex("failure_count="), "failure_threshold=");
  if (xml == original_xml) {
    used_compat_xml = false;
    return source_path;
  }

  const auto compat_path =
    (std::filesystem::temp_directory_path() / "rm65b_weaving_tree.btcpp_v3.xml").string();
  std::ofstream output(compat_path);
  if (!output) {
    throw std::runtime_error("failed to write " + compat_path);
  }
  output << xml;
  used_compat_xml = true;
  return compat_path;
}

std::vector<std::string> string_sequence(const YAML::Node & node)
{
  std::vector<std::string> values;
  if (!node || !node.IsSequence()) {
    return values;
  }
  values.reserve(node.size());
  for (const auto & item : node) {
    values.push_back(item.as<std::string>());
  }
  return values;
}

std::vector<TrajectoryPointSpec> trajectory_points(const YAML::Node & node)
{
  std::vector<TrajectoryPointSpec> points;
  if (!node || !node.IsSequence()) {
    return points;
  }
  points.reserve(node.size());
  for (const auto & point_node : node) {
    TrajectoryPointSpec point;
    point.time_from_start = point_node["time_from_start"].as<double>(0.0);
    const auto positions_node = point_node["positions"];
    if (positions_node && positions_node.IsSequence()) {
      point.positions.reserve(positions_node.size());
      for (const auto & value : positions_node) {
        point.positions.push_back(value.as<double>());
      }
    }
    points.push_back(std::move(point));
  }
  return points;
}

bool load_trajectory_file(const std::string & path, WeavingContext & context)
{
  const YAML::Node root = YAML::LoadFile(path);
  const auto controllers = root["controllers"];
  if (controllers) {
    context.controllers["left"] = controllers["left"].as<std::string>("");
    context.controllers["right"] = controllers["right"].as<std::string>("");
  }

  const auto joint_names = root["joint_names"];
  if (joint_names) {
    context.joint_names["left"] = string_sequence(joint_names["left"]);
    context.joint_names["right"] = string_sequence(joint_names["right"]);
  }

  const auto primitives = root["primitives"];
  if (!primitives || !primitives.IsMap()) {
    return false;
  }

  for (const auto & primitive_entry : primitives) {
    const std::string primitive_name = primitive_entry.first.as<std::string>();
    const auto primitive_node = primitive_entry.second;
    for (const auto & side : {"left", "right"}) {
      if (primitive_node[side]) {
        context.primitives[primitive_name][side] = trajectory_points(primitive_node[side]);
      }
    }
  }
  return true;
}

bool send_trajectory(const WeavingContextPtr & context, const std::string & side,
  const std::vector<TrajectoryPointSpec> & points)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = context->joint_names[side];
  for (const auto & point_spec : points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = point_spec.positions;
    const double seconds = std::max(0.0, point_spec.time_from_start);
    point.time_from_start.sec = static_cast<int32_t>(std::floor(seconds));
    point.time_from_start.nanosec =
      static_cast<uint32_t>((seconds - std::floor(seconds)) * 1000000000.0);
    trajectory.points.push_back(std::move(point));
  }

  if (context->dry_run) {
    context->publish_event(
      "dry_run_trajectory:" + side + ":" + std::to_string(trajectory.points.size()));
    return true;
  }

  const auto client_it = context->arm_clients.find(side);
  if (client_it == context->arm_clients.end() || !client_it->second) {
    context->publish_event("missing_action_client:" + side);
    return false;
  }
  if (!client_it->second->wait_for_action_server(std::chrono::seconds(1))) {
    context->publish_event("missing_action_server:" + side);
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = std::move(trajectory);
  client_it->second->async_send_goal(goal);
  return true;
}

bool command_gripper(
  const WeavingContextPtr & context, const std::string & side, const double position)
{
  if (context->dry_run) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "dry_run_gripper:%s:%.3f", side.c_str(), position);
    context->publish_event(buffer);
    return true;
  }

  const auto client_it = context->gripper_clients.find(side);
  if (client_it == context->gripper_clients.end() || !client_it->second) {
    context->publish_event("missing_gripper_client:" + side);
    return false;
  }
  if (!client_it->second->wait_for_action_server(std::chrono::seconds(1))) {
    context->publish_event("missing_gripper_action:" + side);
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 25.0;
  client_it->second->async_send_goal(goal);
  return true;
}

double primitive_arrival_error(
  const WeavingContextPtr & context, const std::string & primitive)
{
  const auto primitive_it = context->primitives.find(primitive);
  if (primitive_it == context->primitives.end()) {
    return std::numeric_limits<double>::infinity();
  }

  std::lock_guard<std::mutex> guard(context->telemetry_mutex);
  double max_error = 0.0;
  bool compared = false;
  for (const auto & side_points : primitive_it->second) {
    const auto side = side_points.first;
    const auto & points = side_points.second;
    if (points.empty()) {
      continue;
    }
    const auto & target = points.back().positions;
    const auto joint_names_it = context->joint_names.find(side);
    if (joint_names_it == context->joint_names.end()) {
      continue;
    }
    const auto & names = joint_names_it->second;
    for (std::size_t idx = 0; idx < std::min(names.size(), target.size()); ++idx) {
      const auto actual_it = context->latest_joint_positions.find(names[idx]);
      if (actual_it == context->latest_joint_positions.end()) {
        return std::numeric_limits<double>::infinity();
      }
      max_error = std::max(max_error, std::abs(actual_it->second - target[idx]));
      compared = true;
    }
  }
  return compared ? max_error : std::numeric_limits<double>::infinity();
}

class WaitForVisionTarget : public BT::SyncActionNode
{
public:
  WaitForVisionTarget(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target_topic")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string target_topic{"/vision/target_pose"};
    getInput("target_topic", target_topic);
    context->publish_event("bt_action_start:WaitForVisionTarget");
    context->publish_event("vision_wait_topic:" + target_topic);
    context->publish_event("bt_action_success:WaitForVisionTarget");
    return BT::NodeStatus::SUCCESS;
  }
};

class CloseGripper : public BT::SyncActionNode
{
public:
  CloseGripper(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("controller")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string controller;
    getInput("controller", controller);
    const std::string side = controller.find("left") != std::string::npos ? "left" : "right";
    context->publish_event("bt_action_start:CloseGripper");
    const bool ok = command_gripper(context, side, 0.004);
    context->publish_event(std::string("bt_action_") + (ok ? "success" : "failure") +
      ":CloseGripper");
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("controller")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string controller;
    getInput("controller", controller);
    const std::string side = controller.find("left") != std::string::npos ? "left" : "right";
    context->publish_event("bt_action_start:OpenGripper");
    const bool ok = command_gripper(context, side, 0.045);
    context->publish_event(std::string("bt_action_") + (ok ? "success" : "failure") +
      ":OpenGripper");
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class RunPrimitive : public BT::SyncActionNode
{
public:
  RunPrimitive(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("primitive")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string primitive;
    getInput("primitive", primitive);
    context->publish_event("bt_action_start:RunPrimitive");

    const auto primitive_it = context->primitives.find(primitive);
    if (primitive_it == context->primitives.end()) {
      context->publish_event("missing_primitive:" + primitive);
      context->publish_event("bt_action_failure:RunPrimitive");
      return BT::NodeStatus::FAILURE;
    }

    context->publish_event("primitive_start:" + primitive);
    bool ok = true;
    for (const auto & side_points : primitive_it->second) {
      ok = send_trajectory(context, side_points.first, side_points.second) && ok;
    }
    if (ok) {
      context->publish_event("primitive_sent:" + primitive);
    }
    context->publish_event(std::string("bt_action_") + (ok ? "success" : "failure") +
      ":RunPrimitive");
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CheckInterlock : public BT::SyncActionNode
{
public:
  CheckInterlock(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("condition")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string condition{"arms_ready"};
    getInput("condition", condition);
    context->publish_event("bt_action_start:CheckInterlock");
    {
      std::lock_guard<std::mutex> guard(context->telemetry_mutex);
      if (!context->primitive_lock_owner.empty()) {
        context->publish_event("interlock_blocked:primitive_lock_owner:" +
          context->primitive_lock_owner);
        context->publish_event("bt_action_failure:CheckInterlock");
        return BT::NodeStatus::FAILURE;
      }
    }
    context->publish_event("interlock_ok:" + condition);
    context->publish_event("bt_action_success:CheckInterlock");
    return BT::NodeStatus::SUCCESS;
  }
};

class AcquirePrimitiveLock : public BT::SyncActionNode
{
public:
  AcquirePrimitiveLock(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("owner")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string owner{"d4_weaving_cycle"};
    getInput("owner", owner);
    context->publish_event("bt_action_start:AcquirePrimitiveLock");
    {
      std::lock_guard<std::mutex> guard(context->telemetry_mutex);
      if (!context->primitive_lock_owner.empty() && context->primitive_lock_owner != owner) {
        context->publish_event("primitive_lock_busy:" + context->primitive_lock_owner);
        context->publish_event("bt_action_failure:AcquirePrimitiveLock");
        return BT::NodeStatus::FAILURE;
      }
      context->primitive_lock_owner = owner;
    }
    context->publish_event("primitive_lock_acquired:" + owner);
    context->publish_event("bt_action_success:AcquirePrimitiveLock");
    return BT::NodeStatus::SUCCESS;
  }
};

class ReleasePrimitiveLock : public BT::SyncActionNode
{
public:
  ReleasePrimitiveLock(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("owner")};
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string owner{"d4_weaving_cycle"};
    getInput("owner", owner);
    context->publish_event("bt_action_start:ReleasePrimitiveLock");
    {
      std::lock_guard<std::mutex> guard(context->telemetry_mutex);
      if (context->primitive_lock_owner == owner) {
        context->primitive_lock_owner.clear();
      }
    }
    context->publish_event("primitive_lock_released:" + owner);
    context->publish_event("bt_action_success:ReleasePrimitiveLock");
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitForArrival : public BT::SyncActionNode
{
public:
  WaitForArrival(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("primitive"),
      BT::InputPort<double>("tolerance_rad"),
      BT::InputPort<double>("timeout_s"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    std::string primitive;
    double tolerance_rad = 0.08;
    double timeout_s = 6.0;
    getInput("primitive", primitive);
    getInput("tolerance_rad", tolerance_rad);
    getInput("timeout_s", timeout_s);
    context->publish_event("bt_action_start:WaitForArrival");

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      const double error = primitive_arrival_error(context, primitive);
      if (std::isfinite(error)) {
        char buffer[160];
        std::snprintf(
          buffer, sizeof(buffer), "arrival_check:%s:max_error_rad=%.5f:tolerance_rad=%.5f",
          primitive.c_str(), error, tolerance_rad);
        context->publish_event(buffer);
        if (error <= tolerance_rad) {
          context->publish_event("arrival_confirmed:" + primitive);
          context->publish_event("bt_action_success:WaitForArrival");
          return BT::NodeStatus::SUCCESS;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (context->dry_run) {
      context->publish_event("arrival_confirmed_by_dry_run_evidence:" + primitive);
      context->publish_event("bt_action_success:WaitForArrival");
      return BT::NodeStatus::SUCCESS;
    }
    context->publish_event("arrival_timeout:" + primitive);
    context->publish_event("bt_action_failure:WaitForArrival");
    return BT::NodeStatus::FAILURE;
  }
};

class WaitForTension : public BT::SyncActionNode
{
public:
  WaitForTension(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_n"),
      BT::InputPort<double>("max_n"),
      BT::InputPort<double>("timeout_s"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto context = get_context(config());
    double min_n = 0.0;
    double max_n = 0.0;
    double timeout_s = 8.0;
    getInput("min_n", min_n);
    getInput("max_n", max_n);
    getInput("timeout_s", timeout_s);
    context->publish_event("bt_action_start:WaitForTension");
    context->publish_event(
      "tension_window:" + std::to_string(min_n) + ":" + std::to_string(max_n));

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      double tension = 0.0;
      std::string status;
      bool has_tension = false;
      {
        std::lock_guard<std::mutex> guard(context->telemetry_mutex);
        has_tension = context->has_tension;
        tension = context->latest_tension_n;
        status = context->latest_tension_status;
      }
      if (has_tension) {
        char buffer[160];
        std::snprintf(
          buffer, sizeof(buffer), "tension_check:value_n=%.3f:status=%s:window=%.3f..%.3f",
          tension, status.c_str(), min_n, max_n);
        context->publish_event(buffer);
        if (tension >= min_n && tension <= max_n && status != "HIGH" && status != "LOW") {
          context->publish_event("tension_confirmed");
          context->publish_event("bt_action_success:WaitForTension");
          return BT::NodeStatus::SUCCESS;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (context->dry_run) {
      context->publish_event("tension_confirmed_by_dry_run_evidence");
      context->publish_event("bt_action_success:WaitForTension");
      return BT::NodeStatus::SUCCESS;
    }
    context->publish_event("tension_timeout");
    context->publish_event("bt_action_failure:WaitForTension");
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace

class WeavingBtRunner : public rclcpp::Node
{
public:
  WeavingBtRunner()
  : rclcpp::Node("weaving_bt_runner")
  {
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("rm65b_weaving_primitives");
    declare_parameter("behavior_tree_file", share_dir + "/config/weaving_tree.xml");
    declare_parameter("trajectory_file", share_dir + "/trajectories/weaving_primitives.yaml");
    declare_parameter("dry_run", true);
    declare_parameter("bt_runtime_name", "cpp_behavior_tree_cpp_v3");
    declare_parameter("playback_stage", "D4_D5");
    declare_parameter("left_gripper_action", "/left_gripper_controller/gripper_cmd");
    declare_parameter("right_gripper_action", "/right_gripper_controller/gripper_cmd");
  }

  void configure(const std::shared_ptr<WeavingBtRunner> & self)
  {
    context_ = std::make_shared<WeavingContext>();
    context_->node = self;
    context_->event_pub = create_publisher<std_msgs::msg::String>("/weaving/events", 10);
    context_->dry_run = get_parameter("dry_run").as_bool();
    context_->playback_stage = get_parameter("playback_stage").as_string();
    context_->latest_tension_stamp = now();
    context_->latest_joint_stamp = now();

    tension_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/weaving/tension_n", 10,
      [context = context_](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(context->telemetry_mutex);
        context->latest_tension_n = static_cast<double>(msg->data);
        context->has_tension = true;
        context->latest_tension_stamp = context->node->now();
      });
    tension_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/weaving/tension_status", 10,
      [context = context_](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(context->telemetry_mutex);
        context->latest_tension_status = msg->data;
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      [context = context_](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(context->telemetry_mutex);
        for (std::size_t idx = 0; idx < std::min(msg->name.size(), msg->position.size()); ++idx) {
          context->latest_joint_positions[msg->name[idx]] = msg->position[idx];
        }
        context->latest_joint_stamp = context->node->now();
      });

    const auto trajectory_file = get_parameter("trajectory_file").as_string();
    if (!load_trajectory_file(trajectory_file, *context_)) {
      throw std::runtime_error("failed to load primitives from " + trajectory_file);
    }

    if (!context_->dry_run) {
      context_->arm_clients["left"] =
        rclcpp_action::create_client<FollowJointTrajectory>(self, context_->controllers["left"]);
      context_->arm_clients["right"] =
        rclcpp_action::create_client<FollowJointTrajectory>(self, context_->controllers["right"]);
      context_->gripper_clients["left"] = rclcpp_action::create_client<GripperCommand>(
        self, get_parameter("left_gripper_action").as_string());
      context_->gripper_clients["right"] = rclcpp_action::create_client<GripperCommand>(
        self, get_parameter("right_gripper_action").as_string());
    }

    factory_.registerNodeType<WaitForVisionTarget>("WaitForVisionTarget");
    factory_.registerNodeType<CloseGripper>("CloseGripper");
    factory_.registerNodeType<OpenGripper>("OpenGripper");
    factory_.registerNodeType<RunPrimitive>("RunPrimitive");
    factory_.registerNodeType<CheckInterlock>("CheckInterlock");
    factory_.registerNodeType<AcquirePrimitiveLock>("AcquirePrimitiveLock");
    factory_.registerNodeType<ReleasePrimitiveLock>("ReleasePrimitiveLock");
    factory_.registerNodeType<WaitForArrival>("WaitForArrival");
    factory_.registerNodeType<WaitForTension>("WaitForTension");

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", context_);
    behavior_tree_file_ = get_parameter("behavior_tree_file").as_string();
    const auto btcpp_v3_file = btcpp_v3_xml_path(behavior_tree_file_, used_compat_xml_);
    try {
      tree_ =
        std::make_unique<BT::Tree>(factory_.createTreeFromFile(btcpp_v3_file, blackboard_));
    } catch (const std::exception & first_error) {
      throw std::runtime_error(
        "BehaviorTree.CPP v3 failed to load " + btcpp_v3_file + ": " + first_error.what());
    }

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { start_once(); });
    RCLCPP_INFO(
      get_logger(), "BehaviorTree.CPP runner ready, dry_run=%s, tree=%s",
      context_->dry_run ? "true" : "false", behavior_tree_file_.c_str());
  }

private:
  void start_once()
  {
    if (started_) {
      return;
    }
    started_ = true;
    timer_->cancel();

    context_->publish_event("bt_runtime:" + get_parameter("bt_runtime_name").as_string());
    context_->publish_event("playback_stage:" + context_->playback_stage);
    context_->publish_event("bt_loaded:" + behavior_tree_file_);
    if (used_compat_xml_) {
      context_->publish_event("bt_xml_compat:btcpp_v4_to_v3");
    }
    context_->publish_event("sequence_start");

    const auto status = tree_->tickRoot();
    const bool ok = status == BT::NodeStatus::SUCCESS;
    context_->publish_event(std::string("bt_tree_complete:") + (ok ? "success" : "failure"));
    context_->publish_event("sequence_complete");
  }

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
  WeavingContextPtr context_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tension_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tension_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::string behavior_tree_file_;
  bool used_compat_xml_{false};
  bool started_{false};
};

}  // namespace rm65b_weaving_primitives

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm65b_weaving_primitives::WeavingBtRunner>();
  int exit_code = 0;
  try {
    node->configure(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
