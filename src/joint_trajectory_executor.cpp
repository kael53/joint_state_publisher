#include "rclcpp/rclcpp.hpp"

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/motor_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <chrono>
#include <map>
#include <string>
#include <urdf/model.h>

#include <g1_dex3_joint_defs.hpp>

using namespace std::chrono_literals;

// Struct to hold joint limits
struct JointLimits {
  double lower;
  double upper;
  double velocity;
  double effort;
};

class JointTrajectoryExecutor : public rclcpp::Node {
public:
  JointTrajectoryExecutor()
  : Node("joint_trajectory_executor")
  {
    RCLCPP_INFO(this->get_logger(), "Joint Trajectory Executor Node Initialized");

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", qos_profile);
    left_hand_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dex3/left/command", 10);
    right_hand_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dex3/right/command", 10);

    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_targets", 10,
      std::bind(&JointTrajectoryExecutor::trajectoryCallback, this, std::placeholders::_1));

    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "/lf/lowstate", 10,
      std::bind(&JointTrajectoryExecutor::lowstateCallback, this, std::placeholders::_1));

    // Load URDF and parse joint limits
    std::string urdf_xml;
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /robot_state_publisher service...");
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->values.size() == 1 && response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
        urdf_xml = response->values[0].string_value;
      } else {
        RCLCPP_FATAL(this->get_logger(), "robot_description not found in /robot_state_publisher");
        rclcpp::shutdown();
        return;
      }
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to /robot_state_publisher/get_parameters service");
      rclcpp::shutdown();
      return;
    }

    if (!urdf_xml.empty()) {
      urdf::Model urdf_model;
      if (urdf_model.initString(urdf_xml)) {
        for (const auto& joint_pair : urdf_model.joints_) {
          const auto& joint = joint_pair.second;
          if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) continue;
          if (!joint->limits) continue;
          JointLimits lim;
          lim.lower = joint->limits->lower;
          lim.upper = joint->limits->upper;
          lim.velocity = joint->limits->velocity;
          lim.effort = joint->limits->effort;
          joint_limits_[joint->name] = lim;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded joint limits from URDF");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to parse URDF for joint limits");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "robot_description parameter is empty, joint limits not loaded");
    }
  }

  ~JointTrajectoryExecutor() override {
    RCLCPP_INFO(this->get_logger(), "Shutting down Joint Trajectory Executor Node");

    // Final command to stop arm control
    unitree_hg::msg::LowCmd final_cmd;
    final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 0.0f;
    cmd_pub_->publish(final_cmd);
  }

private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_hand_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_hand_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  std::map<std::string, JointLimits> joint_limits_;
  std::vector<float> latest_joint_positions_;

  void lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    // Store latest positions for all arm joints
    latest_joint_positions_.resize(joint_name_to_index.size(), 0.0f);
    for (const auto& pair : joint_name_to_index) {
      size_t idx = pair.second;
      if (idx < msg->motor_state.size()) {
        latest_joint_positions_[idx] = msg->motor_state[idx].q;
      }
    }
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // Find out which hand to use based on the joint names
    if (msg->joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty joint names in trajectory message");
      return;
    }
    auto hand_it = std::find_if(msg->joint_names.begin(), msg->joint_names.end(),
      [](const std::string& name) {
        return name.find("left") != std::string::npos || name.find("right") != std::string::npos;
      });
    if (hand_it == msg->joint_names.end()) {
      RCLCPP_ERROR(this->get_logger(), "No side found in trajectory message");
      return;
    }
    bool is_left_hand = hand_it->find("left") != std::string::npos;
    RCLCPP_INFO(this->get_logger(), "Executing trajectory for %s hand", is_left_hand ? "left" : "right");
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hand_cmd_pub = is_left_hand ? left_hand_pub_ : right_hand_pub_;
    
    // Start with preparing the hand, open it fully
    std_msgs::msg::Bool hand_cmd;
    hand_cmd.data = false; // Open hand command
    hand_cmd_pub->publish(hand_cmd); // Open hand command
    // Wait for the hand to open
    rclcpp::sleep_for(1s);  // Wait for the hand to open
    RCLCPP_INFO(this->get_logger(), "Hand opened fully, starting trajectory execution");

    auto start_time = this->now();

    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto& point = msg->points[i];
      unitree_hg::msg::LowCmd cmd_msg;

      cmd_msg.motor_cmd[JointIndex::kNotUsedJoint].q = 0.5f; // Full transition speed for trajectory following
      // Fill all joints with latest state first
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        if (latest_joint_positions_.size() > idx) {
          cmd_msg.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          cmd_msg.motor_cmd[idx].q = 0.0f;
        }
        cmd_msg.motor_cmd[idx].dq = 0.f;
        cmd_msg.motor_cmd[idx].kp = 60.0f;
        cmd_msg.motor_cmd[idx].kd = 1.5f;
        cmd_msg.motor_cmd[idx].tau = 0.f;
      }
      // Overwrite with trajectory values for joints present in this point
      for (size_t j = 0; j < point.positions.size(); ++j) {
        auto target_joint_name = msg->joint_names[j];
        auto target_index = joint_name_to_index.at(target_joint_name);
        auto target_position = point.positions[j];
        auto lim = joint_limits_.at(target_joint_name);
        target_position = std::min(std::max(target_position, lim.lower), lim.upper);
        cmd_msg.motor_cmd[target_index].q = target_position;
      }

      // Wait until the scheduled time for this point
      rclcpp::Time scheduled_time = start_time + point.time_from_start;
      rclcpp::Duration wait_time = scheduled_time - this->now();
      if (wait_time > rclcpp::Duration::from_seconds(0)) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
      }

      cmd_pub_->publish(cmd_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory point %zu executed, sleeping for %d seconds", msg->points.size(), 2);
    rclcpp::sleep_for(1s);  // Wait for the last command to take effect

    // After executing the trajectory, close the hand
    hand_cmd.data = true; // Close hand command
    hand_cmd_pub->publish(hand_cmd); // Close hand command
    // Wait for the hand to close
    rclcpp::sleep_for(1s);  // Wait for the hand to close
    RCLCPP_INFO(this->get_logger(), "Hand closed after trajectory execution");

    RCLCPP_INFO(this->get_logger(), "Trajectory execution complete, returning to default pose.");

    // Smoothly interpolate final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q from 1.0 to 0.0
    const double interp_duration = 1.0; // seconds
    const int interp_steps = 50;
    auto sleep_ns = std::chrono::nanoseconds(static_cast<int64_t>((interp_duration / interp_steps) * 1e9));
    for (int step = 0; step <= interp_steps; ++step) {
      double t = static_cast<double>(step) / interp_steps;
      double value = (1.0 - t) * 1.0 + t * 0.0; // Linear interpolation
      unitree_hg::msg::LowCmd final_cmd;
      final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = static_cast<float>(value);
      cmd_pub_->publish(final_cmd);
      rclcpp::sleep_for(sleep_ns);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
