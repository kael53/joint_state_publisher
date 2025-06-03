#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <unitree_hg/msg/hand_cmd.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>

#include <unitree_hg/msg/hand_state.hpp>
#include <unitree_hg/msg/motor_state.hpp>
#include <unitree_hg/msg/press_sensor_state.hpp>

#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <chrono>
#include <map>
#include <string>
#include <urdf/model.h>

#include <g1_dex3_joint_defs.hpp>

using namespace std::chrono_literals;

typedef struct {
    uint8_t id     : 4;   // Motor ID: 0,1...,13,14  15 represents broadcasting to all motors
    uint8_t status : 3;   // Working mode: 0.Lock 1.FOC 6
    uint8_t timeout: 1;   // Master->Motor: 0.Disable timeout protection 1.Enable (Default 1s timeout)
                          // Motor->Master: 0.No timeout 1.Timeout protection triggered (needs control bit 0 to clear)
} RIS_Mode_t;             // Control mode 1Byte

// Struct to hold joint limits
struct JointLimits {
  double lower;
  double upper;
  double velocity;
  double effort;
};

class Dex3Controller : public rclcpp::Node {
public:
  Dex3Controller() : Node("dex3_controller") {
    // Parameterize side, then input/output topics (defaulting to side-based names)
    this->declare_parameter("side", "left");
    std::string side;
    this->get_parameter("side", side);
    std::string default_input = "/dex3/" + side + "/command";
    std::string output_topic = "/dex3/" + side + "/cmd"; // Fixed output topic
    this->declare_parameter("input_topic", default_input);
    std::string input_topic;
    this->get_parameter("input_topic", input_topic);

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    hand_cmd_pub_ = this->create_publisher<unitree_hg::msg::HandCmd>(output_topic, qos_profile);
    // Subscribe to high-level commands (e.g., "open hand", "close hand")
    hand_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      input_topic, 10,
      std::bind(&Dex3Controller::handCmdCallback, this, std::placeholders::_1));

    // Subscribe to feedback from the hand
    std::string state_topic = "/dex3/" + side + "/state";
    hand_state_sub_ = this->create_subscription<unitree_hg::msg::HandState>(
      state_topic, 10,
      std::bind(&Dex3Controller::handStateCallback, this, std::placeholders::_1));

    // Load URDF from /robot_description parameter and parse joint limits
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

    if (urdf_xml.empty()) {
      RCLCPP_ERROR(this->get_logger(), "robot_description parameter is missing or empty. Cannot continue.");
      rclcpp::shutdown();
      return;
    }
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot_description parameter");
      rclcpp::shutdown();
      return;
    }
    // Extract hand joints for the specified side (using <side>_hand_palm_link as the base link)
    std::string hand_base_link = side + "_hand_palm_link";
    std::set<std::string> hand_joints_set;
    std::function<void(const std::string&)> collect_hand_joints;
    collect_hand_joints = [&](const std::string& link_name) {
      for (const auto& joint_pair : model.joints_) {
        auto joint = joint_pair.second;
        if (joint && joint->parent_link_name == link_name) {
          hand_joints_set.insert(joint->name);
          // Recursively collect child joints
          if (!joint->child_link_name.empty()) {
            collect_hand_joints(joint->child_link_name);
          }
        }
      }
    };
    collect_hand_joints(hand_base_link);
    hand_joint_names.clear();
    joint_limits_.clear();
    for (const auto& joint_name : hand_joints_set) {
      hand_joint_names.push_back(joint_name);
      auto joint = model.getJoint(joint_name);
      if (joint && joint->limits) {
        joint_limits_[joint_name] = {joint->limits->lower, joint->limits->upper, joint->limits->velocity, joint->limits->effort};
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu hand joints for side '%s' (base link: %s) from robot_description", hand_joint_names.size(), side.c_str(), hand_base_link.c_str());

    RCLCPP_INFO(this->get_logger(), "Dex3Controller started. Subscribing to %s, publishing to %s, feedback from %s", input_topic.c_str(), output_topic.c_str(), state_topic.c_str());

    // Timer for periodic closed-loop grasping (1 Hz)
    closed_loop_timer_ = this->create_wall_timer(
      1000ms, std::bind(&Dex3Controller::closedLoopGrasping, this));
  }
private:
  void handCmdCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received open hand command");
      closing_ = false; // Interrupt any closing loop

      unitree_hg::msg::HandCmd hand_cmd;
      hand_cmd.motor_cmd.resize(hand_joint_names.size()); // Resize motor_cmd vector
      for (size_t i = 0; i < hand_joint_names.size(); ++i) {
        auto joint_name = hand_joint_names[i];

        RIS_Mode_t ris_mode;
        ris_mode.id = i; // Set id
        ris_mode.status = 0x01;  // Set status to 0x01 (FOC mode)
        ris_mode.timeout = 0x00; // Set timeout to 0x00 (no timeout)
    
        uint8_t mode = 0;
        mode |= (ris_mode.id & 0x0F); // Get lower 4 bits of id
        mode |= (ris_mode.status & 0x07) << 4; // Get upper 3 bits of status and shift left 4 bits
        mode |= (ris_mode.timeout & 0x01) << 7; // Get upper 1 bit of timeout and shift left 7 bits

        hand_cmd.motor_cmd[i].mode = mode; // Set the mode for the hand joint

        float target_position = 0.0f; // Default target position for opening the hand

        // Clamp using URDF joint limits if available
        auto lim_it = joint_limits_.find(joint_name);
        if (lim_it != joint_limits_.end()) {
          const auto& lim = lim_it->second;
          if (joint_name.find("thumb_0") != std::string::npos) {
            // Set thumb_0 to the middle of its range
            target_position = 0.5f * (lim.lower + lim.upper);
          } else {
            target_position = lim.lower; // Use lower limit for opening the hand
          }
        }

        hand_cmd.motor_cmd[i].q = target_position; // Open the hand fully (or thumb_0 to middle)
        hand_cmd.motor_cmd[i].dq = 0.f; // No velocity command for opening
        hand_cmd.motor_cmd[i].kp = 1.5f;
        hand_cmd.motor_cmd[i].kd = 0.1f;
        hand_cmd.motor_cmd[i].tau = 0.f;

        RCLCPP_INFO(this->get_logger(), "Setting hand joint %s to position %f", joint_name.c_str(), target_position);
      }

      // Publish the hand command to open it fully
      hand_cmd_pub_->publish(hand_cmd);
      rclcpp::sleep_for(1s);  // Wait for the last command to take effect
    } else {
      RCLCPP_INFO(this->get_logger(), "Received close hand command");
      if (!closing_) {
        closing_ = true;
        // No need to start a timer, main loop handles closing
      }
    }
  }

  void handStateCallback(const unitree_hg::msg::HandState::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received hand state feedback");
    // Aggregate tactile sensor values for thumb, index, middle, and palm, using only valid values and scaling
    float thumb_sum = 0.0f, index_sum = 0.0f, middle_sum = 0.0f, palm_sum = 0.0f;
    size_t thumb_count = 0, index_count = 0, middle_count = 0, palm_count = 0;
    msg->press_sensor_state.resize(9); // Ensure we have 9 sensors as expected
    //RCLCPP_INFO(this->get_logger(), "Processing tactile sensor data for hand state feedback size %zu", msg->press_sensor_state.size());
    for (const auto& press : msg->press_sensor_state) {
      // Thumb: indices 0, 1
      for (size_t idx : {0, 1}) {
        if (idx < press.pressure.size() && press.pressure[idx] != 30000) {
          thumb_sum += press.pressure[idx] / 10000.0f;
          ++thumb_count;
        }
      }
      // Index: indices 4, 5
      for (size_t idx : {4, 5}) {
        if (idx < press.pressure.size() && press.pressure[idx] != 30000) {
          index_sum += press.pressure[idx] / 10000.0f;
          ++index_count;
        }
      }
      // Middle: indices 2, 3
      for (size_t idx : {2, 3}) {
        if (idx < press.pressure.size() && press.pressure[idx] != 30000) {
          middle_sum += press.pressure[idx] / 10000.0f;
          ++middle_count;
        }
      }
      // Palm: indices 6, 7, 8
      for (size_t idx : {6, 7, 8}) {
        if (idx < press.pressure.size() && press.pressure[idx] != 30000) {
          palm_sum += press.pressure[idx] / 10000.0f;
          ++palm_count;
        }
      }
    }
    float thumb_avg = thumb_count > 0 ? thumb_sum / thumb_count : 0.0f;
    float finger_palm_sum = 0.0f;
    int finger_palm_count = 0;
    if (index_count > 0) { finger_palm_sum += index_sum; finger_palm_count += index_count; }
    if (middle_count > 0) { finger_palm_sum += middle_sum; finger_palm_count += middle_count; }
    if (palm_count > 0) { finger_palm_sum += palm_sum; finger_palm_count += palm_count; }
    float finger_palm_avg = finger_palm_count > 0 ? finger_palm_sum / finger_palm_count : 0.0f;
    thumb_tactile_ = thumb_avg;
    finger_tactile_ = finger_palm_avg;
    RCLCPP_DEBUG(this->get_logger(), "Tactile thumb avg: %f, finger/palm avg: %f", thumb_avg, finger_palm_avg);
  }

  void closedLoopGrasping() {
    static std::vector<float> open_positions, closed_positions, interp_positions;
    static bool initialized = false;
    if (closing_) {
      // Initialize joint positions if not done
      if (!initialized) {
        size_t n = hand_joint_names.size();
        open_positions.resize(n, 0.0f);
        closed_positions.resize(n, 0.0f);
        interp_positions.resize(n, 0.0f);
        for (size_t i = 0; i < n; ++i) {
          const auto& joint_name = hand_joint_names[i];
          auto lim_it = joint_limits_.find(joint_name);
          if (lim_it != joint_limits_.end()) {
            const auto& lim = lim_it->second;
            if (joint_name.find("thumb_0") != std::string::npos) {
              float mid = 0.5f * (lim.lower + lim.upper);
              open_positions[i] = mid;
              closed_positions[i] = mid;
              interp_positions[i] = mid;
            } else {
              open_positions[i] = lim.lower;
              closed_positions[i] = lim.upper;
              interp_positions[i] = lim.lower;
            }
          }
        }
        initialized = true;
      }
      // Feedback-driven grasp maintenance
      const float step_fraction = 0.05f;
      const double tactile_threshold = 0.5;
      double thumb_val = thumb_tactile_;
      double finger_val = finger_tactile_;
      bool need_regrip = !(thumb_val > tactile_threshold && finger_val > tactile_threshold);
      if (need_regrip) {
        bool all_reached = true;
        unitree_hg::msg::HandCmd interp_cmd;
        interp_cmd.motor_cmd.resize(hand_joint_names.size());
        for (size_t i = 0; i < hand_joint_names.size(); ++i) {
          const auto& joint_name = hand_joint_names[i];
          RIS_Mode_t ris_mode;
          ris_mode.id = i;
          ris_mode.status = 0x01;
          ris_mode.timeout = 0x00;
          uint8_t mode = 0;
          mode |= (ris_mode.id & 0x0F);
          mode |= (ris_mode.status & 0x07) << 4;
          mode |= (ris_mode.timeout & 0x01) << 7;
          interp_cmd.motor_cmd[i].mode = mode;
          float target = closed_positions[i];
          if (joint_name.find("thumb_0") != std::string::npos) {
            interp_cmd.motor_cmd[i].q = target;
          } else {
            float diff = closed_positions[i] - interp_positions[i];
            float step = step_fraction * (closed_positions[i] - open_positions[i]);
            if (std::abs(diff) > std::abs(step)) {
              interp_positions[i] += step;
              all_reached = false;
            } else {
              interp_positions[i] = closed_positions[i];
            }
            interp_cmd.motor_cmd[i].q = interp_positions[i];
          }
          interp_cmd.motor_cmd[i].dq = 0.0f;
          interp_cmd.motor_cmd[i].kp = 1.5f;
          interp_cmd.motor_cmd[i].kd = 0.1f;
          interp_cmd.motor_cmd[i].tau = 0.0f;
        }
        hand_cmd_pub_->publish(interp_cmd);
      }
    }
  }

  std::vector<std::string> hand_joint_names;
  bool closing_ = false;
  rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr hand_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hand_cmd_sub_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr hand_state_sub_;
  std::map<std::string, JointLimits> joint_limits_;

  double thumb_tactile_ = 0.0;
  double finger_tactile_ = 0.0;

  rclcpp::TimerBase::SharedPtr closed_loop_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dex3Controller>());
  rclcpp::shutdown();
  return 0;
}

