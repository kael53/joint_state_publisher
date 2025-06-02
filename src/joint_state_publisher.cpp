// ROS2 Joint State Publisher Node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/hand_state.hpp>

#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <future>

#include <g1_dex3_joint_defs.hpp>

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher()
  : Node("joint_state_publisher") {
    RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node Initialized");

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

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF from robot_description parameter");
      rclcpp::shutdown();
      return;
    }
    for (const auto& joint : model.joints_) {
      if (joint.second->type == urdf::Joint::REVOLUTE ||
          joint.second->type == urdf::Joint::PRISMATIC ||
          joint.second->type == urdf::Joint::CONTINUOUS) {
        js_names_.push_back(joint.first);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu joints from robot_description", js_names_.size());
    // Dynamically size state vectors based on joint maps
    auto hand_joint_count = hand_joint_name_to_index.size() / 2; // 2 hands
    lowstate_joints_.resize(joint_name_to_index.size(), unitree_hg::msg::MotorState());
    left_hand_joints_.resize(hand_joint_count, unitree_hg::msg::MotorState());
    right_hand_joints_.resize(hand_joint_count, unitree_hg::msg::MotorState());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "/lf/lowstate", 10,
      std::bind(&JointStatePublisher::lowstate_callback, this, std::placeholders::_1));
    left_state_sub_ = this->create_subscription<unitree_hg::msg::HandState>(
      "/lf/dex3/left/state", 10,
      std::bind(&JointStatePublisher::left_state_callback, this, std::placeholders::_1));
    right_state_sub_ = this->create_subscription<unitree_hg::msg::HandState>(
      "/lf/dex3/right/state", 10,
      std::bind(&JointStatePublisher::right_state_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&JointStatePublisher::publish_joint_states, this));
  }

private:
  std::vector<std::string> js_names_;
  // Store latest joint values
  std::vector<unitree_hg::msg::MotorState> lowstate_joints_;
  std::vector<unitree_hg::msg::MotorState> left_hand_joints_;
  std::vector<unitree_hg::msg::MotorState> right_hand_joints_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr left_state_sub_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr right_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg) {
    lowstate_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }
  void left_state_callback(const unitree_hg::msg::HandState::SharedPtr msg) {
    left_hand_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }
  void right_state_callback(const unitree_hg::msg::HandState::SharedPtr msg) {
    right_hand_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }

  void publish_joint_states() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = js_names_;
    js.position.resize(js.name.size(), 0.0);

    // Use the provided name-index maps for assignment
    for (const auto& pair : joint_name_to_index) {
      auto idx = pair.second;
      auto it = std::find(js.name.begin(), js.name.end(), pair.first);
      if (it != js.name.end() && idx < lowstate_joints_.size()) {
        size_t js_idx = std::distance(js.name.begin(), it);
        js.position[js_idx] = lowstate_joints_[idx].q;
      }
    }
    for (const auto& pair : hand_joint_name_to_index) {
      auto idx = pair.second;
      auto it = std::find(js.name.begin(), js.name.end(), pair.first);
      if (it != js.name.end()) {
        size_t js_idx = std::distance(js.name.begin(), it);
        if (pair.first.find("left_") == 0) {
          js.position[js_idx] = left_hand_joints_[idx].q;
        } else if (pair.first.find("right_") == 0) {
          js.position[js_idx] = right_hand_joints_[idx].q;
        }
      }
    }
    joint_state_pub_->publish(js);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
