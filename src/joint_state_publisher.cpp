// ROS2 Joint State Publisher Node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/hand_state.hpp>

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher()
  : Node("joint_state_publisher"),
      lowstate_joints_(35, unitree_hg::msg::MotorState()),
      left_joints_(7, unitree_hg::msg::MotorState()),
      right_joints_(7, unitree_hg::msg::MotorState()) {
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
  // Store latest joint values
  std::vector<unitree_hg::msg::MotorState> lowstate_joints_;
  std::vector<unitree_hg::msg::MotorState> left_joints_;
  std::vector<unitree_hg::msg::MotorState> right_joints_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr left_state_sub_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr right_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg) {
    lowstate_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }
  void left_state_callback(const unitree_hg::msg::HandState::SharedPtr msg) {
    left_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }
  void right_state_callback(const unitree_hg::msg::HandState::SharedPtr msg) {
    right_joints_.assign(msg->motor_state.begin(), msg->motor_state.end());
  }

  void publish_joint_states() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {
      "left_hip_pitch_joint",
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_knee_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      "right_hip_pitch_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_knee_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
      "waist_yaw_joint",
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_pitch_joint",
      "left_wrist_yaw_joint",
      "left_hand_thumb_0_joint",
      "left_hand_thumb_1_joint",
      "left_hand_thumb_2_joint",
      "left_hand_middle_0_joint",
      "left_hand_middle_1_joint",
      "left_hand_index_0_joint",
      "left_hand_index_1_joint",
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_pitch_joint",
      "right_wrist_yaw_joint",
      "right_hand_thumb_0_joint",
      "right_hand_thumb_1_joint",
      "right_hand_thumb_2_joint",
      "right_hand_middle_0_joint",
      "right_hand_middle_1_joint",
      "right_hand_index_0_joint",
      "right_hand_index_1_joint"
    };

    js.position.resize(js.name.size(), 0.0);
    js.velocity.resize(js.name.size(), 0.0);
    js.effort.resize(js.name.size(), 0.0);

    for (size_t i = 0; i < 29; ++i) {
      if (i == 13 || i == 14) continue; // fixed waist roll, pitch joints

      size_t target_index = i;

      if (i > 14 && i < 22) { target_index = i - 2; }
      else if (i > 21) { target_index = i + 5; }

      js.position[target_index] = lowstate_joints_[i].q;
      //js.velocity[target_index] = lowstate_joints_[i].dq;
      //js.effort[target_index] = lowstate_joints_[i].tau_est;
    }

    for (size_t i = 0; i < 7; ++i) {
        size_t target_index = 20 + i;
        js.position[target_index] = left_joints_[i].q;
        //js.velocity[target_index] = left_joints_[i].dq;
        //js.effort[target_index] = left_joints_[i].tau_est;

        target_index = 27 + i;
        js.position[target_index] = right_joints_[i].q;
        //js.velocity[target_index] = right_joints_[i].dq;
        //js.effort[target_index] = right_joints_[i].tau_est;
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
