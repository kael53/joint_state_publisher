#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vector>

using std::placeholders::_1;

class DetectionToGoalNode : public rclcpp::Node {
public:
  DetectionToGoalNode() : Node("detection_to_goal_node") {
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "detections", 10, std::bind(&DetectionToGoalNode::detection_callback, this, _1));
    selection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "detection_selection", 10, std::bind(&DetectionToGoalNode::selection_callback, this, _1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10);
    RCLCPP_INFO(this->get_logger(), "DetectionToGoalNode initialized");
  }

private:
  void selection_callback(const std_msgs::msg::String::SharedPtr msg) {
    selected_id_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Selected detection id: %s", selected_id_.c_str());
    // Publish goal pose only once, using the latest detections
    for (const auto & detection : detections_) {
      if (detection.results.empty()) continue;
      if (detection.results[0].id == selected_id_) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header = detection.header;
        goal_pose.pose = detection.bbox.center;
        goal_pub_->publish(goal_pose);
        RCLCPP_INFO(this->get_logger(), "Published goal pose for id: %s", selected_id_.c_str());
        break;
      }
    }
  }

  void detection_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    detections_ = msg->detections;
  }

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  std::string selected_id_;
  std::vector<vision_msgs::msg::Detection3D> detections_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
