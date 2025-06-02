#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <memory>

struct Detection {
  std::string class_name;
  float confidence;
  int x_min, y_min, x_max, y_max;
};

class ProjectTo3DNode : public rclcpp::Node {
public:
  ProjectTo3DNode() : Node("project_to_3d_node") {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter<std::string>("detections_topic", "/yolox/bounding_boxes");
    this->declare_parameter<std::string>("pointcloud_topic", "/objects_3d");
    this->declare_parameter<std::string>("detection3d_topic", "/detections_3d");
    this->declare_parameter<std::string>("output_frame", "d435_link");
    this->declare_parameter<std::vector<std::string>>("allowed_classes", {"cup", "bottle", "book", "bowl"});

    std::string rgb_topic, depth_topic, camera_info_topic, detections_topic, pointcloud_topic, detection3d_topic, output_frame;
    std::vector<std::string> allowed_classes;
    this->get_parameter("rgb_topic", rgb_topic);
    this->get_parameter("depth_topic", depth_topic);
    this->get_parameter("camera_info_topic", camera_info_topic);
    this->get_parameter("detections_topic", detections_topic);
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    this->get_parameter("detection3d_topic", detection3d_topic);
    this->get_parameter("output_frame", output_frame);
    this->get_parameter("allowed_classes", allowed_classes);

    RCLCPP_INFO(this->get_logger(), "Project To 3D Object Node Initialized");
    
    rgb_sub_.subscribe(this, rgb_topic);
    depth_sub_.subscribe(this, depth_topic);
    info_sub_.subscribe(this, camera_info_topic);

    detections_sub_ = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
      detections_topic, 10,
      std::bind(&ProjectTo3DNode::detectionsCallback, this, std::placeholders::_1));


    sync_.reset(new Sync(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_));
    sync_->registerCallback(std::bind(&ProjectTo3DNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
    detection_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(detection3d_topic, 10);

    allowed_classes_ = allowed_classes;
    output_frame_ = output_frame;
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr detections_sub_;
  std::vector<Detection> latest_detections_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;

  std::vector<std::string> allowed_classes_;
  std::string output_frame_;

  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) {

    if (latest_detections_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No detections received yet.");
      return;
    }
    if (!rgb_msg || !depth_msg || !info_msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null message(s).");
      return;
    }
    if ((rgb_msg->height != depth_msg->height) || (rgb_msg->width != depth_msg->width)) {
      RCLCPP_ERROR(this->get_logger(), "Image sizes are different.");
      return;
    }

    try {
      cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
      cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;

      float fx = info_msg->k[0];
      float fy = info_msg->k[4];
      float cx = info_msg->k[2];
      float cy = info_msg->k[5];

      if (fx == 0.0f || fy == 0.0f) {
        RCLCPP_ERROR(this->get_logger(), "Camera intrinsics fx or fy is zero!");
        return;
      }

      std::vector<Detection> detections = latest_detections_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      vision_msgs::msg::Detection3DArray detection_array;

      // Use parameterized output frame
      detection_array.header = rgb_msg->header;
      detection_array.header.frame_id = output_frame_;

      for (const auto &det : detections) {
        // If allowed_classes_ is empty, allow all classes
        if (!allowed_classes_.empty() && std::find(allowed_classes_.begin(), allowed_classes_.end(), det.class_name) == allowed_classes_.end()) {
          RCLCPP_DEBUG(this->get_logger(), "Skipping detection: %s", det.class_name.c_str());
          continue;
        }
        if (det.confidence < 0.3) {
          RCLCPP_DEBUG(this->get_logger(), "Skipping detection with low confidence: %s (%.2f)", det.class_name.c_str(), det.confidence);
          continue;
        }

        if (det.x_min < 0 || det.y_min < 0 || det.x_max > rgb.cols || det.y_max > rgb.rows) {
          RCLCPP_DEBUG(this->get_logger(), "Skipping detection with out-of-bounds coordinates: %s", det.class_name.c_str());
          continue;
        }

        if (det.x_min >= det.x_max || det.y_min >= det.y_max) {
          RCLCPP_DEBUG(this->get_logger(), "Skipping detection with invalid bounding box: %s", det.class_name.c_str());
          continue;
        }

        RCLCPP_DEBUG(this->get_logger(), "Processing detection: %s (confidence: %.2f)", det.class_name.c_str(), det.confidence);
        try {
          cv::Mat roi = depth(cv::Rect(det.x_min, det.y_min, det.x_max - det.x_min, det.y_max - det.y_min));
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

          for (int v = 0; v < roi.rows; ++v) {
            for (int u = 0; u < roi.cols; ++u) {
              uint16_t Z_raw = roi.at<uint16_t>(v, u);
              float Z = static_cast<float>(Z_raw) * 0.001f; // Convert mm to meters
              if (!std::isfinite(Z) || Z <= 0.0 || Z > 3.0) continue;
              int img_x = u + det.x_min;
              int img_y = v + det.y_min;
              if (img_x < 0 || img_x >= rgb.cols || img_y < 0 || img_y >= rgb.rows) continue;
              float X = (img_x - cx) * Z / fx;
              float Y = (img_y - cy) * Z / fy;
              if (!std::isfinite(X) || !std::isfinite(Y)) continue;

              pcl::PointXYZRGB point;
              point.x = X;
              point.y = Y;
              point.z = Z;
              cv::Vec3b color = rgb.at<cv::Vec3b>(img_y, img_x);
              point.r = color[2];
              point.g = color[1];
              point.b = color[0];
              cloud->points.push_back(point);
            }
          }

          // Use cloud for further processing (no clustering)
          float coverage = float(cloud->size()) / (roi.rows * roi.cols);
          if (coverage < 0.1) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping detection with low coverage: %s (coverage: %.2f)", det.class_name.c_str(), coverage);
            continue;
          }

          RCLCPP_DEBUG(this->get_logger(), "Processing cloud with %zu points and coverage %f for detection: %s", cloud->size(), coverage, det.class_name.c_str());

          pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
          sor.setInputCloud(cloud);
          sor.setMeanK(20);
          sor.setStddevMulThresh(1.0);
          sor.filter(*cloud);

          bool all_nan = true;
          // Transform points from camera optical frame to camera_link frame BEFORE adding to total_cloud
          for (auto& point : cloud->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
              all_nan = false;
            }
            float x_opt = point.x;
            float y_opt = point.y;
            float z_opt = point.z;
            point.x = z_opt;
            point.y = -x_opt;
            point.z = -y_opt;
          }

          if (all_nan) {
            RCLCPP_DEBUG(this->get_logger(), "All points are NaN/Inf for detection: %s", det.class_name.c_str());
            continue;
          }

          *total_cloud += *cloud;

          RCLCPP_DEBUG(this->get_logger(), "Filtered cloud size: %zu points for detection: %s", cloud->size(), det.class_name.c_str());

          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
          if (cloud->empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cloud is empty after removing NaNs for detection: %s", det.class_name.c_str());
            continue;
          }

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*cloud, centroid);

          RCLCPP_DEBUG(this->get_logger(), "Centroid for detection %s: (%f, %f, %f)", det.class_name.c_str(), centroid[0], centroid[1], centroid[2]);

          pcl::PointXYZRGB min_pt, max_pt;
          pcl::getMinMax3D(*cloud, min_pt, max_pt);

          RCLCPP_DEBUG(this->get_logger(), "Bounding box for detection %s: min(%f, %f, %f), max(%f, %f, %f)", 
                       det.class_name.c_str(), min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);

          vision_msgs::msg::Detection3D detection;
          detection.header = rgb_msg->header;
          detection.header.frame_id = output_frame_;

          vision_msgs::msg::ObjectHypothesisWithPose hypo;
          hypo.id = det.class_name;
          hypo.score = det.confidence;
          hypo.pose.pose.position.x = centroid[0];
          hypo.pose.pose.position.y = centroid[1];
          hypo.pose.pose.position.z = centroid[2];
          hypo.pose.pose.orientation.w = 1.0;

          detection.results.push_back(hypo);
          detection.bbox.center = hypo.pose.pose;
          detection.bbox.size.x = max_pt.x - min_pt.x;
          detection.bbox.size.y = max_pt.y - min_pt.y;
          detection.bbox.size.z = max_pt.z - min_pt.z;

          detection_array.detections.push_back(detection);
        } catch (const cv::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "OpenCV exception in detection loop: %s", e.what());
          continue;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Standard exception in detection loop: %s", e.what());
          continue;
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "Unknown exception in detection loop");
          continue;
        }
      }

      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*total_cloud, cloud_msg);
      cloud_msg.header = rgb_msg->header;
      cloud_msg.header.frame_id = output_frame_;
      pointcloud_pub_->publish(cloud_msg);
      detection_pub_->publish(detection_array);
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception in imageCallback: %s", e.what());
      return;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception in imageCallback: %s", e.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in imageCallback");
      return;
    }
  }

  void detectionsCallback(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg) {
    latest_detections_.clear();
    for (const auto &d : msg->bounding_boxes) {
      RCLCPP_DEBUG(this->get_logger(), "Received detection: %s with confidence %.2f", d.class_id.c_str(), d.probability);
        if (std::find(allowed_classes_.begin(), allowed_classes_.end(), d.class_id) != allowed_classes_.end()) {
            RCLCPP_DEBUG(this->get_logger(), "Adding detection: %s", d.class_id.c_str());
            latest_detections_.emplace_back(Detection{d.class_id, d.probability, d.xmin, d.ymin, d.xmax, d.ymax});
        }
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProjectTo3DNode>());
  rclcpp::shutdown();
  return 0;
}