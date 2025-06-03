#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>
#include <fcl/fcl.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <std_msgs/msg/string.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <resource_retriever/retriever.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.h>
#include <map>
#include <vector>
#include <string>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

class IKFCLPlannerNode : public rclcpp::Node
{
public:
    IKFCLPlannerNode() : Node("ik_fcl_ompl_planner")
    {
        // Parameterization
        this->declare_parameter("trajectory_time_step", 0.05);
        this->declare_parameter("planning_timeout", 1.0);
        this->declare_parameter("base_link", "pelvis");
        this->declare_parameter("right_tip", "right_hand_palm_link");
        this->declare_parameter("left_tip", "left_hand_palm_link");
        this->declare_parameter("detection_topic", "/detections");
        this->declare_parameter("selected_class_topic", "/selected_detection_class");
        this->declare_parameter("planner_type", "RRTConnect");
        this->declare_parameter("collision_skip_pairs", std::vector<std::string>{});

        // Fetch robot_description from global parameter server
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
            RCLCPP_FATAL(this->get_logger(), "robot_description parameter is missing or empty. Cannot continue.");
            rclcpp::shutdown();
            return;
        }

        this->get_parameter("trajectory_time_step", time_step_);
        this->get_parameter("planning_timeout", planning_timeout_);
        this->get_parameter("base_link", base_link_);
        this->get_parameter("right_tip", right_tip_);
        this->get_parameter("left_tip", left_tip_);
        this->get_parameter("detection_topic", detection_topic_);
        this->get_parameter("selected_class_topic", selected_class_topic_);
        this->get_parameter("planner_type", planner_type_);
        this->get_parameter("collision_skip_pairs", collision_skip_pairs_);

        if (!urdf_model.initString(urdf_xml)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF");
            rclcpp::shutdown();
            return;
        }

        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create KDL tree");
            rclcpp::shutdown();
            return;
        }

        ik_right = std::make_shared<TRAC_IK::TRAC_IK>(base_link_, right_tip_, urdf_xml);
        ik_left = std::make_shared<TRAC_IK::TRAC_IK>(base_link_, left_tip_, urdf_xml);

        if (!kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for right arm");
            rclcpp::shutdown();
            return;
        }

        if (!kdl_tree.getChain(base_link_, left_tip_, kdl_chain_left)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for left arm");
            rclcpp::shutdown();
            return;
        }

        fk_right_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_right);
        fk_left_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_left);

        buildCollisionObjects();

        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            detection_topic_, 10, std::bind(&IKFCLPlannerNode::detectionCallback, this, _1));
        selected_class_sub_ = this->create_subscription<std_msgs::msg::String>(
            selected_class_topic_, 10, std::bind(&IKFCLPlannerNode::selectedClassCallback, this, _1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_targets", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&IKFCLPlannerNode::jointStateCallback, this, _1));

        // Parse joint limits from URDF for OMPL bounds
        for (const auto& joint_pair : urdf_model.joints_) {
            const auto& joint = joint_pair.second;
            if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) continue;
            if (!joint->limits) continue;
            joint_limits_[joint->name] = std::make_pair(joint->limits->lower, joint->limits->upper);
        }
    }

private:
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_right, kdl_chain_left;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_right, ik_left;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_right_solver, fk_left_solver;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_class_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    vision_msgs::msg::Detection3DArray::SharedPtr last_detections_;
    std::optional<std::string> selected_class_;
    std::vector<std::string> joint_names_;
    std::vector<double> latest_joint_positions_;

    struct LinkCollision {
        std::shared_ptr<fcl::CollisionGeometryd> geometry;
        std::shared_ptr<fcl::CollisionObjectd> object;
        std::string segment_name;
    };

    std::map<std::string, LinkCollision> link_collisions;

    // Add joint_limits_ as a member variable
    std::map<std::string, std::pair<double, double>> joint_limits_;

    // Map from class name (string) to int64 id
    std::map<std::string, int64_t> class_name_to_id_ = {
        {"cup", 1},
        {"bottle", 2},
        {"can", 3},
        // Add more mappings as needed
    };

    // Parameters
    double time_step_ = 0.05;
    double planning_timeout_ = 1.0;
    std::string base_link_ = "pelvis";
    std::string right_tip_ = "right_hand_palm_link";
    std::string left_tip_ = "left_hand_palm_link";
    std::string detection_topic_ = "/detections";
    std::string selected_class_topic_ = "/selected_detection_class";
    std::string planner_type_ = "RRTConnect";
    std::vector<std::string> collision_skip_pairs_;
    std::string log_level_ = "info";

    void buildCollisionObjects()
    {
        for (const auto& link_pair : urdf_model.links_) {
            auto link = link_pair.second;
            if (!link->collision || !link->collision->geometry) continue;

            std::shared_ptr<fcl::CollisionGeometryd> geom;
            if (link->collision->geometry->type == urdf::Geometry::BOX) {
                urdf::Box* box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
                geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
            }
            else if (link->collision->geometry->type == urdf::Geometry::MESH) {
                urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->collision->geometry.get());
                if (!mesh) continue;
                // Load mesh using geometric_shapes
                shapes::Shape* shape = shapes::createMeshFromResource(mesh->filename);
                shapes::Mesh* shape_mesh = dynamic_cast<shapes::Mesh*>(shape);
                if (!shape_mesh) {
                    delete shape;
                    continue;
                }
                // Apply scale manually if needed
                if (mesh->scale.x != 1.0 || mesh->scale.y != 1.0 || mesh->scale.z != 1.0) {
                    for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i) {
                        shape_mesh->vertices[3 * i + 0] *= mesh->scale.x;
                        shape_mesh->vertices[3 * i + 1] *= mesh->scale.y;
                        shape_mesh->vertices[3 * i + 2] *= mesh->scale.z;
                    }
                }
                // Convert to FCL BVHModel
                auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
                std::vector<fcl::Vector3d> points;
                for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i) {
                    points.emplace_back(
                        shape_mesh->vertices[3 * i + 1],
                        shape_mesh->vertices[3 * i + 2]);
                }
                std::vector<fcl::Triangle> triangles;
                for (unsigned int i = 0; i < shape_mesh->triangle_count; ++i) {
                    triangles.emplace_back(
                        shape_mesh->triangles[3 * i + 0],
                        shape_mesh->triangles[3 * i + 1],
                        shape_mesh->triangles[3 * i + 2]);
                }
                bvh->beginModel();
                bvh->addSubModel(points, triangles);
                bvh->endModel();
                geom = bvh;
                delete shape_mesh;
            }
            else continue;

            if (!geom) continue;
            auto obj = std::make_shared<fcl::CollisionObjectd>(geom);
            link_collisions[link->name] = {geom, obj, link->name};
        }
    }

    void selectedClassCallback(const std_msgs::msg::String::SharedPtr msg) {
        selected_class_ = msg->data;
        tryPlanForSelectedClass();
    }

    void detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        last_detections_ = msg;
        tryPlanForSelectedClass();
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_names_ = msg->name;
        latest_joint_positions_ = msg->position;
    }

    void tryPlanForSelectedClass() {
        if (!selected_class_ || !last_detections_) return;
        // Map selected_class_ string to int64 id
        //auto id_it = class_name_to_id_.find(*selected_class_);
        //if (id_it == class_name_to_id_.end()) return;
        //int64_t selected_id = id_it->second;
        for (const auto& detection : last_detections_->detections) {
            if (detection.results.empty()) continue;
            if (detection.results[0].id == selected_class_) {
                geometry_msgs::msg::Pose pose = detection.bbox.center;
                bool use_right = pose.position.y < 0.0;
                // Dynamically generate planning_joints from KDL chain
                const KDL::Chain& chain = use_right ? kdl_chain_right : kdl_chain_left;
                std::vector<std::string> planning_joints;
                for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
                    const auto& seg = chain.getSegment(i);
                    const auto& joint = seg.getJoint();
                    if (joint.getType() != KDL::Joint::None) {
                        planning_joints.push_back(joint.getName());
                    }
                }
                std::vector<double> planning_positions;
                for (const auto& jname : planning_joints) {
                    auto it = std::find(joint_names_.begin(), joint_names_.end(), jname);
                    if (it != joint_names_.end()) {
                        size_t idx = std::distance(joint_names_.begin(), it);
                        planning_positions.push_back(latest_joint_positions_[idx]);
                    } else {
                        planning_positions.push_back(0.0); // fallback if not found
                    }
                }
                KDL::Frame target_frame(KDL::Rotation::Quaternion(
                                            pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w),
                                        KDL::Vector(
                                            pose.position.x,
                                            pose.position.y,
                                            pose.position.z));
                KDL::JntArray seed(planning_joints.size());
                for (size_t i = 0; i < planning_joints.size(); ++i) seed(i) = planning_positions[i];
                KDL::JntArray goal;
                auto& solver = use_right ? ik_right : ik_left;
                if (!solver->CartToJnt(seed, target_frame, goal)) {
                    RCLCPP_WARN(this->get_logger(), "IK failed for selected class instance");
                    return;
                }

                // OMPL bounds: use URDF joint limits if available, else fallback to [-3.14, 3.14]
                auto space = std::make_shared<ob::RealVectorStateSpace>(planning_joints.size());
                ob::RealVectorBounds bounds(planning_joints.size());
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    auto lim_it = joint_limits_.find(planning_joints[i]);
                    if (lim_it != joint_limits_.end()) {
                        bounds.setLow(i, lim_it->second.first);
                        bounds.setHigh(i, lim_it->second.second);
                    } else {
                        bounds.setLow(i, -3.14);
                        bounds.setHigh(i, 3.14);
                    }
                }
                space->setBounds(bounds);

                auto ss = std::make_shared<og::SimpleSetup>(space);

                ss->setStateValidityChecker([this, use_right, planning_joints](const ob::State* state) {
                    const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
                    KDL::JntArray joints(planning_joints.size());
                    for (size_t i = 0; i < planning_joints.size(); ++i) joints(i) = values[i];
                    // Optionally skip collision pairs
                    return !isInCollision(joints, use_right, this->collision_skip_pairs_);
                });

                ob::ScopedState<> start(space), goal_state(space);
                for (size_t i = 0; i < planning_joints.size(); ++i) start[i] = planning_positions[i];
                for (size_t i = 0; i < planning_joints.size(); ++i) goal_state[i] = goal(i);

                ss->setStartAndGoalStates(start, goal_state);
                // Planner type parameterization
                if (planner_type_ == "RRTConnect") {
                    ss->setPlanner(std::make_shared<og::RRTConnect>(ss->getSpaceInformation()));
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown planner_type '%s', defaulting to RRTConnect", planner_type_.c_str());
                    ss->setPlanner(std::make_shared<og::RRTConnect>(ss->getSpaceInformation()));
                }

                if (ss->solve(planning_timeout_)) {
                    auto path = ss->getSolutionPath();
                    path.interpolate();
                    const auto& states = path.getStates();

                    trajectory_msgs::msg::JointTrajectory traj_msg;
                    traj_msg.joint_names = planning_joints;

                    for (size_t idx = 0; idx < states.size(); ++idx) {
                        const auto& state = states[idx];
                        trajectory_msgs::msg::JointTrajectoryPoint point;
                        for (size_t i = 0; i < planning_joints.size(); ++i) {
                            point.positions.push_back(state->as<ob::RealVectorStateSpace::StateType>()->values[i]);
                        }
                        point.time_from_start = rclcpp::Duration::from_seconds(time_step_ * (idx + 1));
                        traj_msg.points.push_back(point);
                    }

                    // Sanity check: joint_names and positions size must match
                    for (const auto& pt : traj_msg.points) {
                        if (pt.positions.size() != traj_msg.joint_names.size()) {
                            RCLCPP_ERROR(this->get_logger(), "Trajectory point size mismatch: %zu vs %zu", pt.positions.size(), traj_msg.joint_names.size());
                            return;
                        }
                    }

                    traj_msg.header.stamp = this->now();
                    traj_pub_->publish(traj_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "OMPL failed to find a path for selected class instance");
                }
                break;
            }
        }
    }

    // Update isInCollision to optionally skip pairs
    bool isInCollision(const KDL::JntArray& joints, bool use_right, const std::vector<std::string>& skip_pairs = {})
    {
        auto& fk_solver = use_right ? fk_right_solver : fk_left_solver;
        auto& kdl_chain = use_right ? kdl_chain_right : kdl_chain_left;
        std::map<std::string, KDL::Frame> segment_frames;
        KDL::Frame out;
        for (size_t i = 0; i < kdl_chain.getNrOfSegments(); ++i) {
            if (fk_solver->JntToCart(joints, out, i + 1) >= 0) {
                const auto& seg_name = kdl_chain.getSegment(i).getName();
                segment_frames[seg_name] = out;
            }
        }
        for (auto& [link_name, lc] : link_collisions) {
            auto it = segment_frames.find(lc.segment_name);
            if (it != segment_frames.end()) {
                const auto& frame = it->second;
                Eigen::Matrix3d rot;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        rot(r, c) = frame.M(r, c);
                Eigen::Vector3d trans(frame.p.x(), frame.p.y(), frame.p.z());
                fcl::Transform3d tf(rot);
                tf.translation() = trans;
                lc.object->setTransform(tf);
            }
        }
        for (auto it1 = link_collisions.begin(); it1 != link_collisions.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != link_collisions.end(); ++it2) {
                // Skip collision pairs if specified (format: "link1:link2")
                std::string pair1 = it1->first + ":" + it2->first;
                std::string pair2 = it2->first + ":" + it1->first;
                if (std::find(skip_pairs.begin(), skip_pairs.end(), pair1) != skip_pairs.end() ||
                    std::find(skip_pairs.begin(), skip_pairs.end(), pair2) != skip_pairs.end()) {
                    continue;
                }
                fcl::CollisionRequestd req;
                fcl::CollisionResultd res;
                fcl::collide(it1->second.object.get(), it2->second.object.get(), req, res);
                if (res.isCollision()) return true;
            }
        }
        return false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKFCLPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
