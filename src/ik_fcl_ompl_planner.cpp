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

#include <geometry_msgs/msg/pose_stamped.hpp>
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
        this->declare_parameter("base_link", "torso_link");
        this->declare_parameter("right_tip", "right_wrist_yaw_link");
        this->declare_parameter("left_tip", "left_wrist_yaw_link");
        this->declare_parameter("goal_pose_topic", "/goal_pose");
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
        this->get_parameter("goal_pose_topic", goal_pose_topic_);
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
        if (!kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for right arm");
            rclcpp::shutdown();
            return;
        }
        printKDLChainInfo(kdl_chain_right, "right", this->get_logger());

        if (!kdl_tree.getChain(base_link_, left_tip_, kdl_chain_left)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for left arm");
            rclcpp::shutdown();
            return;
        }
        printKDLChainInfo(kdl_chain_left, "left", this->get_logger());

        // Parse joint limits from URDF for OMPL bounds
        for (const auto& joint_pair : urdf_model.joints_) {
            const auto& joint = joint_pair.second;
            if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC && joint->type != urdf::Joint::FIXED) continue;
            if (!joint->limits) continue;
            joint_limits_[joint->name] = std::make_pair(joint->limits->lower, joint->limits->upper);
        }

        // Build joint limits arrays for right and left arms
        KDL::JntArray right_lower(kdl_chain_right.getNrOfJoints()), right_upper(kdl_chain_right.getNrOfJoints());
        KDL::JntArray left_lower(kdl_chain_left.getNrOfJoints()), left_upper(kdl_chain_left.getNrOfJoints());
        size_t idx = 0;
        for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
            const auto& joint = kdl_chain_right.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                auto lim = joint_limits_.find(joint.getName());
                if (lim != joint_limits_.end()) {
                    right_lower(idx) = lim->second.first;
                    right_upper(idx) = lim->second.second;
                } else {
                    right_lower(idx) = -3.14;
                    right_upper(idx) = 3.14;
                }
                ++idx;
            }
        }
        idx = 0;
        for (unsigned int i = 0; i < kdl_chain_left.getNrOfSegments(); ++i) {
            const auto& joint = kdl_chain_left.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                auto lim = joint_limits_.find(joint.getName());
                if (lim != joint_limits_.end()) {
                    left_lower(idx) = lim->second.first;
                    left_upper(idx) = lim->second.second;
                } else {
                    left_lower(idx) = -3.14;
                    left_upper(idx) = 3.14;
                }
                ++idx;
            }
        }

        // Debug: Print right arm KDL chain joint names and limits
        std::ostringstream right_chain_oss;
        right_chain_oss << "Right arm KDL chain joints and limits:";
        idx = 0;
        for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
            const auto& joint = kdl_chain_right.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                right_chain_oss << "\n  " << joint.getName() << ": [" << right_lower(idx) << ", " << right_upper(idx) << "]";
                ++idx;
            } else {
                right_chain_oss << "\n  " << joint.getName() << ": [None]";
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", right_chain_oss.str().c_str());
        // Debug: Print left arm KDL chain joint names and limits
        std::ostringstream left_chain_oss;
        left_chain_oss << "Left arm KDL chain joints and limits:";
        idx = 0;
        for (unsigned int i = 0; i < kdl_chain_left.getNrOfSegments(); ++i) {
            const auto& joint = kdl_chain_left.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                left_chain_oss << "\n  " << joint.getName() << ": [" << left_lower(idx) << ", " << left_upper(idx) << "]";
                ++idx;
            } else {
                left_chain_oss << "\n  " << joint.getName() << ": [None]";
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", left_chain_oss.str().c_str());

        // Use longer timeout and error tolerance for TRAC-IK
        double ik_timeout = 1.0; // seconds
        double ik_tol = 1e-5;  // Tolerance for IK solutions
        TRAC_IK::SolveType solve_type = TRAC_IK::Distance;  // Distance, Manip1, Manip2, Speed
        ik_right = std::make_shared<TRAC_IK::TRAC_IK>(kdl_chain_right, right_lower, right_upper, ik_timeout, ik_tol, solve_type);
        ik_left = std::make_shared<TRAC_IK::TRAC_IK>(kdl_chain_left, left_lower, left_upper, ik_timeout, ik_tol, solve_type);

        assert(right_lower.rows() == kdl_chain_right.getNrOfJoints());
        assert(right_upper.rows() == kdl_chain_right.getNrOfJoints());

        assert(left_lower.rows() == kdl_chain_left.getNrOfJoints());
        assert(left_upper.rows() == kdl_chain_left.getNrOfJoints());

        fk_right_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_right);
        fk_left_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_left);

        buildCollisionObjects();

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&IKFCLPlannerNode::goalPoseCallback, this, _1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_targets", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&IKFCLPlannerNode::jointStateCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "IKFCLPlannerNode initialized with %zu joints", joint_limits_.size());
        RCLCPP_INFO(this->get_logger(), "Using base link: %s, right tip: %s, left tip: %s",
                    base_link_.c_str(), right_tip_.c_str(), left_tip_.c_str());
        RCLCPP_INFO(this->get_logger(), "Planning timeout: %.2f seconds, time step: %.2f seconds",
                    planning_timeout_, time_step_);
        RCLCPP_INFO(this->get_logger(), "Collision skip pairs: %zu", collision_skip_pairs_.size());
        for (const auto& pair : collision_skip_pairs_) {
            RCLCPP_INFO(this->get_logger(), "Skipping collision check for pair: %s", pair.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Planner type: %s", planner_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right arm: base_link = %s, tip_link = %s", base_link_.c_str(), right_tip_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left arm: base_link = %s, tip_link = %s", base_link_.c_str(), left_tip_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right arm KDL chain segments: %d", kdl_chain_right.getNrOfSegments());
        RCLCPP_INFO(this->get_logger(), "Left arm KDL chain segments: %d", kdl_chain_left.getNrOfSegments());
        RCLCPP_INFO(this->get_logger(), "Right arm KDL chain joints: %d", kdl_chain_right.getNrOfJoints());
        RCLCPP_INFO(this->get_logger(), "Left arm KDL chain joints: %d", kdl_chain_left.getNrOfJoints());

        // --- DEBUG: Print all URDF link names ---
        std::ostringstream urdf_links_oss;
        urdf_links_oss << "URDF links (" << urdf_model.links_.size() << "): ";
        for (const auto& link_pair : urdf_model.links_) {
            urdf_links_oss << link_pair.first << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", urdf_links_oss.str().c_str());

        // --- DEBUG: Print KDL chain link and joint sequence for right arm ---
        std::ostringstream kdl_right_oss;
        kdl_right_oss << "Right arm KDL chain: ";
        for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
            const auto& seg = kdl_chain_right.getSegment(i);
            kdl_right_oss << "[" << seg.getName() << ": ";
            const auto& joint = seg.getJoint();
            switch (joint.getType()) {
                case KDL::Joint::None: kdl_right_oss << "None"; break;
                case KDL::Joint::RotAxis: kdl_right_oss << "RotAxis"; break;
                case KDL::Joint::TransAxis: kdl_right_oss << "TransAxis"; break;
                case KDL::Joint::RotX: kdl_right_oss << "RotX"; break;
                case KDL::Joint::RotY: kdl_right_oss << "RotY"; break;
                case KDL::Joint::RotZ: kdl_right_oss << "RotZ"; break;
                case KDL::Joint::TransX: kdl_right_oss << "TransX"; break;
                case KDL::Joint::TransY: kdl_right_oss << "TransY"; break;
                case KDL::Joint::TransZ: kdl_right_oss << "TransZ"; break;
                default: kdl_right_oss << "Unknown"; break;
            }
            kdl_right_oss << ", joint='" << joint.getName() << "'], ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", kdl_right_oss.str().c_str());

        // --- DEBUG: Print KDL chain link and joint sequence for left arm ---
        std::ostringstream kdl_left_oss;
        kdl_left_oss << "Left arm KDL chain: ";
        for (unsigned int i = 0; i < kdl_chain_left.getNrOfSegments(); ++i) {
            const auto& seg = kdl_chain_left.getSegment(i);
            kdl_left_oss << "[" << seg.getName() << ": ";
            const auto& joint = seg.getJoint();
            switch (joint.getType()) {
                case KDL::Joint::None: kdl_left_oss << "None"; break;
                case KDL::Joint::RotAxis: kdl_left_oss << "RotAxis"; break;
                case KDL::Joint::TransAxis: kdl_left_oss << "TransAxis"; break;
                case KDL::Joint::RotX: kdl_left_oss << "RotX"; break;
                case KDL::Joint::RotY: kdl_left_oss << "RotY"; break;
                case KDL::Joint::RotZ: kdl_left_oss << "RotZ"; break;
                case KDL::Joint::TransX: kdl_left_oss << "TransX"; break;
                case KDL::Joint::TransY: kdl_left_oss << "TransY"; break;
                case KDL::Joint::TransZ: kdl_left_oss << "TransZ"; break;
                default: kdl_left_oss << "Unknown"; break;
            }
            kdl_left_oss << ", joint='" << joint.getName() << "'], ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", kdl_left_oss.str().c_str());

        // Debug: Print joint limits
        for (const auto& lim : joint_limits_) {
            RCLCPP_INFO(this->get_logger(), "Joint %s limits: [%.3f, %.3f]", lim.first.c_str(), lim.second.first, lim.second.second);
        }
    }

private:
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_right, kdl_chain_left;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_right, ik_left;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_right_solver, fk_left_solver;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::vector<std::string> joint_names_;
    std::vector<double> latest_joint_positions_;

    struct LinkCollision {
        std::shared_ptr<fcl::CollisionGeometryd> geometry;
        std::shared_ptr<fcl::CollisionObjectd> object;
        std::string segment_name;
        Eigen::Isometry3d local_transform;
    };

    std::map<std::string, LinkCollision> link_collisions;

    // Add joint_limits_ as a member variable
    std::map<std::string, std::pair<double, double>> joint_limits_;

    // Parameters
    double time_step_ = 0.05;
    double planning_timeout_ = 1.0;
    std::string base_link_ = "pelvis";
    std::string right_tip_ = "right_hand_palm_link";
    std::string left_tip_ = "left_hand_palm_link";
    std::string goal_pose_topic_ = "/goal_pose";
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
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: BOX [%.3f, %.3f, %.3f]", link->name.c_str(), box->dim.x, box->dim.y, box->dim.z);
            }
            else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
                urdf::Cylinder* cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
                geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: CYLINDER [radius=%.3f, length=%.3f]", link->name.c_str(), cyl->radius, cyl->length);
            }
            else if (link->collision->geometry->type == urdf::Geometry::MESH) {
                urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->collision->geometry.get());
                if (!mesh) {
                    RCLCPP_WARN(this->get_logger(), "Failed to cast mesh geometry for link %s", link->name.c_str());
                    continue;
                }
                std::string mesh_filename = mesh->filename;
                double scale_x = mesh->scale.x;
                double scale_y = mesh->scale.y;
                double scale_z = mesh->scale.z;
                RCLCPP_DEBUG(this->get_logger(), "Attempting to load mesh for link %s: %s", link->name.c_str(), mesh_filename.c_str());
                shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh_filename, Eigen::Vector3d(scale_x, scale_y, scale_z));
                if (!shape_mesh || shape_mesh->vertex_count == 0) {
                    RCLCPP_WARN(this->get_logger(), "Mesh for link %s is empty or failed to load (resource: %s)", link->name.c_str(), mesh_filename.c_str());
                    continue;
                }
                // Convert to FCL BVHModel
                auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
                std::vector<fcl::Vector3d> vertices;
                for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i) {
                    vertices.emplace_back(
                        shape_mesh->vertices[3*i+0],
                        shape_mesh->vertices[3*i+1],
                        shape_mesh->vertices[3*i+2]);
                }
                bvh->beginModel();
                for (unsigned int i = 0; i < shape_mesh->triangle_count; ++i) {
                    unsigned int idx0 = shape_mesh->triangles[3*i+0];
                    unsigned int idx1 = shape_mesh->triangles[3*i+1];
                    unsigned int idx2 = shape_mesh->triangles[3*i+2];
                    bvh->addTriangle(vertices[idx0], vertices[idx1], vertices[idx2]);
                }
                bvh->endModel();
                delete shape_mesh;
                geom = bvh;
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: MESH [%s] (scaled %.3f, %.3f, %.3f)", link->name.c_str(), mesh_filename.c_str(), scale_x, scale_y, scale_z);
            }
            else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
                urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
                if (!sphere) {
                    RCLCPP_WARN(this->get_logger(), "Failed to cast sphere geometry for link %s", link->name.c_str());
                    continue;
                }
                geom = std::make_shared<fcl::Sphered>(sphere->radius);
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: SPHERE [radius=%.3f]", link->name.c_str(), sphere->radius);
            }
            else {
                // Skip unknown geometry types for collision
                RCLCPP_WARN(this->get_logger(), "Skipping collision geometry for link %s: unknown geometry type", link->name.c_str());
                continue;
            }

            // Apply <origin> from collision tag
            Eigen::Isometry3d local_tf = Eigen::Isometry3d::Identity();
            const urdf::Pose& pose = link->collision->origin;
            const urdf::Vector3& p = pose.position;
            const urdf::Rotation& r = pose.rotation;
            double rx, ry, rz, rw;
            r.getQuaternion(rx, ry, rz, rw);
            Eigen::Quaterniond q(rw, rx, ry, rz);
            local_tf.linear() = q.toRotationMatrix();
            local_tf.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            auto obj = std::make_shared<fcl::CollisionObjectd>(geom);
            link_collisions[link->name] = {geom, obj, link->name, local_tf};
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_names_ = msg->name;
        latest_joint_positions_ = msg->position;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        bool use_right = pose->pose.position.y < 0.0;
        // Dynamically generate planning_joints from KDL chain
        const KDL::Chain& chain = use_right ? kdl_chain_right : kdl_chain_left;
        std::vector<std::string> planning_joints;
        std::set<std::string> planning_links; // <-- collect relevant links
        std::ostringstream seg_oss;
        seg_oss << "Planning chain segments: ";
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            const auto& seg = chain.getSegment(i);
            planning_links.insert(seg.getName());
            seg_oss << seg.getName() << ", ";
            const auto& joint = seg.getJoint();
            if (joint.getType() != KDL::Joint::None) {
                planning_joints.push_back(joint.getName());
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", seg_oss.str().c_str());

        // --- Joint order check: compare planning_joints to expected URDF joint order ---
        std::ostringstream expected_oss, actual_oss, warn_oss;
        expected_oss << "URDF joint_names_: ";
        for (const auto& j : joint_names_) expected_oss << j << ", ";
        actual_oss << "KDL planning_joints: ";
        for (const auto& j : planning_joints) actual_oss << j << ", ";
        RCLCPP_INFO(this->get_logger(), "%s", expected_oss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", actual_oss.str().c_str());
        // Check for missing or out-of-order joints
        bool order_ok = true;
        size_t last_found = 0;
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), planning_joints[i]);
            if (it == joint_names_.end()) {
                warn_oss << "[WARN] Planning joint '" << planning_joints[i] << "' not found in URDF joint_names_. ";
                order_ok = false;
            } else {
                size_t idx = std::distance(joint_names_.begin(), it);
                if (idx < last_found) {
                    warn_oss << "[WARN] Planning joint '" << planning_joints[i] << "' is out of order (index " << idx << "). ";
                    order_ok = false;
                }
                last_found = idx;
            }
        }
        if (order_ok) {
            RCLCPP_INFO(this->get_logger(), "KDL planning_joints match URDF joint_names_ order.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "KDL planning_joints do NOT match URDF joint_names_ order: %s", warn_oss.str().c_str());
            rclcpp::shutdown();
            return;
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
        // Debug: Print start state joint names and values
        std::ostringstream oss;
        oss << "Start state: ";
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            oss << planning_joints[i] << "=" << planning_positions[i] << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

        KDL::Frame target_frame(KDL::Rotation::Quaternion(
                                    pose->pose.orientation.x,
                                    pose->pose.orientation.y,
                                    pose->pose.orientation.z,
                                    pose->pose.orientation.w),
                                KDL::Vector(
                                    pose->pose.position.x,
                                    pose->pose.position.y,
                                    pose->pose.position.z));

        // Debug target frame
        double rx, ry, rz;
        target_frame.M.GetRPY(rx, ry, rz);
        RCLCPP_INFO(this->get_logger(), "Target frame details:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
            target_frame.p.x(), target_frame.p.y(), target_frame.p.z());
        RCLCPP_INFO(this->get_logger(), "  RPY: [%.3f, %.3f, %.3f]", rx, ry, rz);
        
        // Print planning joint names and order
        std::ostringstream joint_oss;
        joint_oss << "Planning joints (order): ";
        for (const auto& j : planning_joints) joint_oss << j << ", ";
        RCLCPP_INFO(this->get_logger(), "%s", joint_oss.str().c_str());
        // Print base and tip link names
        RCLCPP_INFO(this->get_logger(), "KDL/TRAC-IK base link: %s, tip link: %s", base_link_.c_str(), use_right ? right_tip_.c_str() : left_tip_.c_str());
        // Print joint limits
        for (const auto& j : planning_joints) {
            auto lim_it = joint_limits_.find(j);
            if (lim_it != joint_limits_.end()) {
                RCLCPP_INFO(this->get_logger(), "Joint %s limits: [%.3f, %.3f]", j.c_str(), lim_it->second.first, lim_it->second.second);
            } else {
                RCLCPP_WARN(this->get_logger(), "Joint %s has no limits, using [-3.14, 3.14]", j.c_str());
            }
        }
        // Try IK with current state as seed
        KDL::JntArray seed(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) seed(i) = planning_positions[i];
        auto& solver = use_right ? ik_right : ik_left;
        KDL::JntArray goal(planning_joints.size());
        int ik_result = solver->CartToJnt(seed, target_frame, goal);
        
        // Print detailed information about seed and solution
        std::ostringstream seed_oss, goal_oss;
        seed_oss << "IK seed state: [ ";
        goal_oss << "IK solution:   [ ";
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            seed_oss << std::fixed << std::setprecision(6) << seed(i);
            goal_oss << std::fixed << std::setprecision(6) << goal(i);
            if (i + 1 < planning_joints.size()) {
                seed_oss << ", ";
                goal_oss << ", ";
            }
        }
        
        // Check specific return codes
        const char* result_str;
        switch (ik_result) {
            case -1: result_str = "Solver init error: invalid chain or limits"; break;
            case -3: result_str = "No solution found"; break;
            default: result_str = ik_result > 0 ? "Success" : "Unknown error";
        }
        RCLCPP_INFO(this->get_logger(), "TRAC-IK result: %s (code: %d)", result_str, ik_result);
        
        bool ik_success = (ik_result > 0);
        seed_oss << " ]";
        goal_oss << " ]";
        RCLCPP_INFO(this->get_logger(), "%s", seed_oss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", goal_oss.str().c_str());

        if (!ik_success) {
            RCLCPP_WARN(this->get_logger(), "IK failed with current state as seed. Trying neutral seed.");
            // Try neutral seed (all zeros)
            KDL::JntArray neutral_seed(planning_joints.size());
            for (size_t i = 0; i < planning_joints.size(); ++i) neutral_seed(i) = 0.0;
            if (solver->CartToJnt(neutral_seed, target_frame, goal) > 0) {
                RCLCPP_INFO(this->get_logger(), "IK succeeded with neutral seed.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "IK failed with both current and neutral seed. Aborting.");
                return;
            }
        }
        // Print input goal pose
        RCLCPP_INFO(this->get_logger(), "Input goal pose: position [%.3f, %.3f, %.3f], orientation [%.3f, %.3f, %.3f, %.3f]", 
            pose->pose.position.x, pose->pose.position.y, pose->pose.position.z, 
            pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
        // Compute and print current end-effector pose (FK)
        KDL::JntArray current_jnt(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) current_jnt(i) = planning_positions[i];
        KDL::Frame current_ee;
        auto& fk_solver = use_right ? fk_right_solver : fk_left_solver;
        if (fk_solver->JntToCart(current_jnt, current_ee) >= 0) {
            double x = current_ee.p.x(), y = current_ee.p.y(), z = current_ee.p.z();
            double qx, qy, qz, qw;
            current_ee.M.GetQuaternion(qx, qy, qz, qw);
            RCLCPP_INFO(this->get_logger(), "Current EE pose: position [%.3f, %.3f, %.3f], orientation [%.3f, %.3f, %.3f, %.3f]", x, y, z, qx, qy, qz, qw);
        } else {
            RCLCPP_WARN(this->get_logger(), "FK failed for current joint state");
        }
        if (!solver->CartToJnt(seed, target_frame, goal)) {
            RCLCPP_WARN(this->get_logger(), "IK failed for goal pose");
            return;
        }
        // Print computed IK solution for goal pose
        std::ostringstream ikoss;
        ikoss << "IK solution for goal pose: ";
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            ikoss << planning_joints[i] << "=" << goal(i);
            if (i + 1 < planning_joints.size()) ikoss << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ikoss.str().c_str());

        // If IK result is too close to the seed, try random seeds up to N times
        double ik_max_diff = 0.0;
        double solution_threshold = 0.01; // Minimum required difference in at least one joint
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            double diff = std::abs(goal(i) - seed(i));
            if (diff > ik_max_diff) ik_max_diff = diff;
        }
        int max_random_tries = 20;  // Increased from 5 to 20
        int random_tries = 0;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        
        KDL::JntArray best_solution = goal;
        double best_diff = ik_max_diff;
        
        while (best_diff < solution_threshold && random_tries < max_random_tries) {
            RCLCPP_INFO(this->get_logger(), 
                "IK solution is too close to seed (max diff %.6f). Trying random seed (%d/%d)", 
                best_diff, random_tries+1, max_random_tries);
                
            KDL::JntArray random_seed(planning_joints.size());
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                auto lim_it = joint_limits_.find(planning_joints[i]);
                double low = lim_it != joint_limits_.end() ? lim_it->second.first : -3.14;
                double high = lim_it != joint_limits_.end() ? lim_it->second.second : 3.14;
                std::uniform_real_distribution<> dis(low, high);
                random_seed(i) = dis(gen);
            }
            
            // Try IK with random seed
            KDL::JntArray new_goal;
            if (solver->CartToJnt(random_seed, target_frame, new_goal)) {
                double max_diff = 0.0;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    double diff = std::abs(new_goal(i) - random_seed(i));
                    if (diff > max_diff) max_diff = diff;
                }
                
                // Update best solution if this one is more different from its seed
                if (max_diff > best_diff) {
                    best_diff = max_diff;
                    best_solution = new_goal;
                    
                    // Print the improved solution
                    std::ostringstream sol_oss;
                    sol_oss << "Found better solution (diff=" << max_diff << "): [ ";
                    for (size_t i = 0; i < planning_joints.size(); ++i) {
                        sol_oss << std::fixed << std::setprecision(6) << new_goal(i);
                        if (i + 1 < planning_joints.size()) sol_oss << ", ";
                    }
                    sol_oss << " ]";
                    RCLCPP_INFO(this->get_logger(), "%s", sol_oss.str().c_str());
                }
            }
            random_tries++;
        }
        
        if (best_diff < solution_threshold) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find a sufficiently different IK solution after %d attempts. Best difference: %.6f", max_random_tries, best_diff);
            return;
        }
        
        // Use the best solution found
        goal = best_solution;

        // Compute and print current end-effector pose (FK)
        KDL::JntArray current_jnt_final(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) current_jnt_final(i) = goal(i);
        KDL::Frame final_ee;
        if (fk_solver->JntToCart(current_jnt_final, final_ee) >= 0) {
            double x = final_ee.p.x(), y = final_ee.p.y(), z = final_ee.p.z();
            double qx, qy, qz, qw;
            final_ee.M.GetQuaternion(qx, qy, qz, qw);
            RCLCPP_INFO(this->get_logger(), "Final EE pose: position [%.3f, %.3f, %.3f], orientation [%.3f, %.3f, %.3f, %.3f]", x, y, z, qx, qy, qz, qw);
        } else {
            RCLCPP_WARN(this->get_logger(), "FK failed for final joint state");
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
        // Pass planning_links to the state validity checker
        ss->setStateValidityChecker([this, use_right, planning_joints, planning_links](const ob::State* state) {
            const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            KDL::JntArray joints(planning_joints.size());
            std::ostringstream state_oss;
            state_oss << "[ ";
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                joints(i) = values[i];
                state_oss << planning_joints[i] << "=" << values[i];
                if (i + 1 < planning_joints.size()) state_oss << ", ";
            }
            state_oss << " ]";
            bool is_in_collision = isInCollision(joints, use_right, this->collision_skip_pairs_, planning_links);
            RCLCPP_DEBUG(this->get_logger(), "Checking collision for state %s, result: %s", state_oss.str().c_str(), is_in_collision ? "Collision" : "Free");
            return !is_in_collision;
        });
        ob::ScopedState<> start(space), goal_state(space);
        for (size_t i = 0; i < planning_joints.size(); ++i) start[i] = planning_positions[i];
        for (size_t i = 0; i < planning_joints.size(); ++i) goal_state[i] = goal(i);
        ss->setStartAndGoalStates(start, goal_state);

        // --- Debug: Check start and goal state validity before planning ---
        {
            bool start_valid = ss->getStateValidityChecker()->isValid(start.get());
            bool goal_valid = ss->getStateValidityChecker()->isValid(goal_state.get());
            std::ostringstream soss, goss;
            soss << "Start state validity: " << (start_valid ? "VALID" : "INVALID") << ". Values: ";
            goss << "Goal state validity: " << (goal_valid ? "VALID" : "INVALID") << ". Values: ";
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                soss << planning_joints[i] << "=" << start[i] << ", ";
                goss << planning_joints[i] << "=" << goal_state[i] << ", ";
            }
            RCLCPP_WARN(this->get_logger(), "%s", soss.str().c_str());
            RCLCPP_WARN(this->get_logger(), "%s", goss.str().c_str());
        }

        //ss->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        
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
            for (const auto& pt : traj_msg.points) {
                if (pt.positions.size() != traj_msg.joint_names.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Trajectory point size mismatch: %zu vs %zu", pt.positions.size(), traj_msg.joint_names.size());
                    return;
                }
            }
            // --- Debug: Check if final state matches goal ---
            if (!traj_msg.points.empty()) {
                const auto& final_pt = traj_msg.points.back();
                double max_diff = 0.0;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    double diff = std::abs(final_pt.positions[i] - goal(i));
                    if (diff > max_diff) max_diff = diff;
                }
                RCLCPP_WARN(this->get_logger(), "Final trajectory state vs goal: max joint diff = %.6f", max_diff);
                if (max_diff < 1e-3) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory achieves the goal (within tolerance)");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Trajectory does NOT reach the goal (max diff > 1e-3)");
                }
            }
            traj_msg.header.stamp = this->now();
            traj_pub_->publish(traj_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "OMPL failed to find a path for goal pose");
        }
    }

    // Restrict collision checking to only pairs where at least one link is in planning_links
    bool isInCollision(const KDL::JntArray& joints, bool use_right, const std::vector<std::string>& skip_pairs = {}, const std::set<std::string>& planning_links = {})
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
                Eigen::Isometry3d world_tf = Eigen::Isometry3d::Identity();
                world_tf.linear() = rot;
                world_tf.translation() = trans;
                // Combine FK and local collision origin
                Eigen::Isometry3d final_tf = world_tf * lc.local_transform;
                fcl::Transform3d tf(final_tf.matrix());
                lc.object->setTransform(tf);
                // Debug: Print translation and rotation for each collision object
                //RCLCPP_INFO(this->get_logger(), "Collision object %s translation: [%.3f, %.3f, %.3f]", link_name.c_str(), trans.x(), trans.y(), trans.z());
                //RCLCPP_INFO(this->get_logger(), "Collision object %s rotation matrix: [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]", link_name.c_str(),
                //    rot(0,0), rot(0,1), rot(0,2), rot(1,0), rot(1,1), rot(1,2), rot(2,0), rot(2,1), rot(2,2));
                // Print bounding box (AABB) in world coordinates
                //fcl::AABBd aabb = lc.object->getAABB();
                //RCLCPP_INFO(this->get_logger(), "Collision object %s AABB: min[%.3f, %.3f, %.3f] max[%.3f, %.3f, %.3f]", link_name.c_str(),
                //    aabb.min_.x(), aabb.min_.y(), aabb.min_.z(), aabb.max_.x(), aabb.max_.y(), aabb.max_.z());
            } else {
                // If not updated, print a warning
                //RCLCPP_WARN(this->get_logger(), "Collision object %s not updated with a transform (may be at origin)", link_name.c_str());
            }
        }
        for (auto it1 = link_collisions.begin(); it1 != link_collisions.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != link_collisions.end(); ++it2) {
                // Only check if at least one link is in planning_links
                if (!planning_links.empty() &&
                    (planning_links.find(it1->first) == planning_links.end() ||
                     planning_links.find(it2->first) == planning_links.end())) {
                    continue;
                }
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
                if (res.isCollision()) {
                    RCLCPP_WARN(this->get_logger(), "Collision detected between %s and %s", it1->first.c_str(), it2->first.c_str());
                    return true;
                }
            }
        }
        return false;
    }

    // Utility: Print KDL chain structure for diagnostics
    void printKDLChainInfo(const KDL::Chain& chain, const std::string& chain_name, rclcpp::Logger logger) {
        std::ostringstream oss;
        oss << "KDL Chain [" << chain_name << "] structure:";
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            const auto& seg = chain.getSegment(i);
            const auto& joint = seg.getJoint();
            oss << "\n  Segment " << i << ": name='" << seg.getName() << "', joint='" << joint.getName() << "', type=";
            switch (joint.getType()) {
                case KDL::Joint::None: oss << "None"; break;
                case KDL::Joint::RotAxis: oss << "RotAxis"; break;
                case KDL::Joint::RotX: oss << "RotX"; break;
                case KDL::Joint::RotY: oss << "RotY"; break;
                case KDL::Joint::RotZ: oss << "RotZ"; break;
                case KDL::Joint::TransAxis: oss << "TransAxis"; break;
                case KDL::Joint::TransX: oss << "TransX"; break;
                case KDL::Joint::TransY: oss << "TransY"; break;
                case KDL::Joint::TransZ: oss << "TransZ"; break;
                default: oss << "Unknown";
            }
        }
        RCLCPP_INFO(logger, "%s", oss.str().c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKFCLPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
