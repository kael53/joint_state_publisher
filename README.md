# unitree_g1_dex3_stack

## Overview

**unitree_g1_dex3_stack** is a full-featured ROS 2 manipulation stack for the Unitree G1 humanoid robot equipped with DEX3 hands. This package provides all the core nodes and configuration needed for robust upper-body and hand control, including:

- **Joint state publishing** for visualization and feedback
- **Hand (gripper) control** with tactile feedback-driven grasping
- **Trajectory execution** for the arm and hand
- **Motion planning** using OMPL, FCL, and TRAC-IK, with collision checking
- **Dynamic configuration** from the robot's URDF (no hardcoded joint lists)

This stack is designed for tabletop object picking and manipulation, and is modular, robust, and easy to extend.

---

## Features

- **Dynamic URDF-based configuration:**
  - All joint names, limits, and mappings are extracted from the URDF provided in `/robot_description`.
  - No hardcoded joint lists; robust to changes in robot model.
- **Hand control:**
  - Feedback-driven closed-loop grasping using tactile sensors.
  - Parameterized and robust open/close routines.
- **Trajectory execution:**
  - Executes joint-space trajectories for the arm and hand.
  - Ensures safe, clamped commands within joint limits.
- **Motion planning:**
  - Uses OMPL (RRTConnect), FCL for collision checking, and TRAC-IK for fast inverse kinematics.
  - Plans collision-free trajectories to detected objects.
- **Perception integration:**
  - Projects 2D detections to 3D using RGB-D images and publishes 3D object locations for manipulation.
- **Integration-ready:**
  - Designed to integrate with perception pipelines (object detection, pose estimation).
  - Modular nodes for easy extension or replacement.

---

## Nodes

- `joint_state_publisher`: Publishes unified joint states for the robot and both hands, dynamically sized and mapped from the URDF.
- `dex3_controller`: Controls the DEX3 hand (left or right), supporting robust open/close commands and tactile feedback-driven grasping.
- `joint_trajectory_executor`: Executes arm and hand trajectories, opening/closing the hand at the right time, and sending safe commands to the robot.
- `ik_fcl_ompl_planner`: Plans collision-free arm trajectories to target poses using OMPL, FCL, and TRAC-IK, integrating with perception topics for object picking.
- `project_to_3d_node`: Projects 2D object detections (bounding boxes) from RGB-D images into 3D point clouds and publishes 3D object detections for manipulation. Integrates with YOLOX or similar perception pipelines and outputs ROS 2 point clouds and Detection3DArray messages.

---

## Usage

1. **Launch the robot model and joint states:**
   - Use the provided launch file to start the robot state publisher and joint state publisher with your robot's URDF.
   - Example:
     ```bash
     ros2 launch unitree_g1_dex3_stack robot.launch.py
     # You can override the URDF file:
     ros2 launch unitree_g1_dex3_stack robot.launch.py urdf_name:=g1_29dof_with_hand_rev_1_0.urdf
     # Or provide a full path:
     ros2 launch unitree_g1_dex3_stack robot.launch.py urdf_path:=/path/to/your.urdf
     ```
2. **Launch the perception pipeline:**
   - Brings up the RealSense camera, YOLOX object detector, and 2D-to-3D projection node.
   - Example:
     ```bash
     ros2 launch unitree_g1_dex3_stack perception.launch.py
     # You can override YOLOX model, output topics, frame, and allowed classes:
     ros2 launch unitree_g1_dex3_stack perception.launch.py yolox_model_path:=/path/to/model.trt pointcloud_topic:=/my_cloud detection3d_topic:=/my_detections output_frame:=my_frame allowed_classes:="['cup','bottle']"
     ```
3. **Launch the planner node:**
   - Starts the OMPL+FCL+IK-based motion planner. All parameters can be set from the command line.
   - Example:
     ```bash
     ros2 launch unitree_g1_dex3_stack planner.launch.py planning_timeout:=2.0 planner_type:=RRTConnect log_level:=debug
     ```
4. **Launch the control nodes:**
   - Starts the joint trajectory executor and both DEX3 hand controllers (left and right).
   - Example:
     ```bash
     ros2 launch unitree_g1_dex3_stack control.launch.py
     ```
5. **Send object detections and select a target:**
   - (Perception pipeline not included; integrate your own or use standard ROS 2 perception nodes.)
6. **Plan and execute a pick:**
   - The planner node will generate a trajectory to the selected object and publish it.
   - The executor node will move the arm and hand to pick the object using tactile feedback.

---

## Directory Structure

- `src/` — Source code for all nodes
- `include/` — Shared headers (joint definitions, mappings)
- `robots/` — URDF and mesh files for the G1 and DEX3 hands
- `launch/` — Launch files for starting the stack

---

## Dependencies

- ROS 2 (Foxy or later)
- [unitree_hg](https://github.com/unitreerobotics/unitree_ros2) (custom messages)
- OMPL, FCL, TRAC-IK, KDL, geometric_shapes, resource_retriever, vision_msgs

All dependencies are declared in `package.xml` and `CMakeLists.txt`.

---

## Maintainer

Muhammed Kerem Kahraman (<keremkahraman53@gmail.com>)