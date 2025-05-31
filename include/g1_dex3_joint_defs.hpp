#pragma once
#include <string>
#include <map>

// Hand joint indices
enum HandJointIndex {
    kThumb0,
    kThumb1,
    kThumb2,
    kMiddle0,
    kMiddle1,
    kIndex0,
    kIndex1,
};

// Main robot joint indices
enum JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,
    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,
    kWaistYaw,
    kWaistRoll,
    kWaistPitch,
    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,
    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

// Hand joint name to index map
const std::map<std::string, HandJointIndex> hand_joint_name_to_index = {
    {"left_hand_thumb_0_joint", kThumb0},
    {"left_hand_thumb_1_joint", kThumb1},
    {"left_hand_thumb_2_joint", kThumb2},
    {"left_hand_middle_0_joint", kMiddle0},
    {"left_hand_middle_1_joint", kMiddle1},
    {"left_hand_index_0_joint", kIndex0},
    {"left_hand_index_1_joint", kIndex1},
    {"right_hand_thumb_0_joint", kThumb0},
    {"right_hand_thumb_1_joint", kThumb1},
    {"right_hand_thumb_2_joint", kThumb2},
    {"right_hand_middle_0_joint", kMiddle0},
    {"right_hand_middle_1_joint", kMiddle1},
    {"right_hand_index_0_joint", kIndex0},
    {"right_hand_index_1_joint", kIndex1}
};

// Main robot joint name to index map
const std::map<std::string, JointIndex> joint_name_to_index = {
    {"left_hip_pitch_joint", kLeftHipPitch},
    {"left_hip_roll_joint", kLeftHipRoll},
    {"left_hip_yaw_joint", kLeftHipYaw},
    {"left_knee_joint", kLeftKnee},
    {"left_ankle_pitch_joint", kLeftAnkle},
    {"left_ankle_roll_joint", kLeftAnkleRoll},
    {"right_hip_pitch_joint", kRightHipPitch},
    {"right_hip_roll_joint", kRightHipRoll},
    {"right_hip_yaw_joint", kRightHipYaw},
    {"right_knee_joint", kRightKnee},
    {"right_ankle_pitch_joint", kRightAnkle},
    {"right_ankle_roll_joint", kRightAnkleRoll},
    {"waist_yaw_joint", kWaistYaw},
    {"waist_roll_joint", kWaistRoll},
    {"waist_pitch_joint", kWaistPitch},
    {"left_shoulder_pitch_joint", kLeftShoulderPitch},
    {"left_shoulder_roll_joint", kLeftShoulderRoll},
    {"left_shoulder_yaw_joint", kLeftShoulderYaw},
    {"left_elbow_joint", kLeftElbow},
    {"left_wrist_roll_joint", kLeftWristRoll},
    {"left_wrist_pitch_joint", kLeftWristPitch},
    {"left_wrist_yaw_joint", kLeftWristYaw},
    {"right_shoulder_pitch_joint", kRightShoulderPitch},
    {"right_shoulder_roll_joint", kRightShoulderRoll},
    {"right_shoulder_yaw_joint", kRightShoulderYaw},
    {"right_elbow_joint", kRightElbow},
    {"right_wrist_roll_joint", kRightWristRoll},
    {"right_wrist_pitch_joint", kRightWristPitch},
    {"right_wrist_yaw_joint", kRightWristYaw}
};