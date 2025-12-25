/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#ifndef PINOCCHIO_KIN_DYN_H
#define PINOCCHIO_KIN_DYN_H

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include <string>
#include <vector>

#include "RobotState.h"

class Pin_KinDyn {
public:
    std::vector<bool> motorReachLimit;
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    Eigen::VectorXd tauJointOld;
    std::string urdf_path;
    pinocchio::Model model_biped;
    pinocchio::Model model_biped_fixed;
    int model_nv, model_nv_fixed;
    pinocchio::JointIndex r_ankle_joint, l_ankle_joint, base_joint, r_hip_joint, l_hip_joint;
    pinocchio::FrameIndex r_fe_frame, l_fe_frame;
    pinocchio::JointIndex r_ankle_joint_fixed, l_ankle_joint_fixed, r_hip_joint_fixed, l_hip_joint_fixed;
    // pinocchio::JointIndex r_hand_joint, l_hand_joint,r_hand_joint_fixed, l_hand_joint_fixed, waist_yaw_joint;
    pinocchio::FrameIndex r_fe_frame_fixed, l_fe_frame_fixed, r_hip_frame_fixed, l_hip_frame_fixed;
    Eigen::VectorXd q,dq,ddq;
    Eigen::Matrix3d Rcur;
    Eigen::Quaternion<double> quatCur;
    Eigen::Matrix<double,6,-1> J_r, J_l, J_r_fe, J_l_fe, J_base;
    Eigen::Matrix<double,6,-1> dJ_r,dJ_l, dJ_r_fe, dJ_l_fe, dJ_base;
    // Eigen::Matrix<double,6,-1> J_hd_r, J_hd_l, dJ_hd_r, dJ_hd_l;
    Eigen::Matrix<double,3,-1> Jcom;
    Eigen::Vector3d fe_r_pos, fe_l_pos, base_pos;    // foot-end position in world frame
    // Eigen::Vector3d hd_r_pos, hd_l_pos;  // hand position in world frame
    Eigen::Vector3d hip_r_pos, hip_l_pos;
    Eigen::Matrix3d fe_r_rot, fe_l_rot, base_rot;
    // Eigen::Matrix3d hd_r_rot, hd_l_rot;
    
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;

    Eigen::VectorXd q_fixed,dq_fixed,ddq_fixed;
    Eigen::Matrix<double,6,-1> J_r_fe_fixed, J_l_fe_fixed; // foot-end Jacobian in fixed frame
    Eigen::Vector3d fe_r_pos_body, fe_l_pos_body;  // foot-end position in body frame
    // Eigen::Vector3d hd_r_pos_body, hd_l_pos_body; // hand position in body frame
    Eigen::Vector3d hip_r_pos_body, hip_l_pos_body;
    Eigen::Matrix3d fe_r_rot_body, fe_l_rot_body;
    // Eigen::Matrix3d hd_r_rot_body, hd_l_rot_body;

    Pin_KinDyn(std::string urdf_pathIn);
    void runFloat(RobotState &robotState);
    void dataBusRead_Float(RobotState const &robotState);
    void dataBusWrite_Float(RobotState &robotState);
    void computeJdJ_Float();
    void computeDyn_Float();

    void runFixed(RobotState &robotState);
    void dataBusRead_Fixed(RobotState const &robotState);
    void dataBusWrite_Fixed(RobotState &robotState);
    void computeJ_Fixed();

    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double,3,1> &w);
private:
    pinocchio::Data data_biped, data_biped_fixed;

};

#endif