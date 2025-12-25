//
// Created by boxing on 24-1-12.
//
#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "iomanip"

#include "StateEstimate.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"

class RobotState{
    public:
    const int model_nv; // number of dq

    // motors, sensors and states feedback
    double fL[3];
    double fR[3];
    Eigen::Vector3d basePos;
    Eigen::Vector3d baseLinVel; // velocity of the basePos
    Eigen::Vector3d baseAcc;   // baseAcc of the base link
    Eigen::Vector3d rpy;
    Eigen::Vector3d baseAngVel;  // angular velocity of the base link

    Eigen::VectorXd motors_pos_cur;
    Eigen::VectorXd motors_vel_cur;
    Eigen::VectorXd motors_tor_cur;
    Eigen::VectorXd FL_est, FR_est; // foot force estimation

    Eigen::VectorXd motors_pos_des;
    Eigen::VectorXd motors_vel_des;
    Eigen::VectorXd motors_tor_des;
    Eigen::VectorXd motors_tor_out;
    Eigen::VectorXd motors_Kp;
    Eigen::VectorXd motors_Kd;

    // states and key variables
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd q_fixed, dq_fixed, ddq_fixed;
    Eigen::VectorXd qOld;
    Eigen::MatrixXd J_base, J_l, J_r;
    Eigen::MatrixXd J_l_fixed, J_r_fixed;
    Eigen::MatrixXd dJ_base, dJ_l, dJ_r;
    // Eigen::MatrixXd J_hd_l, J_hd_r, dJ_hd_l, dJ_hd_r;
    Eigen::MatrixXd Jcom_W; // jacobian of CoM, in world frame
    Eigen::Vector3d pCoM_W;
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos;
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; // in world frame
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L; // in Body frame
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;
    Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;

    // Eigen::Vector3d hd_r_pos_W, hd_l_pos_W; // in world frame
    // Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W;
    // Eigen::Vector3d hd_r_pos_L, hd_l_pos_L; // in body frame
    // Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L;

    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_G, dyn_Non;
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy;

    Eigen::Matrix<double,3,3> inertia;

    enum MotionState{
        Stand,
        Walk,
        Other
    };
    enum LegState{
        LSt,
        RSt,
        NSt
        // DSt   // no use but reserverd
    };
    LegState legState;
    MotionState motionState;

    // intput values for WBC
    Eigen::Vector3d base_rpy_des;
    Eigen::Vector3d base_pos_des;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q; // des_q is useless
    Eigen::Vector3d swing_fe_pos_des_W;
    Eigen::Vector3d swing_fe_rpy_des_W;
    Eigen::VectorXd swing_joint_des_q;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::VectorXd Fr_ff;

    // output values for WBC
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;
    Eigen::VectorXd wbc_tauJointRes;
    Eigen::VectorXd wbc_FrRes;
    
    RobotState(int pino_nv):model_nv(pino_nv){
        motors_pos_cur = Eigen::VectorXd::Zero(model_nv-6);
        motors_vel_cur = Eigen::VectorXd::Zero(model_nv-6);
        motors_tor_cur = Eigen::VectorXd::Zero(model_nv-6);
        motors_pos_des = Eigen::VectorXd::Zero(model_nv-6);
        motors_vel_des = Eigen::VectorXd::Zero(model_nv-6);
        motors_tor_des = Eigen::VectorXd::Zero(model_nv-6);
        motors_tor_out = Eigen::VectorXd::Zero(model_nv-6);
        motors_Kp = Eigen::VectorXd::Zero(model_nv-6);
        motors_Kd = Eigen::VectorXd::Zero(model_nv-6);
        // 原始值
        // motors_Kp.block<4,1>(0,0) << 60, 250, 250, 60;
        // motors_Kp.block<4,1>(4,0) << 60, 250, 250, 60;
        // motors_Kd.block<4,1>(0,0) << 1, 1, 1, 1;
        // motors_Kd.block<4,1>(4,0) << 1, 1, 1, 1;
        // 刚度较大，阻尼较大
        // motors_Kp.block<4,1>(0,0) << 25, 250, 250, 25;
        // motors_Kp.block<4,1>(4,0) << 25, 250, 250, 25;
        motors_Kp.block<4,1>(0,0) << 200, 500, 500, 50;
        motors_Kp.block<4,1>(4,0) << 200, 500, 500, 50;
        motors_Kd.block<4,1>(0,0) << 10, 25, 25, 5;
        motors_Kd.block<4,1>(4,0) << 10, 25, 25, 5;
        // 刚度较小
        // motors_Kp.block<4,1>(0,0) << 6, 25, 25, 6;
        // motors_Kp.block<4,1>(4,0) << 6, 25, 25, 6;
        // motors_Kd.block<4,1>(0,0) << 1, 1, 1, 1;
        // motors_Kd.block<4,1>(4,0) << 1, 1, 1, 1;
        swing_joint_des_q = Eigen::VectorXd::Zero(4);
        q=Eigen::VectorXd::Zero(model_nv+1);
        qOld=Eigen::VectorXd::Zero(model_nv+1);
        dq=Eigen::VectorXd::Zero(model_nv);
        ddq=Eigen::VectorXd::Zero(model_nv);
        q_fixed=Eigen::VectorXd::Zero(model_nv-6);
        dq_fixed=Eigen::VectorXd::Zero(model_nv-6);
        ddq_fixed=Eigen::VectorXd::Zero(model_nv-6);
        FL_est=Eigen::VectorXd::Zero(6);
        FR_est=Eigen::VectorXd::Zero(6);
        Fr_ff = Eigen::VectorXd::Zero(12);
        des_ddq = Eigen::VectorXd::Zero(model_nv);
        des_dq = Eigen::VectorXd::Zero(model_nv);
        des_delta_q = Eigen::VectorXd::Zero(model_nv);
        base_rpy_des.setZero();
        base_pos_des.setZero();
    }

    void updateJointQ(LowlevelState* jointState)
    {
        // TODO: change q from motor angles to joint angles in real robot
        for (int i = 0; i < 8; i++)
        {
            motors_pos_cur[i] = jointState->motorState[i].q;
            motors_vel_cur[i] = jointState->motorState[i].dq;
            motors_tor_cur[i] = jointState->motorState[i].tauEst;
        }

        q_fixed = motors_pos_cur;
        dq_fixed = motors_vel_cur;
    }

    void updateJointTau_PDTest(LowlevelCmd* jointCmd)
    {
        float motors_pos_des[8] = { 
            0, -0.208657, 0.670694, -0.462037, 
            0,  0.208657, -0.670694, 0.462037 
        };
        
        float motors_vel_des[8] = { 0.0 };
        float motors_tor_des[8] = { 0.0 };

        for(int i = 0; i < 8; i++)
        {
            jointCmd->motorCmd[i].q = motors_pos_des[i];
            jointCmd->motorCmd[i].dq = motors_vel_des[i];
            jointCmd->motorCmd[i].tau = motors_tor_des[i];

            jointCmd->motorCmd[i].Kp = motors_Kp[i];
            jointCmd->motorCmd[i].Kd = motors_Kd[i];
        }
    }

    void updateJointTau(LowlevelCmd* jointCmd)
    {
        for(int i=0;i<8;i++)
        {
            jointCmd->motorCmd[i].q = motors_pos_des[i];
            jointCmd->motorCmd[i].dq = motors_vel_des[i];
            jointCmd->motorCmd[i].tau = motors_tor_des[i];

            jointCmd->motorCmd[i].Kp = motors_Kp[i];
            jointCmd->motorCmd[i].Kd = motors_Kd[i];
        }
    }

    // update q according to sensor values, must update sensor values before
    void updateQ(StateEstimate *baseEst){
        basePos = baseEst->position;
        baseLinVel = baseEst->vWorld;
        rpy = baseEst->rpy;
        baseAngVel = baseEst->omegaWorld;
        base_omega_L = baseEst->omegaBody;

        base_omega_W = baseAngVel;
        auto Rcur= eul2Rot(rpy[0], rpy[1], rpy[2]);

        //  q = [global_base_position, global_base_quaternion, joint_positions]
        //  dq = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]

        auto quatNow=eul2quat(rpy[0], rpy[1], rpy[2]);
        q.block(0,0,3,1) = basePos;
        q(3)=quatNow.x();
        q(4)=quatNow.y();
        q(5)=quatNow.z();
        q(6)=quatNow.w();
        for (int i=0;i<model_nv-6;i++)
            q(i+7)=motors_pos_cur[i];

        Eigen::Vector3d vCoM_W;
        vCoM_W = baseLinVel;
        dq.block<3,1>(0,0) = vCoM_W;
        dq.block<3,1>(3,0) = base_omega_W;

        for (int i=0;i<model_nv-6;i++)
        {
            dq(i+6)=motors_vel_cur[i];
        }

        base_pos = q.block(0,0,3,1);
        base_rpy = rpy;
        base_rot = Rcur;
        qOld=q;

        printq(q);
        printdq(dq);
    }


    static void printdq(const Eigen::VectorXd &q){
        std::cout<< "body dq: " <<std::setprecision(5)<<q.block<6,1>(0,0).transpose()<<std::endl;
        std::cout<< "left leg dq: " <<std::setprecision(5)<<q.block<4,1>(6,0).transpose()<<std::endl;
        std::cout<< "right leg dq: " <<std::setprecision(5)<<q.block<4,1>(10,0).transpose()<<std::endl;
    }

    static void printq(const Eigen::VectorXd &q){
        std::cout<< "body q: " <<std::setprecision(5)<<q.block<7,1>(0,0).transpose()<<std::endl;
        std::cout<< "left leg q: " <<std::setprecision(5)<<q.block<4,1>(7,0).transpose()<<std::endl;
        std::cout<< "right leg q: " <<std::setprecision(5)<<q.block<4,1>(11,0).transpose()<<std::endl;
    }

    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
        Eigen::Matrix<double,3,3> Rx,Ry,Rz;
        Rz<<cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0,0,1;
        Ry<<cos(pitch),0,sin(pitch),
                0,1,0,
                -sin(pitch),0,cos(pitch);
        Rx<<1,0,0,
                0,cos(roll),-sin(roll),
                0,sin(roll),cos(roll);
        return Rz*Ry*Rx;
    }

    Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw) {
        Eigen::Matrix3d R= eul2Rot(roll,pitch,yaw);
        Eigen::Quaternion<double> quatCur;
        quatCur = R; //rotation matrix converted to quaternion
        Eigen::Quaterniond resQuat;
        resQuat=quatCur;
        return resQuat;
    }
};

#endif


