#ifndef MPCLOCOMOTION_H
#define MPCLOCOMOTION_H

#include <eigen3/Eigen/Dense>
#include "../include/common/SwingLegController.h"
#include "../include/common/FootSwingTrajectory.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/cppTypes.h"
#include "GaitGenerator.h"
#include <fstream>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;
using Eigen::Array2d;
using Eigen::Array2i;
using Eigen::Array2f;

using namespace std;

/**
 * @file MPCLocomotion.h
 * @brief Convex Model Predictive Control (MPC) for Bipedal Locomotion
 *
 * This file defines the MPCLocomotion class, which implements a convex MPC
 * approach to generate and control gait patterns for bipedal robots. The class
 * offers functionality to set gait patterns, update the MPC as needed, and track 
 * the state of the robot in terms of position, orientation, and contact with the ground.
 */


struct CMPC_Result {
  LegControllerCommand commands[2];
  Vec2<float> contactPhase;
};

class MPCLocomotion {
public:
    swingLegController swing;

    // Constructors
    MPCLocomotion(double _dt, int _iterations_between_mpc);
  
    // Main Functionalities
    void run(ControlFSMData& data);
    void setGaitNum(int gaitNum) { gaitNumber = gaitNum; }
    bool firstRun = true;

    void dataBusWrite(RobotState &robotState);
private:
    void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data);
    void GenerateTrajectory(int* mpcTable, ControlFSMData& data);

    // Locomotion and Gait Parameters
    int iterationsBetweenMPC;
    int horizonLength;
    double dt;
    double dtMPC;
    int iterationCounter = 0;
    Vec6<double> f_ff[2];
    // test
    Vec6<double> f_ff_pre[2];
    Vec6<double> f_ff_test[2];
    Vec12<double> Forces_Sol;
    Vec2<double> swingTimes;
    FootSwingTrajectory<double> footSwingTrajectories[2];
    Gait walking, standing;
    Gait walking_left,walking_right;

    // Feedback and Control Variables
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstSwing[2] = {true, true};
    double swingTimeRemaining[2];
    double stand_traj[6];
    int current_gait;
    int next_gait;
    int gaitNumber;
    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    Vec3<double> pFoot[2];
    CMPC_Result result;
    double trajAll[12*10];
    Mat43<double> W;  
    Vec3<double> a;  
    Vec4<double> pz;
    double ground_pitch;

    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;
    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;
    Vec3<double> pFoot_des[2];
    Vec3<double> vFoot_des[2];
    Vec3<double> aFoot_des[2];
    Vec3<double> Fr_des[2];
    Vec2<double> contact_state;
    Vec3<double> v_des_robot;
    bool climb = 0;
    ofstream foot_position;
    Vec3<double> ori_des_world;   

    Vec2<double> contactStates;
    Vec2<double> swingStates;
    Vec12<double> Fr_ff_W;
    Vec12<double> dX_cal;

    bool if_yaw_stable = false;
    double yaw_init = 0;
};


#endif //MPCLOCOMOTION_H
