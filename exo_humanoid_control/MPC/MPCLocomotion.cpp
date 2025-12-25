#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "MPCLocomotion.h"
#include "RobotStateMpc.h"

using namespace ori;
using Eigen::Dynamic;


/* =========================== Controller ============================= */
MPCLocomotion::MPCLocomotion(double _dt, int _iterations_between_mpc) : 
 iterationsBetweenMPC(_iterations_between_mpc),
 horizonLength(10),
 dt(_dt),
 walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),
//  walking(horizonLength, Vec2<int>(0, 3), Vec2<int>(7, 7), "Walking"),
//  walking(horizonLength, Vec2<int>(0, 2), Vec2<int>(8, 8), "Walking"),
 walking_left(horizonLength, Vec2<int>(0, 0), Vec2<int>(7, 10), "Walking_left"),
 walking_right(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 7), "Walking_right"),
 standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  gaitNumber = 1;
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;

  // foot_position.open("foot_pos.txt");
}

/******************************************************************************************************/
/******************************************************************************************************/

void MPCLocomotion::run(ControlFSMData &data)
{
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  Gait *gait = &standing;
  Gait *gait_new = &standing;
  if (gaitNumber == 1)
  {
    gait = &standing; 
    gait_new = &standing;
  }
  else if (gaitNumber == 2)
  {
    gait = &walking;
    gait_new = &walking;
  }
  else if (gaitNumber == 3)
  {
    gait = &walking_left;
    gait_new = &walking_right;
  }
  else if (gaitNumber == 4)
  {
    gait = &walking_right;
    gait_new = &walking_left;
  }
  swing.updateGait(gait);

  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];

  world_position_desired[2] = 0.90;

  std::cout << "world_position_desired: " << world_position_desired.transpose() << std::endl;

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
    * (data._legController->data[i].p);
    // std::cout << "pFoot " << i << ": " << pFoot[i] << std::endl;
  }
  // std::cout << "RBody: " << seResult.rBody << std::endl;

  // some first time initialization
  if (firstRun)
  {
    swing.initSwingLegController(&data, gait, dtMPC);
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    if (gaitNumber == 1)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];

      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    firstRun = false;
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter); 

  // gait
  contactStates = gait->getContactSubPhase();
  swingStates = gait->getSwingSubPhase();

  // int *mpcTable = gait->mpc_gait();
  int *mpcTable = gait->mpc_gait(gait_new);

  swing.updateSwingLeg(); // update swing leg controller
  updateMPCIfNeeded(mpcTable, data);

  // Vec6<double> footForce;
  // Vec6<double> footMoment;
  // data._mpcinterface->recvMpcOutput(footForce, footMoment);
  // //Get solution and update foot forces    
  // for (int leg = 0; leg < 2; leg++)
  // {
  //   Vec3<double> GRF;
  //   Vec3<double> GRF_R;
  //   Vec3<double> GRM;
  //   Vec3<double> GRM_R;
  //   Vec6<double> f;

  //   GRF = footForce.block(leg * 3, 0, 3, 1);
  //   GRM = footMoment.block(leg * 3, 0, 3, 1);

  //   GRF_R = - seResult.rBody * GRF;
  //   GRM_R = - seResult.rBody * GRM;
  //   // std::cout << "RBody: " << seResult.rBody << std::endl;

  //   f.block(0, 0, 3, 1) = GRF_R;
  //   f.block(3, 0, 3, 1) = GRM_R;

  //   f_ff[leg] = f;
  // }

  iterationCounter++;

  Vec2<double> se_contactState(0, 0);

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot); 
    std::cout << "swing " << foot << ": " << swingState << std::endl;
    std::cout << "Contact " << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {
      se_contactState[foot] = contactState;
      data._stateEstimator->setResult_contactEstimate(foot, 0);
    }
    else if (contactState > 0) // foot is in stance
    { 
      // data._legController->commands[foot].feedforwardForce = f_ff_test[foot];
      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      se_contactState[foot] = contactState;
      data._stateEstimator->setResult_contactEstimate(foot, contactState);
    }
  }

  dataBusWrite(*data._robotState);
  
  // std::cout << "gaitNumber "<< gaitNumber << std::endl;
 
  if (iterationCounter % (horizonLength * iterationsBetweenMPC) == 0)
  {
    // std::cout << "gaitNumber !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl;
    if (gaitNumber == 3)
    {
      // std::cout << "gaitNumber 444444444444444444444444444444444444444444444444"<< std::endl;
      gaitNumber = 4;
    }
    else if (gaitNumber == 4)
    {
      // std::cout << "gaitNumber 333333333333333333333333333333333333333333333333"<< std::endl;
      gaitNumber = 3;
    }
  }  
}

void MPCLocomotion::dataBusWrite(RobotState &robotState)
{
  // contact state of legs
  if(contactStates[0] > 0)
  {
    robotState.legState = RobotState::LegState::LSt;
    robotState.stance_fe_pos_cur_W = robotState.fe_l_pos_W;
    robotState.stance_fe_rot_cur_W = robotState.fe_l_rot_W;
  }
  else if(contactStates[1] > 0)
  {
    robotState.legState = RobotState::LegState::RSt;
    robotState.stance_fe_pos_cur_W = robotState.fe_r_pos_W;
    robotState.stance_fe_rot_cur_W = robotState.fe_r_rot_W;
  }
  else{
    robotState.legState = RobotState::LegState::NSt;
    std::cout << "Error: No foot in contact" << std::endl;
  }

  // des commands
  for (int i = 0; i < 3; i++)
  {
    robotState.base_rpy_des(i) = trajAll[i+12];
    robotState.base_pos_des(i) = trajAll[i+3+12];

    // robotState.base_rpy_des(i) = 0;
    // robotState.base_pos_des(i) = world_position_desired(i);
  }
  robotState.base_rpy_des(2) = trajAll[2+12];

  // des ddq dq delta_q
  double k = 5;
  robotState.des_ddq.block<2, 1>(0, 0) << dX_cal(9), dX_cal(10);
  robotState.des_ddq(5) = k * (trajAll[8] - robotState.dq(5));

  robotState.des_dq.block<3, 1>(0, 0) << trajAll[9], trajAll[10], trajAll[11]; // lin_vel
  robotState.des_dq.block<2, 1>(3, 0) << 0.0, 0.0;
  robotState.des_dq(5) = trajAll[8];

  robotState.des_delta_q.block<2, 1>(0, 0) = robotState.des_dq.block<2, 1>(0, 0) * dt;
  robotState.des_delta_q(5) = robotState.des_dq(5) * dt;

  // foot force
  robotState.Fr_ff = Fr_ff_W;
}

void MPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data)
{

  if ((iterationCounter % 4) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *quat = seResult.orientation.data();

    //Joint angles to compute foot rotation
    Eigen::Matrix<double, 8, 1> q;
    for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 4; k++)
      {
        q(i * 4 + k) = data._legController->data[i].q(k);
      }
    }

    double *joint_angles = q.data();

    Mat3<float> R_foot[2];
    for (int i = 0; i < 2; i++)
    {
      R_foot[i] = data._legController->data[i].R_foot;
    }

    double PI = 3.14159265359;
    // //Joint angles offset correction
    // q(2) +=  0.3*PI;
    // q(3) -=  0.6*PI;
    // q(4) +=  0.3*PI;

    // q(7) +=  0.3*PI;
    // q(8) -=  0.6*PI;
    // q(9) +=  0.3*PI;

    double PI2 = 2*PI;
    for(int i = 0; i < 8; i++){
      q(i) = fmod(q(i) , PI2);
    }

    // std::cout << "pFoot Left: " << pFoot[0].transpose() << std::endl;
    // std::cout << "pFoot Right: " << pFoot[1].transpose() << std::endl;
    // std::cout << "seResult.position" << seResult.position.transpose() << std::endl;

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }

    double *weights;
    double *Alpha_K;

    // // weight of standing
    // double Q_s[12] = {100, 200, 250,  2000, 500, 2000,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    // double Alpha_s[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    // // weight of walking
    // double Q_w[12] = {100, 600, 250,  5000, 200, 500,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    // double Alpha_w[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    // if (mpcTable[0] + mpcTable[1] == 2)
    // {
    //   weights = Q_s;
    //   Alpha_K = Alpha_s;
    //   std::cout << "standing" << std::endl;
    // }
    // else
    // {
    //   weights = Q_w;
    //   Alpha_K = Alpha_w;
    //   std::cout << "walking" << std::endl;
    // }

    //MPC Weights 
    // weight of test
    // double Q[12] = {100, 1000, 250,  2000, 500, 2000,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    // weight of standing
    // double Q[12] = {500, 1000, 250,  3000, 1000, 1000,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    // useless
    // double Q[12] = {500, 1000, 250,  3000, 1000, 2000,  1, 1, 1,  1, 1, 1}; 
    // double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    // weights = Q;
    // Alpha_K = Alpha;

    double yaw = seResult.rpy[2];

    std::cout << "current position: " << p[0] << "  "<< p[1] << "  "<< p[2] << std::endl;


    v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    const double max_pos_error = .05;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];
    double zStart = world_position_desired[2];

    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;
    std::cout << "desired position: " << world_position_desired.transpose()<< std::endl;

    // Vec3<double> ori_des_world;
    ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5]; 

    double wz_des = stateCommand->data.stateDes[11];   

    double trajInitial[12] = {/*rpy_comp[0] + */stateCommand->data.stateDes[3],  // 0
                              /*rpy_comp[1] + */stateCommand->data.stateDes[4],    // 1
                              yaw_init,    // 2
                              xStart,                                   // 3
                              yStart,                                   // 4
                              zStart ,   // 5
                              0,                                        // 6
                              0,                                        // 7
                              stateCommand->data.stateDes[11],  // 8
                              v_des_world[0],                           // 9
                              v_des_world[1],                           // 10
                              0};                                       // 11

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      // if(i == 0) // start at current position  TODO consider not doing this
      // {
      //   trajAll[0] = seResult.rpy[0];
      //   trajAll[1] = seResult.rpy[1];
      //   trajAll[2] = seResult.rpy[2];
      //   trajAll[3] = seResult.position[0];
      //   trajAll[4] = seResult.position[1];
      //   trajAll[5] = seResult.position[2];
      // }
      // else
      // {
        if (v_des_world[0] == 0) {
        trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else{
         trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
        }
        if (v_des_world[1] == 0) {
        trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else{
         trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
        }
        if (stateCommand->data.stateDes[11] == 0){
         trajAll[12*i + 2] = trajInitial[2];
         if (!if_yaw_stable){
          if_yaw_stable = true;
          yaw_init = yaw;
          trajAll[12*i + 2] = yaw_init;
         }
        }
        else{
        if_yaw_stable = false;
        trajAll[12*i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
        }
      // }

      // std::cout << "yaw traj" << i << ": " <<  trajAll[12*i + 2] << std::endl;

      // std::cout << "traj " << i << std::endl;
      // for (int j = 0; j < 12; j++) {
      //   std::cout << trajAll[12 * i + j] << "  ";
      // }
      // std::cout<< " " <<std::endl;

    }

    data._mpc->_update_data->update_problem_data(p, v, quat, w, r, yaw, trajAll, mpcTable, R_foot, 
                                          data._robotState->pCoM_W.cast<float>(), data._robotState->inertia.cast<float>());

    data._mpc->setup_problem(dtMPC, horizonLength, 0.25, 4000);
    
    Vec6<double> footForce;
    Vec6<double> footMoment;
    //Solve MPC
    Timer t_mpc_solve;
    t_mpc_solve.start();
    data._mpc->solveMpcForFootForces_multithread(footForce, footMoment, dX_cal);
    printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());
    //Get solution and update foot forces    
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;

      GRF = footForce.block(leg * 3, 0, 3, 1);
      GRM = footMoment.block(leg * 3, 0, 3, 1);

      Fr_ff_W.block(leg * 6, 0, 3, 1) = GRF;
      Fr_ff_W.block(leg * 6 + 3, 0, 3, 1) = GRM;

      GRF_R = - seResult.rBody * GRF;
      GRM_R = - seResult.rBody * GRM;
      // std::cout << "RBody: " << seResult.rBody << std::endl;

      f.block(0, 0, 3, 1) = GRF_R;
      f.block(3, 0, 3, 1) = GRM_R;

      f_ff_pre[leg] = f_ff[leg];
      f_ff[leg] = f;
    }
  }

  // if ((iterationCounter % 4) < 2)
  // {
  //     f_ff_test[0] = f_ff_pre[0];
  //     f_ff_test[1] = f_ff_pre[1];
  // }
  // else
  // {
  //     f_ff_test[0] = f_ff[0];
  //     f_ff_test[1] = f_ff[1];
  // }
  
}