#include "../../include/common/SwingLegController.h"

/******************************************************************************************************/
/******************************************************************************************************/

swingLegController::swingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.0);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::initSwingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    updateFootPosition();
    
    // TODO: Setting swing phase lift leg height
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.04);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
}

void swingLegController::updateGait(Gait* gait)
{
    this->gait = gait;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingLeg(){
    seResult = data->_stateEstimator->getResult();
    leftJointFb = data->_legController->data[0].q;
    rightJointFb = data->_legController->data[1].q;
    updateFootPosition();
    updateSwingStates();
    updateSwingTimes();
    computeFootPlacement();    
    computeFootDesiredPosition();
    // setDesiredJointState();
    setWBCDesiredJointState();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){    
    pFoot_w[i] =  seResult.position + seResult.rBody.transpose() 
                * (data->_legController->data[i].p); 
    }

    pFoot_w[0][2] = 0.0;
    pFoot_w[1][2] = 0.0;

    std::cout << "pFoot_w Left: " << pFoot_w[0].transpose() << std::endl;
    std::cout << "pFoot_w Right: " << pFoot_w[1].transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingStates(){
    swingStates = gait->getSwingSubPhase();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingTimes(){
    for(int leg = 0; leg < nLegs; leg++){
        if(firstSwing[leg]){
            swingTimes[leg] = _dtSwing * gait->_swing[nLegs-1];
        }else{
            swingTimes[leg] -= _dt;
            if(swingTimes[leg] <= 0){
                firstSwing[leg] = true;
            }
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootPlacement(){
    auto &stateCommand = data->_desiredStateCommand;

    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;

    v_des_world = seResult.rBody.transpose() * v_des_robot;    
    for(int foot = 0; foot < nLegs; foot++){
        // TODO: Setting swing phase lift leg height
        footSwingTrajectory[foot].setHeight(0.06);
        // there is offset from the CoM to truck
        Vec3<double> offset;
        offset << 0.14850 - 0.08, 0.0, 0.0;
        Vec3<double> Pf = seResult.position +
                seResult.rBody.transpose() * (data->_legController->data[foot].hip_p)
                + seResult.vWorld * swingTimes[foot] - offset;

        // Vec3<double> Pf = seResult.position +
        //         seResult.rBody.transpose() * (data->_legController->data[foot].hip_p)
        //         + v_des_world * swingTimes[foot];

        // std::cout << "v_des_world: " << v_des_world.transpose() << std::endl;

        // double pfx_offsetCoM = foot == 0 ? 0.0058 : -0.0008;
        double pfx_offsetCoM = foot == 0 ? 0 : 0;  

        // TODO: Adjusting footfall parameters
        double p_rel_max =  0.2;

        double pfx_rel   =  seResult.vWorld[0] * 0.5 * gait->_stance[nLegs-1] * _dtSwing +
                            0.1  * (seResult.vWorld[0] - v_des_world[0]) + pfx_offsetCoM;

        double pfy_rel   =  seResult.vWorld[1] * 0.5 * gait->_stance[nLegs-1] * _dtSwing +
                            0.1  * (seResult.vWorld[1] - v_des_world[1]);

        // double pfx_rel   =  v_des_world[0] * 0.5 * gait->_stance[nLegs-1] * _dtSwing;

        // double pfy_rel   =  v_des_world[1] * 0.5 * gait->_stance[nLegs-1] * _dtSwing;

        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel; 
        // fix foot position in the y axis
        Pf[1] = (seResult.position +
                seResult.rBody.transpose() * (data->_legController->data[foot].hip_p))[1];
        Pf[2] = 0; // -0.01

        footSwingTrajectory[foot].setFinalPosition(Pf);   
        std::cout << "pfx_rel: " << pfy_rel << ", pfy_rel: " << pfy_rel << std::endl;
        std::cout << "Pf: " << Pf.transpose() << std::endl;
    }
}


/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] > 0){
            if (firstSwing[foot]){
            //   std::cout << "firstSwing[" << foot << "] = "<< firstSwing[foot] << "\n";  
              firstSwing[foot] = false;
              footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
            }
            //Compute and get the desired foot position and velocity
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingStates[foot], 0.2); //FIX: second argument not used in function
            Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
            Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();
            
            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position);
            vFoot_b[foot] = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // fix foot position in the y axis
            // pFoot_b[foot](0) = 0;
            // vFoot_b[foot] = Vec3<double>::Zero();    

            // swing state of legs
            std::cout << "pDesFootWorld: " << pDesFootWorld.transpose() << std::endl;
            data->_robotState->swing_fe_pos_des_W = pDesFootWorld;
            data->_robotState->swing_fe_rpy_des_W << 0, 0, seResult.rpy(2);
        }
    }    
}

// 使用TRAC-IK计算逆运动学
void swingLegController::computeIK(const Vec3<double> &bodyPositionDesired, Vec4<double> &jointAnglesDes, 
                                    Vec4<double> &jointAnglesFb, int leg){          
    // Set the initial values of the joints
    KDL::JntArray nominal(4);
    nominal.data = jointAnglesFb;

    boost::posix_time::ptime start_time;
    boost::posix_time::time_duration diff;
    double total_time = 0;

    KDL::Frame end_effector_pose;
    KDL::JntArray result;
    int rc;

    // // Test trac_ik 
    // std::vector<KDL::JntArray> solutions;

    end_effector_pose.M = KDL::Rotation(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
    );

    end_effector_pose.p = KDL::Vector(bodyPositionDesired(0),bodyPositionDesired(1),bodyPositionDesired(2));

    std::cout << "end_effector_pose: x " << end_effector_pose.p[0] << " y " << end_effector_pose.p[1] << " z " << end_effector_pose.p[2] << std::endl;

    start_time = boost::posix_time::microsec_clock::local_time();
    if(leg == 0)
    {
        // rc表示解的数量
        rc = data->_legController->_biped._ik_solver_left.CartToJnt(nominal, end_effector_pose, result);

        // for (int i = 0; i < rc; i++)
        // {
        //     data->_legController->_biped._ik_solver_left.getSolutions(solutions);
           
        //     std::cout << "Foot" << leg << " solution " << i << ": " <<solutions[i].data.transpose() << std::endl;
        // }
    }
    else if(leg == 1)
    {
        rc = data->_legController->_biped._ik_solver_right.CartToJnt(nominal, end_effector_pose, result);

        // for (int i = 0; i < rc; i++)
        // {
        //     data->_legController->_biped._ik_solver_right.getSolutions(solutions);
           
        //     std::cout << "Foot" << leg << " solution " << i << ": " <<solutions[i].data.transpose() << std::endl;
        // }
    }
    
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    total_time = diff.total_nanoseconds() / 1e9;

    if(rc > 0)
    {
        // std::cout << "Foot" << leg << " rc " << rc << std::endl;
        std::cout<< "Foot"<< leg <<" TRAC-IK Joint " << result.data.transpose() <<std::endl;
        std::cout<< "Foot"<< leg <<" TRAC-IK Time " << total_time <<std::endl;

        jointAnglesDes = result.data;
    }
    else
    {
        jointAnglesDes = jointAnglesFb;
        std::cout <<"TRAC-IK Failed "<<std::endl;
    }

    std::cout << "jointAnglesDes " << jointAnglesDes.transpose() << std::endl;
    std::cout << "jointAnglesFb " << jointAnglesFb.transpose() << std::endl;

    data->_robotState->swing_joint_des_q = jointAnglesDes;
}

// /******************************************************************************************************/
// /******************************************************************************************************/

// 使用逆运动学，控制每个关节的PID
void swingLegController::setDesiredJointState(){
    Vec4<double> qFb;
    for(int leg = 0; leg < nLegs; leg++){
        if(leg == 0){
            qFb = leftJointFb;
        }
        else if(leg == 1){
            qFb = rightJointFb;
        }

        if(swingStates[leg] > 0){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, qFb, leg);
            data->_legController->commands[leg].qdDes = Vec4<double>::Zero();
            Eigen::VectorXd kpgains(4);
            kpgains << 500, 1000, 1000, 125;
            Eigen::VectorXd kdgains(4);
            kdgains << 1, 2, 2, 1;
             data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
             data->_legController->commands[leg].pDes = pFoot_b[leg];
             data->_legController->commands[leg].vDes = vFoot_b[leg];
             data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
             data->_legController->commands[leg].kdJoint = kdgains.asDiagonal();                      
        }else{
            //Ensure no interference with stance leg controller
            Eigen::VectorXd kpgains(4);
            Eigen::VectorXd kdgains(4);
            kpgains.setZero();
            kdgains.setZero();
            data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
            data->_legController->commands[leg].kdJoint = kdgains.asDiagonal(); 
            data->_legController->commands[leg].kpCartesian = Eigen::Matrix3d::Zero();
            data->_legController->commands[leg].kdCartesian = Eigen::Matrix3d::Zero();               
        }
    }
}

// 使用逆运动学，控制每个关节的PID
void swingLegController::setWBCDesiredJointState(){
    Vec4<double> qFb;
    for(int leg = 0; leg < nLegs; leg++){
        if(leg == 0){
            qFb = leftJointFb;
        }
        else if(leg == 1){
            qFb = rightJointFb;
        }

        if(swingStates[leg] > 0){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, qFb, leg);          
        }
    }
}

// 使用雅各比矩阵，控制足端的PID
// void swingLegController::setDesiredJointState()
// {
//     for(int leg = 0; leg < nLegs; leg++){
//         if(swingStates[leg] > 0){
//             data->_legController->commands[leg].qdDes = Vec4<double>::Zero();
//             Eigen::VectorXd kpgains(3);
//             // kpgains << 300, 300, 1500;
//             kpgains << 600, 600, 2000;
//             Eigen::VectorXd kdgains(3);
//             kdgains << 10, 10, 30;
//              data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
//              data->_legController->commands[leg].pDes = pFoot_b[leg];
//              data->_legController->commands[leg].vDes = vFoot_b[leg];
//              data->_legController->commands[leg].kpJoint = Mat4<double>::Zero();
//              data->_legController->commands[leg].kdJoint = Mat4<double>::Zero();
//              data->_legController->commands[leg].kpCartesian = kpgains.asDiagonal();
//              data->_legController->commands[leg].kdCartesian = kdgains.asDiagonal();                  
//         }else{
//             //Ensure no interference with stance leg controller
//             data->_legController->commands[leg].kpJoint = Mat4<double>::Zero();
//             data->_legController->commands[leg].kdJoint = Mat4<double>::Zero(); 
//             data->_legController->commands[leg].kpCartesian = Eigen::Matrix3d::Zero();
//             data->_legController->commands[leg].kdCartesian = Eigen::Matrix3d::Zero();          
//         }
//     }
// }
