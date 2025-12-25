#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>
#include <ros/ros.h>

#include <iostream>
#include <fstream>

// upper level of joint controller 
// send data to joint controller

std::ofstream  outfile_lpf;
std::ofstream  outfile_normal;

void LegControllerCommand::zero(){
    tau = Vec4<double>::Zero();
    qDes = Vec4<double>::Zero();
    qdDes = Vec4<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat4<double>::Zero();
    kdJoint = Mat4<double>::Zero();
}

/*
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec4<double>::Zero();
    qd = Vec4<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat64<double>::Zero();
    J_force = Mat34<double>::Zero();
    tau = Vec4<double>::Zero();
}

LegControllerData::LegControllerData() 
{
    zero();

    // filter initialization
    for (int i = 0; i < 4; i++)
    {
        qd_filter[i] = LowPassFilter(200, 0.001);
    }   
    qd_filter[1] = LowPassFilter(100, 0.001); // thigh
    qd_filter[3] = LowPassFilter(100, 0.001); // toe
}

LegController::LegController(Biped& biped) : 
_biped(biped)
{
    for(int i = 0; i < 2; i++){
        commands[i].zero();
        data[i].zero();
    }

    // outfile_lpf.open("data_process/file_lpf.txt");
    // outfile_normal.open("data_process/file_normal.txt");
}

void LegController::zeroCommand(){
    for (int i = 0; i < 2; i++){
        commands[i].zero();
    }
}

// TODO: LowlevelState写一个updateData函数，将反馈从电机转到关节
void LegController::updateData(const RobotState* state){
    for (int leg = 0; leg < 2; leg++){
        data[leg].q = state->motors_pos_cur.block(4*leg,0,4,1);
        data[leg].qd = state->motors_vel_cur.block(4*leg,0,4,1);
        data[leg].tau = state->motors_tor_cur.block(4*leg,0,4,1);
        
        for(int j = 0; j < 4; j++){
            data[leg].qd_f(j) = data[leg].qd_filter[j].applyFilter(data[leg].qd(j));
        }
    }
    // outfile_normal << data[0].qd.transpose() << std::endl;
    // outfile_lpf << data[0].qd_f.transpose() << std::endl;

    data[0].p = state->fe_l_pos_L;
    data[0].R_foot = state->fe_l_rot_L.cast<float>();
    data[0].hip_p = state->hip_l_pos_L;
    data[0].J_force_moment = state->J_l_fixed.block(0,0,6,4);
    data[0].J_force = state->J_l_fixed.block(0,0,3,4);

    data[1].p = state->fe_r_pos_L;
    data[1].R_foot = state->fe_r_rot_L.cast<float>();
    data[1].hip_p = state->hip_r_pos_L;
    data[1].J_force_moment = state->J_r_fixed.block(0,4,6,4);
    data[1].J_force = state->J_r_fixed.block(0,4,3,4);
    
    for (int leg = 0; leg < 2; leg++){
        data[leg].v = data[leg].J_force * data[leg].qd;
        data[leg].v_f = data[leg].J_force * data[leg].qd_f;
        // std::cout << "data[leg].v: "<< data[leg].v.transpose() << std::endl;
    }
}

void LegController::updateData(const LowlevelState* state){
    for (int leg = 0; leg < 2; leg++){
        for(int j = 0; j < 4; j++){
            data[leg].q(j) = state->motorState[leg*4+j].q;
            data[leg].qd(j) = state->motorState[leg*4+j].dq;
            data[leg].tau(j) = state->motorState[leg*4+j].tauEst;
            // std::cout << "motor joint data" << leg*4+j << ": "<< data[leg].q(j) << std::endl;

            data[leg].qd_f(j) = data[leg].qd_filter[j].applyFilter(data[leg].qd(j));
        }
    }
    // outfile_normal << data[0].qd.transpose() << std::endl;
    // outfile_lpf << data[0].qd_f.transpose() << std::endl;

    // std::cout << "computeLegJacobianAndPosition()" << std::endl;
    computeLegJacobianAndPosition();
    // std::cout << "computeLegJacobianAndPosition() end" << std::endl;
    
    for (int leg = 0; leg < 2; leg++){
        data[leg].v = data[leg].J_force * data[leg].qd;
        data[leg].v_f = data[leg].J_force * data[leg].qd_f;
        // std::cout << "data[leg].v: "<< data[leg].v.transpose() << std::endl;
    }
}

void LegController::updateCommand(RobotState* state, Pin_KinDyn* pino){
    Eigen::VectorXd pos_des = pino->integrateDIY(state->q, state->wbc_delta_q_final);
    state->motors_pos_des = pos_des.block(7, 0, state->model_nv - 6, 1);
    state->motors_vel_des = state->wbc_dq_final;
    state->motors_tor_des = state->wbc_tauJointRes;

    std::cout << "L_motors_pos_des: "<< state->motors_pos_des.block(0,0,4,1).transpose() << std::endl;
    std::cout << "L_motors_vel_des: "<< state->motors_vel_des.block(0,0,4,1).transpose() << std::endl;
    std::cout << "L_motors_tor_des: "<< state->motors_tor_des.block(0,0,4,1).transpose() << std::endl;
    std::cout << "R_motors_pos_des: "<< state->motors_pos_des.block(4,0,4,1).transpose() << std::endl;
    std::cout << "R_motors_vel_des: "<< state->motors_vel_des.block(4,0,4,1).transpose() << std::endl;
    std::cout << "R_motors_tor_des: "<< state->motors_tor_des.block(4,0,4,1).transpose() << std::endl;
}

void LegController::updateCommand(LowlevelCmd* cmd){

    for (int i = 0; i < 2; i++){
        Vec6<double> footForce = commands[i].feedforwardForce;
        Vec4<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg
        std::cout<<"footForce: "<< footForce(0)<<" "<< footForce(1)<<" "<< footForce(2)<<" "<< footForce(3)<<std::endl;

        // for(int j = 0; j < 4; j++){
        //     std::cout << "legtau" << j << ": "<< legtau(j) << std::endl;
        // }
        // std::cout << "legtau: "<< legtau.transpose() << std::endl;

        // TODO: 摆动相实现一个笛卡尔坐标系下的PD控制
        // cartesian PD control for swing foot

        if(commands[i].kpCartesian(0,0) != 0 || commands[i].kdCartesian(0,0) != 0)
        {
            // std::cout << "pDes " << commands[i].pDes.transpose() << std::endl
            //     << "p " << data[i].p.transpose() << std::endl;  
            // std::cout << "vDes " << commands[i].vDes.transpose() << std::endl
            //     << "v " << data[i].v.transpose() << std::endl;

            Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                        commands[i].kdCartesian * (commands[i].vDes - data[i].v);
            // std::cout<< "SwingLeg_id: "<< i << std::endl;
            std::cout<<"footForce_3d: "<< footForce_3d.transpose() <<std::endl;

            Vec4<double> swingtau = data[i].J_force.transpose() * footForce_3d ;
           
            // TODO: 六自由度的关节约束关系需要修改————使脚掌平行于地面，避免奇异解
            // maintain hip2 angle tracking
            swingtau(1) = commands[i].kphip1*(0-data[i].q(1)) + commands[i].kdhip1*(0-data[i].qd(1));
            // make sure foot is parallel with the ground
            swingtau(3) = i == 0 ? commands[i].kptoe * (0-data[i].q(1)-data[i].q(2)-data[i].q(3))+commands[i].kdtoe*(0-data[i].qd(3)) :
                                    commands[i].kptoe * (0-data[i].q(1)-data[i].q(2)-data[i].q(3))+commands[i].kdtoe*(0-data[i].qd(3)) ;

            for(int j = 0; j < 4; j++)
            {
                legtau(j) += swingtau(j);
            }
        }

        std::cout << "legtau: "<< legtau.transpose() << std::endl;

        commands[i].tau += legtau;
        // commands[i].tau += data[i].tau_ff;

        for (int j = 0; j < 4; j++){
            cmd->motorCmd[i*4+j].tau = commands[i].tau(j);
            cmd->motorCmd[i*4+j].q = commands[i].qDes(j);
            cmd->motorCmd[i*4+j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i*4+j].Kp = commands[i].kpJoint(j,j);
            cmd->motorCmd[i*4+j].Kd = commands[i].kdJoint(j,j);
            // std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " << cmd->motorCmd[i*5+j].tau << std::endl;            
        }

        commands[i].tau << 0, 0, 0, 0; // zero torque command to prevent interference
    }
    //std::cout << "cmd sent" << std::endl;
}

void LegController::checkLegJointCount(void){
    for (int i = 0; i < 2; i++)
    {
        if (data[i].q.rows() != 4 || data[i].qd.rows() != 4)
        {
            throw std::invalid_argument("Invalid leg count");
        } 
    }
}

// TODO: 计算雅各比矩阵和足端正向运动学
void LegController::computeLegJacobianAndPosition(void)
{
    // 正向运动学
    // 1. 使用forwardKinematics，计算正向动力学
    // 2. 使用updateFramePlacements，更新frame的值
    // 3. 使用oMf，读取frame在世界坐标系下的位置

    // 雅各比矩阵
    // 使用getFrameJacobian

    checkLegJointCount();

    pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED;
    Vec12<double> q_all, dq_all, ddq_all;
    pinocchio::Data::Matrix6x J(6,_biped._model.nv);
    J.setZero();
    q_all << data[0].q, data[1].q;

    pinocchio::forwardKinematics(_biped._model,_biped._data,q_all);
    pinocchio::updateFramePlacements(_biped._model,_biped._data);

    pinocchio::rnea(_biped._model,_biped._data, q_all, dq_all, ddq_all);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            data[i].tau_ff(j) = _biped._data.tau(i*4+j);
        }
    }

    // std::cout << "-------------------frames pos-------------------" << std::endl;
    auto frames = _biped._model.frames;
    for(pinocchio::FrameIndex id = 0; id < _biped._model.nframes; ++id)
    {
        // std::cout << _model.frames[id].name << std::endl;
        // std::cout << _data.oMf[id].translation().transpose() << std::endl;
        if ( _biped._model.frames[id].name == "L_foot")
        {
            J.setZero();
            data[0].p = _biped._data.oMf[id].translation();
            data[0].R_foot = _biped._data.oMf[id].toHomogeneousMatrix().block(0,0,3,3).cast<float>();
            std::cout << "L_foot pos: " << data[0].p.transpose() << std::endl;
            // std::cout << "L_foot R: " << std::endl << data[0].R_foot << std::endl;
            pinocchio::computeFrameJacobian(_biped._model, _biped._data, q_all, id, rf, J);
            data[0].J_force_moment = J.block(0,0,6,4);
            data[0].J_force = J.block(0,0,3,4);
        }
        else if ( _biped._model.frames[id].name == "R_foot")
        {
            J.setZero();
            data[1].p = _biped._data.oMf[id].translation();
            data[1].R_foot = _biped._data.oMf[id].toHomogeneousMatrix().block(0,0,3,3).cast<float>();
            std::cout << "R_foot pos: " << data[1].p.transpose() << std::endl;
            // std::cout << "R_foot R: " << std::endl << data[1].R_foot << std::endl;
            pinocchio::computeFrameJacobian(_biped._model, _biped._data, q_all, id, rf, J);
            data[1].J_force_moment = J.block(0,4,6,4);
            data[1].J_force = J.block(0,4,3,4);
        }
        else if (_biped._model.frames[id].name == "L_hip2")
        {
            data[0].hip_p = _biped._data.oMf[id].translation();
            std::cout << "L_hip2 pos: " << data[0].hip_p.transpose() << std::endl;
        }
        else if (_biped._model.frames[id].name == "R_hip2")
        {
            data[1].hip_p = _biped._data.oMf[id].translation();
            std::cout << "R_hip2 pos: " << data[1].hip_p.transpose() << std::endl;
        }
    }
}