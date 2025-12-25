/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "../../include/common/pino_kin_dyn.h"

#include <utility>

Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn) {
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_pathIn,root_joint,model_biped);
    pinocchio::urdf::buildModel(urdf_pathIn,model_biped_fixed);
    data_biped=pinocchio::Data(model_biped);
    data_biped_fixed=pinocchio::Data(model_biped_fixed);

    model_nv=model_biped.nv;
    J_l=Eigen::MatrixXd::Zero(6,model_nv);
    J_r=Eigen::MatrixXd::Zero(6,model_nv);
    J_l_fe=Eigen::MatrixXd::Zero(6,model_nv);
    J_r_fe=Eigen::MatrixXd::Zero(6,model_nv);
    // J_hd_l=Eigen::MatrixXd::Zero(6,model_nv);
    // J_hd_r=Eigen::MatrixXd::Zero(6,model_nv);
    J_base=Eigen::MatrixXd::Zero(6,model_nv);
    dJ_l=Eigen::MatrixXd::Zero(6,model_nv);
    dJ_r=Eigen::MatrixXd::Zero(6,model_nv);
    dJ_l_fe=Eigen::MatrixXd::Zero(6,model_nv);
    dJ_r_fe=Eigen::MatrixXd::Zero(6,model_nv);
    // dJ_hd_l=Eigen::MatrixXd::Zero(6,model_nv);
    // dJ_hd_r=Eigen::MatrixXd::Zero(6,model_nv);
    dJ_base=Eigen::MatrixXd::Zero(6,model_nv);
    q=Eigen::MatrixXd::Zero(model_nv + 1, 1);
    dq=Eigen::MatrixXd::Zero(model_nv, 1);
    ddq=Eigen::MatrixXd::Zero(model_nv, 1);
    q.setZero();
    dq.setZero();
    ddq.setZero();
    Rcur.setIdentity();
    dyn_M=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_M_inv=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_C=Eigen::MatrixXd::Zero(model_nv,model_nv);
    dyn_G=Eigen::MatrixXd::Zero(model_nv,1);

    // get joint index for Pinocchio Lib, need to redefined the joint name for new model
    r_ankle_joint=model_biped.getJointId("R_toe_joint");
    l_ankle_joint=model_biped.getJointId("L_toe_joint");
    // r_hand_joint=model_biped.getJointId("R_hand_joint");
    // l_hand_joint=model_biped.getJointId("L_hand_joint");
    // r_hand_joint_fixed=model_biped_fixed.getJointId("R_hand_joint");
    // l_hand_joint_fixed=model_biped_fixed.getJointId("L_hand_joint");
    r_hip_joint=model_biped.getJointId("R_hip2_joint");
    l_hip_joint=model_biped.getJointId("L_hip2_joint");

    r_fe_frame = model_biped.getFrameId("R_foot");
    l_fe_frame = model_biped.getFrameId("L_foot");

    model_nv_fixed = model_biped_fixed.nv;
    J_l_fe_fixed=Eigen::MatrixXd::Zero(6,model_nv_fixed);
    J_r_fe_fixed=Eigen::MatrixXd::Zero(6,model_nv_fixed);
    q_fixed = Eigen::MatrixXd::Zero(model_nv_fixed, 1);
    dq_fixed = Eigen::MatrixXd::Zero(model_nv_fixed, 1);
    ddq_fixed = Eigen::MatrixXd::Zero(model_nv_fixed, 1);

    r_ankle_joint_fixed=model_biped_fixed.getJointId("R_toe_joint");
    l_ankle_joint_fixed=model_biped_fixed.getJointId("L_toe_joint");
    r_hip_joint_fixed=model_biped_fixed.getJointId("R_hip2_joint");
    l_hip_joint_fixed=model_biped_fixed.getJointId("L_hip2_joint");
    base_joint=model_biped.getJointId("root_joint");
    // waist_yaw_joint=model_biped.getJointId("waist_yaw_joint");

    r_fe_frame_fixed = model_biped_fixed.getFrameId("R_foot");
    l_fe_frame_fixed = model_biped_fixed.getFrameId("L_foot");
    r_hip_frame_fixed = model_biped_fixed.getFrameId("R_hip2");
    l_hip_frame_fixed = model_biped_fixed.getFrameId("L_hip2");
}

void Pin_KinDyn::runFloat(RobotState &robotState)
{
    dataBusRead_Float(robotState);
    computeJdJ_Float();
    computeDyn_Float();
    dataBusWrite_Float(robotState);
}

void Pin_KinDyn::dataBusRead_Float(const RobotState &robotState) {
    //  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
    //  while its velocity is expressed in the body coordinate system.
    //  https://github.com/stack-of-tasks/pinocchio/issues/1137
    //  q = [global_base_position, global_base_quaternion, joint_positions]
    //  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    // pinonchio的浮动基广义位置表示在world坐标系下，广义速度表示在body坐标系下
    q=robotState.q;
    dq=robotState.dq;
    dq.block(0,0,3,1)=robotState.base_rot.transpose()*dq.block(0,0,3,1);
    dq.block(3,0,3,1)=robotState.base_rot.transpose()*dq.block(3,0,3,1);
    ddq=robotState.ddq;
}

void Pin_KinDyn::dataBusWrite_Float(RobotState &robotState) {
    // robotState.J_l=J_l;
    // robotState.J_r=J_r;
    robotState.J_l=J_l_fe;
    robotState.J_r=J_r_fe;
    robotState.J_base=J_base;
    // robotState.dJ_l=dJ_l;
    // robotState.dJ_r=dJ_r;
    robotState.dJ_l=dJ_l_fe;
    robotState.dJ_r=dJ_r_fe;
    // robotState.J_hd_l=J_hd_l;
    // robotState.J_hd_r=J_hd_r;
    // robotState.dJ_hd_l=dJ_hd_l;
    // robotState.dJ_hd_r=dJ_hd_r;
    robotState.dJ_base=dJ_base;
    robotState.fe_l_pos_W=fe_l_pos;
    robotState.fe_r_pos_W=fe_r_pos;
    robotState.fe_l_rot_W=fe_l_rot;
    robotState.fe_r_rot_W=fe_r_rot;
    robotState.hip_r_pos_W=hip_r_pos;
    robotState.hip_l_pos_W=hip_l_pos;

    // robotState.hd_l_pos_W=hd_l_pos;
    // robotState.hd_l_rot_W=hd_l_rot;
    // robotState.hd_r_pos_W=hd_r_pos;
    // robotState.hd_r_rot_W=hd_r_rot;

    robotState.dyn_M=dyn_M;
    robotState.dyn_M_inv=dyn_M_inv;
    robotState.dyn_C=dyn_C;
    robotState.dyn_G=dyn_G;
    robotState.dyn_Ag=dyn_Ag;
    robotState.dyn_dAg=dyn_dAg;
    robotState.dyn_Non=dyn_Non;

    robotState.pCoM_W=CoM_pos;
    robotState.Jcom_W=Jcom;

    robotState.inertia = inertia;  // w.r.t body frame
}

void Pin_KinDyn::runFixed(RobotState &robotState) 
{
    dataBusRead_Fixed(robotState);
    computeJ_Fixed();
    dataBusWrite_Fixed(robotState);
}

void Pin_KinDyn::dataBusRead_Fixed(RobotState const &robotState)
{
    q_fixed=robotState.q_fixed;
    dq_fixed=robotState.dq_fixed;
    ddq_fixed=robotState.ddq_fixed;
}

void Pin_KinDyn::dataBusWrite_Fixed(RobotState &robotState)
{
    robotState.J_l_fixed=J_l_fe_fixed;
    robotState.J_r_fixed=J_r_fe_fixed;

    robotState.fe_l_pos_L=fe_l_pos_body;
    robotState.fe_r_pos_L=fe_r_pos_body;
    robotState.fe_l_rot_L=fe_l_rot_body;
    robotState.fe_r_rot_L=fe_r_rot_body;
    robotState.hip_r_pos_L=hip_r_pos_body;
    robotState.hip_l_pos_L=hip_l_pos_body;
    // robotState.hd_l_pos_L=hd_l_pos_body;
    // robotState.hd_l_rot_L=hd_l_rot_body;
    // robotState.hd_r_pos_L=hd_r_pos_body;
    // robotState.hd_r_rot_L=hd_r_rot_body;
}

// update jacobians and joint positions
void Pin_KinDyn::computeJdJ_Float() {
    // 根据当前关节配置更新关节位置
    pinocchio::forwardKinematics(model_biped,data_biped,q);
    // 计算给定模型在特定关节配置下的雅可比矩阵和质心位置。结果可以通过data.Jcom和data.com[0]访问，且两者都在世界坐标系中表示
    pinocchio::jacobianCenterOfMass(model_biped, data_biped, q, true);
//    pinocchio::computeJointJacobians(model_biped,data_biped,q);
    // 计算相对于时间的完整模型雅可比矩阵变化。结果可以通过data.dJ访问
    pinocchio::computeJointJacobiansTimeVariation(model_biped,data_biped,q,dq);
    // 根据关节的相对位置更新关节的全局位置 oMi以及坐标的全局位置 oMf
    pinocchio::updateGlobalPlacements(model_biped,data_biped);
    pinocchio::updateFramePlacements(model_biped,data_biped);
    // LOCAL_WORLD_ALIGNED 在世界坐标系的基向量上表示速度，但速度值表示关节相对于自身的局部运动
    pinocchio::getJointJacobian(model_biped,data_biped,r_ankle_joint,pinocchio::LOCAL_WORLD_ALIGNED,J_r);
    pinocchio::getJointJacobian(model_biped,data_biped,l_ankle_joint,pinocchio::LOCAL_WORLD_ALIGNED,J_l);
    pinocchio::getFrameJacobian(model_biped,data_biped,r_fe_frame,pinocchio::LOCAL_WORLD_ALIGNED,J_r_fe);
    pinocchio::getFrameJacobian(model_biped,data_biped,l_fe_frame,pinocchio::LOCAL_WORLD_ALIGNED,J_l_fe);
    // pinocchio::getJointJacobian(model_biped,data_biped,r_hand_joint,pinocchio::LOCAL_WORLD_ALIGNED,J_hd_r);
    // pinocchio::getJointJacobian(model_biped,data_biped,l_hand_joint,pinocchio::LOCAL_WORLD_ALIGNED,J_hd_l);
    pinocchio::getJointJacobian(model_biped,data_biped,base_joint,pinocchio::LOCAL_WORLD_ALIGNED,J_base);
    
    pinocchio::getJointJacobianTimeVariation(model_biped,data_biped,r_ankle_joint,pinocchio::LOCAL_WORLD_ALIGNED,dJ_r);
    pinocchio::getJointJacobianTimeVariation(model_biped,data_biped,l_ankle_joint,pinocchio::LOCAL_WORLD_ALIGNED,dJ_l);
    pinocchio::getFrameJacobianTimeVariation(model_biped,data_biped,r_fe_frame,pinocchio::LOCAL_WORLD_ALIGNED,dJ_r_fe);
    pinocchio::getFrameJacobianTimeVariation(model_biped,data_biped,l_fe_frame,pinocchio::LOCAL_WORLD_ALIGNED,dJ_l_fe);
    // pinocchio::getJointJacobianTimeVariation(model_biped,data_biped,r_hand_joint,pinocchio::LOCAL_WORLD_ALIGNED,dJ_hd_r);
    // pinocchio::getJointJacobianTimeVariation(model_biped,data_biped,l_hand_joint,pinocchio::LOCAL_WORLD_ALIGNED,dJ_hd_l);
    pinocchio::getJointJacobianTimeVariation(model_biped,data_biped,base_joint,pinocchio::LOCAL_WORLD_ALIGNED,dJ_base);
    fe_l_pos=data_biped.oMf[l_fe_frame].translation();
    fe_l_rot=data_biped.oMf[l_fe_frame].rotation();
    // fe_l_pos=data_biped.oMi[l_ankle_joint].translation();
    // fe_l_rot=data_biped.oMi[l_ankle_joint].rotation();
    hip_l_pos=data_biped.oMi[l_hip_joint].translation();
    fe_r_pos=data_biped.oMf[r_fe_frame].translation();
    fe_r_rot=data_biped.oMf[r_fe_frame].rotation();
    // fe_r_pos=data_biped.oMi[r_ankle_joint].translation();
    // fe_r_rot=data_biped.oMi[r_ankle_joint].rotation();
    hip_r_pos=data_biped.oMi[r_hip_joint].translation();
    base_pos=data_biped.oMi[base_joint].translation();
    base_rot=data_biped.oMi[base_joint].rotation();
    // hd_l_pos=data_biped.oMi[l_hand_joint].translation();
    // hd_l_rot=data_biped.oMi[l_hand_joint].rotation();
    // hd_r_pos=data_biped.oMi[r_hand_joint].translation();
    // hd_r_rot=data_biped.oMi[r_hand_joint].rotation();
    Jcom=data_biped.Jcom;

    std::cout << "fe_l_pos_world: " << fe_l_pos.transpose() << std::endl;
    std::cout << "fe_r_pos_world: " << fe_r_pos.transpose() << std::endl;

    Eigen::MatrixXd Mpj; // transform into world frame, and accept dq that in world frame
    Mpj=Eigen::MatrixXd::Identity(model_nv,model_nv);
    // base_rot表示base in world
    // 将世界坐标系下的虚拟关节转换到base坐标系下
    Mpj.block(0,0,3,3)=base_rot.transpose();
    Mpj.block(3,3,3,3)=base_rot.transpose();
    // std::cout << "Mpj size: " << Mpj.rows() << " " << Mpj.cols() << std::endl;
    // std::cout << "J_l size: " << J_l.rows() << " " << J_l.cols() << std::endl;
    // Mpj * q以及Mpj *dq将其转换到base坐标系下
    J_l=J_l*Mpj;
    J_r=J_r*Mpj;
    J_r_fe=J_r_fe*Mpj;
    J_l_fe=J_l_fe*Mpj;
    J_base=J_base*Mpj;
    dJ_l=dJ_l*Mpj;
    dJ_r=dJ_r*Mpj;
    dJ_l_fe=dJ_l_fe*Mpj;
    dJ_r_fe=dJ_r_fe*Mpj;
    // J_hd_l=J_hd_l*Mpj;
    // J_hd_r=J_hd_r*Mpj;
    // dJ_hd_l=dJ_hd_l*Mpj;
    // dJ_hd_r=dJ_hd_r*Mpj;
    dJ_base=dJ_base*Mpj;
    Jcom=Jcom*Mpj;
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_Fixed() {
    pinocchio::forwardKinematics(model_biped_fixed,data_biped_fixed,q_fixed);
    pinocchio::updateGlobalPlacements(model_biped_fixed,data_biped_fixed);
    pinocchio::updateFramePlacements(model_biped_fixed,data_biped_fixed);

    pinocchio::computeFrameJacobian(model_biped_fixed,data_biped_fixed,q_fixed, l_fe_frame_fixed,pinocchio::LOCAL_WORLD_ALIGNED,J_l_fe_fixed);
    pinocchio::computeFrameJacobian(model_biped_fixed,data_biped_fixed,q_fixed, r_fe_frame_fixed,pinocchio::LOCAL_WORLD_ALIGNED,J_r_fe_fixed);

    fe_l_pos_body=data_biped_fixed.oMf[l_fe_frame_fixed].translation();
    fe_r_pos_body=data_biped_fixed.oMf[r_fe_frame_fixed].translation();
    fe_l_rot_body=data_biped_fixed.oMf[l_fe_frame_fixed].rotation();
    fe_r_rot_body=data_biped_fixed.oMf[r_fe_frame_fixed].rotation();
    hip_l_pos_body=data_biped_fixed.oMf[l_hip_frame_fixed].translation();
    hip_r_pos_body=data_biped_fixed.oMf[r_hip_frame_fixed].translation();
    // hd_l_pos_body=data_biped_fixed.oMi[l_hand_joint_fixed].translation();
    // hd_l_rot_body=data_biped_fixed.oMi[l_hand_joint_fixed].rotation();
    // hd_r_pos_body=data_biped_fixed.oMi[r_hand_joint_fixed].translation();
    // hd_r_rot_body=data_biped_fixed.oMi[r_hand_joint_fixed].rotation();

    std::cout << "fe_l_pos_body: " << fe_l_pos_body.transpose() << std::endl;
    std::cout << "fe_r_pos_body: " << fe_r_pos_body.transpose() << std::endl;
}

// 通过旋转矢量计算更新后的四元数
Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w) {
    Eigen::Matrix3d Rcur=quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc=Eigen::Matrix3d::Identity();
    double theta=w.norm();
    if (theta>1e-8) {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;
        a << 0, -w_norm(2), w_norm(1),
                w_norm(2), 0, -w_norm(0),
                -w_norm(1), w_norm(0), 0;
        // 罗德里格斯公式：R = I * cos(theta) + W * sin(theta) + w * w^T * (1 - cos(theta))  (其中w为旋转矢量，W为w的反对称矩阵)
        // 罗德里格斯公式的矩阵表示形式：R = I + W * sin(theta) + W * W * (1 - cos(theta))
        Rinc=Eigen::Matrix3d::Identity()+a*sin(theta)+a*a*(1-cos(theta));
    }
    Eigen::Matrix3d Rend=Rcur*Rinc;
    Eigen::Quaterniond quatRes;
    quatRes=Rend;
    return quatRes;
}

// intergrate the q with dq, for floating base dynamics
// 根据WBC计算得到的dq，计算更新后的q
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI) {
    Eigen::VectorXd qRes=Eigen::VectorXd::Zero(model_nv+1);
    Eigen::Vector3d wDes;
    wDes<<dqI(3),dqI(4),dqI(5);
    Eigen::Quaterniond quatNew,quatNow;
    quatNow.x()=qI(3);
    quatNow.y()=qI(4);
    quatNow.z()=qI(5);
    quatNow.w()=qI(6);
    quatNew= intQuat(quatNow,wDes);
    qRes=qI;
    qRes(0)+=dqI(0);qRes(1)+=dqI(1);qRes(2)+=dqI(2);
    qRes(3)=quatNew.x();qRes(4)=quatNew.y();qRes(5)=quatNew.z();qRes(6)=quatNew.w();
    for (int i=0;i<model_nv-6;i++)
        qRes(7+i)+=dqI(6+i);
    return qRes;
}

// update dynamic parameters, M*ddq+C*dq+G=tau
void Pin_KinDyn::computeDyn_Float() {
    // 1. 计算质量矩阵M
    // cal M
    // 利用复合刚体算法crba计算关节空间惯性矩阵M的上三角部分
    pinocchio::crba(model_biped, data_biped, q);
    // Pinocchio only gives half of the M, needs to restore it here
    // 使用 data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>() 将严格上三角部分复制到严格下三角部分
    data_biped.M.triangularView<Eigen::Lower>() = data_biped.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped.M;

    // cal Minv
    pinocchio::computeMinverse(model_biped, data_biped, q);
    data_biped.Minv.triangularView<Eigen::Lower>() = data_biped.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv=data_biped.Minv;

    // 2. 计算科里奥利矩阵C
    // cal C
    pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
    dyn_C = data_biped.C;

    // 3. 计算重力向量G
    // cal G
    pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
    dyn_G = data_biped.g;

    // 4. 计算质心动量矩阵Ag以及其导数dAg
    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    pinocchio::dccrba(model_biped,data_biped,q,dq);
    pinocchio::computeCentroidalMomentum(model_biped,data_biped,q,dq);
    dyn_Ag=data_biped.Ag;
    dyn_dAg=data_biped.dAg;

    // cal nonlinear item
    dyn_Non=dyn_C*dq+dyn_G;

    // 5. 计算惯性矩阵I
    // cal I
    pinocchio::ccrba(model_biped, data_biped, q, dq);
    inertia = data_biped.Ig.inertia().matrix();
    // std::cout << "I" << std::endl;
    // std::cout << inertia << std::endl;

    // cal CoM
    CoM_pos = data_biped.com[0];
    // std::cout<<"CoM_W"<<std::endl;
    // std::cout<<CoM_pos.transpose()<<std::endl;

    // base_rot表示base in world
    // 将以下动力学量转换到世界坐标系下
    // 矩阵的坐标变换: I_world = R_world * I_body * R_world^T
    // 其中R_world表示body in world 
    Eigen::MatrixXd Mpj, Mpj_inv; // transform into world frame
    Mpj=Eigen::MatrixXd::Identity(model_nv,model_nv);
    Mpj_inv=Eigen::MatrixXd::Identity(model_nv,model_nv);
    Mpj.block(0,0,3,3)=base_rot.transpose();
    Mpj.block(3,3,3,3)=base_rot.transpose();
    Mpj_inv.block(0,0,3,3)=base_rot;
    Mpj_inv.block(3,3,3,3)=base_rot;
    dyn_M=Mpj_inv*dyn_M*Mpj;
    dyn_M_inv=Mpj_inv*dyn_M_inv*Mpj;
    dyn_C=Mpj_inv*dyn_C*Mpj;
    dyn_G=Mpj_inv*dyn_G;
    dyn_Non=Mpj_inv*dyn_Non;
}