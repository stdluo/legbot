#include "../../include/common/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
void CheaterOrientationEstimator::run() {
  this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.lowState->imu.quaternion[0];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.lowState->imu.quaternion[1];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.lowState->imu.quaternion[2];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.lowState->imu.quaternion[3];

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaWorld(0) =
      this->_stateEstimatorData.lowState->imu.gyroscope[0]*1.0;
  this->_stateEstimatorData.result->omegaWorld(1) =
      this->_stateEstimatorData.lowState->imu.gyroscope[1]*1.0;
  this->_stateEstimatorData.result->omegaWorld(2) =
      this->_stateEstimatorData.lowState->imu.gyroscope[2]*1.0;
  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  // useless
  this->_stateEstimatorData.result->omegaBody =
      this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->omegaWorld;
}

void VectorNavOrientationEstimator::run(){
    // orientation
    this->_stateEstimatorData.result->orientation[0] =
        this->_stateEstimatorData.lowState->imu.quaternion[0];
    this->_stateEstimatorData.result->orientation[1] =
        this->_stateEstimatorData.lowState->imu.quaternion[1];
    this->_stateEstimatorData.result->orientation[2] =
        this->_stateEstimatorData.lowState->imu.quaternion[2];
    this->_stateEstimatorData.result->orientation[3] =
        this->_stateEstimatorData.lowState->imu.quaternion[3];

    // Initial orientation
    // Rotation matrix IMU to IMU0: R_Imu0_Imu
    // Rotation matrix Body to IMU: R_Imu_Body
    // Rotation matrix IMU0 to World: R_World_Imu0
    // R_World_Body = R_World_Imu0 * R_Imu0_Imu * R_Imu_Body
    // The following derivation assumes R_Imu_Body = I (Identity matrix)
    if (_b_visit < 5)
    {
        Vec3<double> rpy_init = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
        rpy_init(0) = 0;
        rpy_init(1) = 0;
        _ori_init_inv = ori::rpyToQuat(-rpy_init);
        _b_visit ++;
        std::cout << "rpy_init: " << rpy_init.transpose() << std::endl;
    }
    this->_stateEstimatorData.result->orientation = 
        ori::quatProduct(_ori_init_inv, this->_stateEstimatorData.result->orientation);

    this->_stateEstimatorData.result->rpy = 
        ori::quatToRPY(this->_stateEstimatorData.result->orientation);

    // std::cout << "orientation: " << this->_stateEstimatorData.result->orientation.transpose() << std::endl;
    std::cout << "RPY: " << this->_stateEstimatorData.result->rpy.transpose() << std::endl;

    // 通过IMU四元数得到的旋转矩阵表示Rotation matrix from IMU to World: R_Imu_World
    // rBody表示Rotation matrix from Body to World: R_Body_World
    this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
        this->_stateEstimatorData.result->orientation);  
    
    // angular velocity
    this->_stateEstimatorData.result->omegaBody[0] = 
        this->_stateEstimatorData.lowState->imu.gyroscope[0];
    this->_stateEstimatorData.result->omegaBody[1] = 
        this->_stateEstimatorData.lowState->imu.gyroscope[1];
    this->_stateEstimatorData.result->omegaBody[2] = 
        this->_stateEstimatorData.lowState->imu.gyroscope[2];
    
    this->_stateEstimatorData.result->omegaWorld = 
        this->_stateEstimatorData.result->rBody.transpose() *
        this->_stateEstimatorData.result->omegaBody;
    
    // linear acceleration
    this->_stateEstimatorData.result->aBody[0] = 
        this->_stateEstimatorData.lowState->imu.accelerometer[0];
    this->_stateEstimatorData.result->aBody[1] = 
        this->_stateEstimatorData.lowState->imu.accelerometer[1];
    this->_stateEstimatorData.result->aBody[2] = 
        this->_stateEstimatorData.lowState->imu.accelerometer[2];

    std::cout << "aBody: " << this->_stateEstimatorData.result->aBody.transpose() << std::endl;
    
    this->_stateEstimatorData.result->aWorld = 
        this->_stateEstimatorData.result->rBody.transpose() *
        this->_stateEstimatorData.result->aBody;
}