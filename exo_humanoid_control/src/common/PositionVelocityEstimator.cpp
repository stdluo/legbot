#include "../../include/common/PositionVelocityEstimator.h"

#include <iostream>
#include <fstream>

std::ofstream  outfile_cheat, outfile_lkf;

void CheaterPositionVelocityEstimator::run() {
  static int num_init = 0;
  num_init++;

 // std::cout << "run StateEstimator" << std::endl;
  for(int i = 0; i < 3; i++){
    // if (num_init < 5000)
    // {
      this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    // }
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
  }

  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;

  // std::cout << "Cheat position: " << this->_stateEstimatorData.result->position.transpose() << std::endl;
  // std::cout << "Cheat vWorld: " << this->_stateEstimatorData.result->vWorld.transpose() << std::endl;

  // std::cout << "vx: " << this->_stateEstimatorData.result->vWorld[0] << std::endl;
  // std::cout << "vy: " << this->_stateEstimatorData.result->vWorld[1] << std::endl;
  // std::cout << "vz: " << this->_stateEstimatorData.result->vWorld[2] << std::endl;
  // std::cout << "vx body: " << this->_stateEstimatorData.result->vBody[0] << std::endl;
  // std::cout << "vy body: " << this->_stateEstimatorData.result->vBody[1] << std::endl;
  // std::cout << "vz body: " << this->_stateEstimatorData.result->vBody[2] << std::endl;
  
  outfile_cheat << this->_stateEstimatorData.result->position.transpose() << " " << this->_stateEstimatorData.result->vWorld.transpose() << std::endl;
  // outfile_cheat.close();
}

void LinearKFPositionVelocityEstimator::setup() {
  outfile_cheat.open("data_process/file_cheat.txt");
  outfile_lkf.open("data_process/file_lkf.txt");

  double dt = 0.001;
  _xhat.setZero();
  // _xhat init
  _xhat(2) = 0.9;
  _xhat.block(6, 0, 3, 1) = Eigen::Vector3d(0.08, 0.14145, 0);
  _xhat.block(9, 0, 3, 1) = Eigen::Vector3d(0.08, -0.14145, 0);
  _ps.setZero();
  _vs.setZero();
  // 状态方程系数矩阵A、B
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
  _B.setZero();
  _B.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<double, 3, 3>::Identity();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  // 观测方程系数矩阵C
  // 足底在世界坐标系的位置和足底在机器人坐标系的位置转换
  // 足底在世界坐标系的速度和足底在机器人坐标系的位置转换
  // 足底在世界坐标系的高度为0
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(0, 6, 6, 6) = double(-1) * Eigen::Matrix<double, 6, 6>::Identity();
  _C.block(6, 0, 3, 6) = C2;
  _C.block(9, 0, 3, 6) = C2;
  _C(13, 11) = double(1);
  _C(12, 8) = double(1);
  // 初始化P矩阵，初始值可调
  _P.setIdentity();
  _P = double(1000) * _P;
  // 过程噪声矩阵Q
  // 观测噪声矩阵R
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();
  _R0.setIdentity();
  std::cout << "LKF init" << std::endl;
}

LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
void LinearKFPositionVelocityEstimator::run() {
  // cheetah的参数
  // foot_height_sensor_noise      :  0.001
  // foot_process_noise_position   :  0.002
  // foot_sensor_noise_position    :  0.001
  // foot_sensor_noise_velocity    :  0.1
  // imu_process_noise_position    :  0.02
  // imu_process_noise_velocity    :  0.02

  // double process_noise_pimu = 10000;
  // double process_noise_vimu = 10000;
  // for standing
  // double process_noise_pimu = 1;
  // double process_noise_vimu = 1;
  double process_noise_pimu = 0.02;
  double process_noise_vimu = 0.02;
  double process_noise_pfoot = 0.002;
  double sensor_noise_pimu_rel_foot = 0.001;
  double sensor_noise_vimu_rel_foot = 0.1;
  double sensor_noise_zfoot = 0.01;
  // double sensor_noise_pimu_rel_foot = 1;
  // double sensor_noise_vimu_rel_foot = 1;
  // double sensor_noise_zfoot = 1;

  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 6, 6) * process_noise_pfoot;

  Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();
  R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
  R.block(6, 6, 6, 6) =
      _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
  R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<double> g(0, 0, double(-9.81));
  Mat3<double> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
  // std::cout << "aWorld: " <<this->_stateEstimatorData.result->aWorld.transpose() << std::endl;
  Vec3<double> a = this->_stateEstimatorData.result->aWorld + g; 
  // std::cout << "A WORLD: " << a[0] << " " << a[1] << " " << a[2] << std::endl;
  Vec2<double> pzs = Vec2<double>::Zero();
  Vec2<double> trusts = Vec2<double>::Zero();
  Vec3<double> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 2; i++) {
    int i1 = 3 * i;
    Vec3<double> p_rel = this->_stateEstimatorData.legControllerData[i].p;
    Vec3<double> dp_rel = this->_stateEstimatorData.legControllerData[i].v_f; 
    Vec3<double> p_f = Rbod * p_rel;
    Vec3<double> dp_f =
        Rbod * (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    // std::cout << "p_rel: " << p_rel[0] << " " << p_rel[1] << " " << p_rel[2] << std::endl;
    // std::cout << "dp_rel: " << dp_rel[0] << " " << dp_rel[1] << " " << dp_rel[2] << std::endl;
    // std::cout << "p_f: " << p_f[0] << " " << p_f[1] << " " << p_f[2] << std::endl;

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 6 + i1;
    rindex3 = 12 + i;

    double trust = 1.0;
    double phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), 1.0);
    // double trust_window = 0.25;
    double trust_window = 0.1;

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (double(1) - trust_window)) {
      trust = (double(1) - phase) / trust_window;
    }
    double high_suspect_number(1000);
    // double high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (double(1) + (double(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<double, 14, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;

  // _xhat(6) = 0;
  // _xhat(9) = 0;
  Eigen::Matrix<double, 12, 12> At = _A.transpose();
  Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
  Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
  Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
  Eigen::Matrix<double, 14, 1> ey = y - yModel;
  Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

  // Eigen::Matrix<double, 14, 14> S_inv = S.inverse();

  // std::cout << "LKF position: " << _xhat.block(0,0,3,1).transpose() << std::endl;
  // std::cout << "LKF vWorld: " << _xhat.block(3,0,3,1).transpose() << std::endl;
  // std::cout << "LKF pLeftFootWorld: " << _xhat.block(6,0,3,1).transpose() << std::endl;
  // std::cout << "LKF pRightFootWorld: " << _xhat.block(9,0,3,1).transpose() << std::endl;

  // std::cout << "ey: " << ey.transpose() << std::endl;

  // todo compute LU only once
  // S_ey = S_inv * ey ———— LU分解为一种求逆的方法，避免奇异
  Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
  // Eigen::Matrix<double, 14, 1> S_ey = S_inv * ey;

  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
  // Eigen::Matrix<double, 14, 12> S_C = S_inv * _C;
  _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

  // 由于数值计算中的误差，矩阵_P可能会变得不对称。通过对_P和其转置求平均，确保_P是对称的。
  Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
  _P = (_P + Pt) / double(2);

  // QS: The function of the following code? 
  if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) {
    _P.block(0, 2, 2, 10).setZero();
    _P.block(2, 0, 10, 2).setZero();
    _P.block(0, 0, 2, 2) /= double(10);
  }

  std::cout << "LKF position: " << _xhat.block(0,0,3,1).transpose() << std::endl;
  std::cout << "LKF vWorld: " << _xhat.block(3,0,3,1).transpose() << std::endl;
  // std::cout << "LKF pLeftFootWorld: " << _xhat.block(6,0,3,1).transpose() << std::endl;
  // std::cout << "LKF pRightFootWorld: " << _xhat.block(9,0,3,1).transpose() << std::endl;
  
  static int num_init = 0;
  num_init++;
  // if(num_init > 100)
  // {
  //   std::cout << "LKF run" << std::endl;
    this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
    this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
    // this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1) + Eigen::Vector3d(0.01, 0, 0);
    this->_stateEstimatorData.result->vBody =
        this->_stateEstimatorData.result->rBody *
        this->_stateEstimatorData.result->vWorld;
  // }

  this->_stateEstimatorData.result->positionLKF = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorldLKF = _xhat.block(3, 0, 3, 1);

  outfile_lkf << this->_stateEstimatorData.result->position.transpose() << " " << this->_stateEstimatorData.result->vWorld.transpose() << " " << this->_stateEstimatorData.result->aWorld.transpose() << std::endl;
  // outfile_lkf.close();
}