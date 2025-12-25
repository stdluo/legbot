/*!
 * @file PositionVelocityEstimator.h
 * @brief compute body position/velocity in world/body frames
 */ 



#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H
#include "StateEstimatorContainer.h"

class CheaterPositionVelocityEstimator : public GenericEstimator{
  public:
    virtual void run();
    virtual void setup() {};
};

class LinearKFPositionVelocityEstimator : public GenericEstimator{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

 private:
  Eigen::Matrix<double, 12, 1> _xhat; // p, v, p_foot1, p_foot2
  Eigen::Matrix<double, 6, 1> _ps; // p_foot1, p_foot2
  Eigen::Matrix<double, 6, 1> _vs; // v_foot1, v_foot2
  Eigen::Matrix<double, 12, 12> _A; 
  Eigen::Matrix<double, 12, 12> _Q0;
  Eigen::Matrix<double, 12, 12> _P;
  Eigen::Matrix<double, 14, 14> _R0;
  Eigen::Matrix<double, 12, 3> _B;
  Eigen::Matrix<double, 14, 12> _C;
};

#endif