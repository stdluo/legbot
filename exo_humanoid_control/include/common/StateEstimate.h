#ifndef _STATE_ESTIMATE_H_
#define _STATE_ESTIMATE_H_

#include "cppTypes.h"
/*!
 * Result of state estimation
 */

struct StateEstimate {
    Vec2<double> contactEstimate; // LKF所需的支撑相相位
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;

    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;

    // LKF
    Vec3<double> positionLKF;
    Vec3<double> vWorldLKF;
};

struct CheaterState{
  Quat<double> orientation;
  Vec3<double> position;
  Vec3<double> omegaBody;
  Vec3<double> vBody;
  Vec3<double> acceleration;
};

#endif