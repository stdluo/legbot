#ifndef _RobotStateMpc
#define _RobotStateMpc

#include <eigen3/Eigen/Dense>
#include "common_types.h"
#include "../include/common/cppTypes.h"

using Eigen::Matrix;
using Eigen::Quaternionf;
class RobotStateMpc
{
    public:
        RobotStateMpc(){
            offset << 0.080025, 0.002704, -0.336423;
            
            I_body << 2.634141,  0.002263, -0.372420,
                    0.002263,  2.241055,  0.011940,
                    -0.372420,  0.011940,  0.695717;
        }
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw, Vec3<float> p_com, Vec3<float>offset, Mat3<float> Interia);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,1> offset;  // 坐标原点到质心的距离
        Matrix<fpt,3,2> r_feet;  // the position of the foot relative to the trunk
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;

        fpt mass = 14.72f;
    //private:
};
#endif
