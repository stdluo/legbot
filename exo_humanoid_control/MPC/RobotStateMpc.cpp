#include "RobotStateMpc.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotStateMpc::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_, Vec3<float> p_com, Vec3<float>offset, Mat3<float> Interia)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        // this->p(i) = p_com[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
        this->offset(i) = offset(i);
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    this->I_body = Interia;

    // std::cout << "Offset: " << offset.transpose() << std::endl;
    // std::cout << "Inertia: " << std::endl << Interia << std::endl;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 2; c++){
        // std::cout<< "r_ : " << rs << ", "<< c << ": " << r_[rs*2 + c] <<std::endl;
        this->r_feet(rs,c) = r_[rs*2 + c];
        }

    // Transform the position from the trunk frame to the center of mass frame
    this->p += offset;
    this->r_feet.block(0,0,3,1) -= offset;
    this->r_feet.block(0,1,3,1) -= offset;

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    
    // R_yaw = R;     
    // R = R_yaw;     

    //TODO: Consider normalizing quaternion??
}

void RobotStateMpc::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



