/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H


#include "Biped.h"
#include "cppTypes.h"
#include "Filter.h"
#include "RobotState.h"
#include "pino_kin_dyn.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"

/*!
 * Data sent from control algorithm to legs
 */ 
struct LegControllerCommand{
    LegControllerCommand() {zero();}

    void zero();

    Vec4<double> qDes, qdDes, tau;
    Vec3<double> pDes, vDes;
    Mat4<double> kpJoint, kdJoint;
    Vec6<double> feedforwardForce;
    Vec3<double> hiptoeforce;
    Mat3<double> kpCartesian;
    Mat3<double> kdCartesian;
    double kphip1 = 20;
    double kdhip1 = 2;
    double kptoe = 20;
    double kdtoe = 2;
};

/*!
 * Data returned from legs to control code
 */ 
struct LegControllerData{
    LegControllerData();

    void zero();
    Vec4<double> q, qd, qd_f;
    Vec3<double> p, v, v_f;
    Mat64<double> J_force_moment;
    Mat34<double> J_force;
    Vec4<double> tau;
    Vec4<double> tau_ff;
    Mat3<float> R_foot;
    Vec3<double> hip_p;

    LowPassFilter qd_filter[4];
};

/*!
 * Controller for 2 legs of exo_humanoid
 */ 
class LegController {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegController(Biped& biped);
    
    void zeroCommand();
    void edampCommand(double gain);
    void updateData(const RobotState* state);
    void updateData(const LowlevelState* state);
    void updateCommand(RobotState* state, Pin_KinDyn* pino);
    void updateCommand(LowlevelCmd* cmd);
    void setEnabled(bool enabled) {_legsEnabled = enabled;};

    LegControllerCommand commands[2];
    LegControllerData data[2];
    bool _legsEnabled = false;
    std::string limbName[4] = {"Hip", "Hip 2", "Calf", "Toe"};
    std::string Side[2] = {"Left ", "Right"};        
    Biped& _biped;

    private:
    void computeLegJacobianAndPosition(void);
    void checkLegJointCount(void);
};

#endif