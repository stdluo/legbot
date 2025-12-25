#ifndef SWINGLEGCONTROLLER_H
#define SWINGLEGCONTROLLER_H

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "../../include/common/LegController.h"
#include "../../MPC/GaitGenerator.h"
#include "../../include/common/ControlFSMData.h"
#include "../../include/common/FootSwingTrajectory.h"
#include "../../include/common/cppTypes.h"

#include <Eigen/Geometry>

 /**
  * @note varibles with _w are in world frame
  * @note varibles with _b are in body frame
 */

#define IKFAST_HAS_LIBRARY

class swingLegController {
    public:
        static constexpr int nLegs = 2;
    
        swingLegController() = default;
        ~swingLegController() = default;

        swingLegController(ControlFSMData *data, Gait* gait, double dtSwing);

        /**
         * @brief Initialize the swing leg controller
         * @param data: pointer to the control data
         * @param gait: pointer to the gait generator
         * @param dtSwing: time step for the swing leg controller
         * @note This function is an alternative to the constructor in case 
         *       the gait generator and control data are not available at 
         *       the time of construction
        */
        void initSwingLegController(ControlFSMData *data, Gait* gait, double dtSwing);

        void updateGait(Gait* gait);
        
        /**
         * @brief Update the swing leg controller
         * @note This function should be called at every control loop iteration
         */
        void updateSwingLeg();
        
        /**
         * @brief Compute an approximate inverse kinematics for 5-DoF swing leg
         * @param bodyPositionDesired: desired position of the end effector in the body frame
         * @param leg: leg index (0 for left, 1 for right)  
         * @param jointAngles: output joint angles
        */
        void computeIK(const Vec3<double> &bodyPositionDesired, Vec4<double> &jointAnglesDes, Vec4<double> &jointAnglesFb, int leg);
 
        

    private:
        Gait* gait;
        const ControlFSMData* data;
        StateEstimate seResult;
        double _dtSwing;
        FootSwingTrajectory<double> footSwingTrajectory[nLegs];
        Vec3<double> pFoot_w[nLegs];
        Vec3<double> vFoot_w[nLegs]; 
        Vec3<double> pFoot_b[nLegs];
        Vec3<double> vFoot_b[nLegs];                        
        Vec2<double> swingStates;
        Vec2<double> swingTimes;
        Vec4<double> qDes[nLegs];  
        Vec4<double> leftJointFb;
        Vec4<double> rightJointFb;
        bool firstSwing[nLegs] = {true, true};        
        
        void updateFootPosition();
        void updateSwingStates();
        void updateSwingTimes();
        void computeFootPlacement();
        void computeFootDesiredPosition();
        void setDesiredJointState();
        void setWBCDesiredJointState();

        // constants can be adjusted if needed
        const double _dt = 0.001;
        const double footHeight = 0.05; 

        // utility functions
        double clamp(double val, double minVal, double maxVal) {
                return std::max(minVal, std::min(val, maxVal));
        }                            

}; // class swingLegController

#endif // SWINGLEGCONTROLLER_H    