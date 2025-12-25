#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

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

#include <trac_ik/trac_ik.hpp>

#include <vector>
#include "cppTypes.h"

class Biped {
    public:
        Biped():_ik_solver_left("",""),_ik_solver_right("",""){};

        Biped(std::string urdf_file, bool if_cout = false);
        Biped(std::string urdf_file, std::string ik_urdf_file, bool if_cout = false);

        // pinocchio
        std::string _urdf_file;
        pinocchio::Model _model;
        pinocchio::Data _data;

        // trac_ik
        std::string _ik_urdf_file;
        TRAC_IK::TRAC_IK _ik_solver_left;
        TRAC_IK::TRAC_IK _ik_solver_right;


    private:
        void checkLegIndex(int leg) const {
            if (leg < 0 || leg >= 2) {
                throw std::invalid_argument("Invalid leg index");
            }
        }
};

#endif