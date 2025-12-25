#include "../../include/common/Biped.h"
#include <ros/ros.h>

Biped::Biped(std::string urdf_file, bool if_cout) : 
_ik_solver_left("",""),
_ik_solver_right("","")
{   
    _urdf_file = urdf_file;
    
    // initialize model and data using pinocchio
    pinocchio::urdf::buildModel(urdf_file, _model);
    _data = pinocchio::Data(_model);
    if (if_cout)
    {
        std::cout << "model name: " << _model.name << std::endl;
        std::cout << "nq: " << _model.nq << std::endl;
        std::cout << "nv: " << _model.nv << std::endl;

        std::cout << "number of joints = " << _model.njoints;
        std::cout << "number of frames = " << _model.nframes;
        std::cout << "number of bodies = " << _model.nbodies;

        std::cout << "joint name & joint position:" << std::endl;
        for (size_t i = 0; i < _model.njoints; i++) 
        {
            std::cout << _model.names.at(i) << std::endl;
            // std::cout << _data.oMi.at(i).translation().transpose() << std::endl;
        }

        std::cout << ("frame name & frame position:") << std::endl;
        for (size_t i = 0; i < _model.nframes; i++)
        {
            std::cout << _model.frames.at(i).name << std::endl;
            // std::cout << _data.oMf.at(i).translation().transpose() << std::endl;
        }
    }
}

Biped::Biped(std::string urdf_file, std::string ik_urdf_file, bool if_cout) :
// _ik_solver_left("trunk","L_foot",ik_urdf_file, 5e-3, 1e-3, TRAC_IK::Distance),
// _ik_solver_right("trunk","R_foot",ik_urdf_file, 5e-3, 1e-3, TRAC_IK::Distance)
_ik_solver_left("trunk","L_foot",ik_urdf_file, 1e-3, 1e-3, TRAC_IK::Speed),
_ik_solver_right("trunk","R_foot",ik_urdf_file, 1e-3, 1e-3, TRAC_IK::Speed)
{   
    /** pinocchio initialize */
    _urdf_file = urdf_file;
    
    // initialize model and data using pinocchio
    pinocchio::urdf::buildModel(urdf_file, _model);
    _data = pinocchio::Data(_model);
    if (if_cout)
    {
        std::cout << "----------pinocchio----------" << std::endl;
        std::cout << "model name: " << _model.name << std::endl;
        std::cout << "nq: " << _model.nq << std::endl;
        std::cout << "nv: " << _model.nv << std::endl;

        std::cout << "number of joints = " << _model.njoints;
        std::cout << "number of frames = " << _model.nframes;
        std::cout << "number of bodies = " << _model.nbodies;

        std::cout << "joint name & joint position:" << std::endl;
        for (size_t i = 0; i < _model.njoints; i++) 
        {
            std::cout << _model.names.at(i) << std::endl;
            // std::cout << _data.oMi.at(i).translation().transpose() << std::endl;
        }

        std::cout << ("frame name & frame position:") << std::endl;
        for (size_t i = 0; i < _model.nframes; i++)
        {
            std::cout << _model.frames.at(i).name << std::endl;
            // std::cout << _data.oMf.at(i).translation().transpose() << std::endl;
        }
    }

    /** trac_ik initialize */

    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    bool valid = _ik_solver_left.getKDLChain(chain);

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    valid = _ik_solver_left.getKDLLimits(ll, ul);

    if (!valid)
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    if (if_cout)
    {
        std::cout << "----------trac_ik----------" << std::endl;
        ROS_INFO("IK left: Using %d joints", chain.getNrOfJoints());
    }

    valid = _ik_solver_right.getKDLChain(chain);

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    valid = _ik_solver_right.getKDLLimits(ll, ul);

    if (!valid)
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    if (if_cout)
    {
        ROS_INFO("IK right: Using %d joints", chain.getNrOfJoints());
    }
}