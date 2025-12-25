#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/FSM/FSM.h"
#include "../include/interface/CheatIO.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

int times_main = 0;
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "exo_humanoid_control", ros::init_options::AnonymousName);

    const std::string urdf_file = std::string("src/exo_humanoid/urdf/exo_humanoid.urdf");
    const std::string ik_urdf_file = std::string("/ik_urdf_model");
    
    std::string robot_name = "exo_humanoid";
    std::cout << "robot name " << robot_name << std::endl;

    CheatIO* ioInter = new CheatIO(robot_name);

    ros::Rate rate(1000);
    double dt = 0.001;
    
    Biped biped(urdf_file, ik_urdf_file);
    Pin_KinDyn* pinocchio =new Pin_KinDyn(urdf_file);
    LegController* legController = new LegController(biped);

    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();
    RobotState* robotState = new RobotState(pinocchio->model_nv);

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);   

    // stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    // stateEstimator->addEstimator<CheaterPositionVelocityEstimator>(); 

    // stateEstimator->addEstimator<VectorNavOrientationEstimator>();   
    // stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>(); 

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();     

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    update_data_t *update = new update_data_t();
    problem_setup_t *problem = new problem_setup_t();
    MPC_Solver *solver = new MPC_Solver();
    
    // MPC计算的并行线程数
    std::vector<MPC_Solver*> solver_vec;
    for (int i = 0; i < MPC_THREADS; i++)
    {
        solver_vec.push_back(new MPC_Solver());
    }
    
    MPC_interface *mpc = new MPC_interface(problem, update, solver, solver_vec);

    WBC_priority *wbc = new WBC_priority(pinocchio->model_nv, 18, 22, 0.5, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_biped = &biped;
    _controlData->_pinocchio = pinocchio;
    _controlData->_legController = legController;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;
    _controlData->_robotState = robotState;
    _controlData->_mpc = mpc;
    _controlData->_wbc = wbc;

    FSM* _FSMController = new FSM(_controlData);

    signal(SIGINT, ShutDown);
    
    // std::cout << "starting main loop" << std::endl;
    while(running)
    {
        rate.sleep();
        ros::spinOnce();

        times_main++;
        if (times_main > 4)
        {
          _FSMController->run();
        }
        
        // if (times_main > 100)
        // {
        //     break;
        // }
    }
    
    delete _controlData;
    return 0;

}
