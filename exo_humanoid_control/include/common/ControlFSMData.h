#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "pino_kin_dyn.h"
#include "LegController.h"
#include "Biped.h"
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"
#include "../interface/IOInterface.h"
#include "../interface/CheatIO.h"
#include "StateEstimatorContainer.h"
#include "DesiredCommand.h"
#include "RobotState.h"
#include "../../MPC/MPC_interface.h"
#include "../../WBC/wbc_priority.h"

struct ControlFSMData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateEstimatorContainer *_stateEstimator;
  DesiredStateCommand *_desiredStateCommand;

  CheatIO *_interface;

  Biped *_biped;
  Pin_KinDyn *_pinocchio;
  LegController *_legController;

  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;
  RobotState *_robotState;
  
  MPC_interface *_mpc;
  WBC_priority *_wbc;

  void sendRecv(){
    _interface->sendRecv(_lowCmd, _lowState);
    // _interface->sendLKFCoM(_stateEstimator->_data.result->positionLKF);
  }
};


#endif  // CONTROLFSM_H