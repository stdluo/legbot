#include "MPC_interface.h"
#include "common_types.h"
#include "RobotStateMpc.h"
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <string.h>

BS::thread_pool pool(POOL_THREADS);

#define K_NUM_LEGS 2

void MPC_interface::setup_problem(double dt, int horizon, double mu, double f_max)
{
  _problem_setup->setup_problem(dt, horizon, mu, f_max);

  // _solver->resize_qp_mats(horizon);
}

void MPC_interface::solveMpcForFootForces(Vec6<double> &footForce, Vec6<double> &footMoment, Vec12<double> &dX_cal)
{
  // _problem_setup->setup_problem();

  MpcOutput result = _solver->solve_mpc_threaded(_update_data, _problem_setup);

  mfp* qs = result.q_soln;
  dX_cal = result.dX_cal.block(0,0,12,1).cast<double>();

  // mfp* qs = _solver->get_q_soln();
  for (int i = 0; i < 6; i++)
  {
    footForce.data()[i] = qs[i];
    footMoment.data()[i] = qs[i+6];
  }

  std::cout << "Foot Force: " << footForce.transpose() << std::endl;
  std::cout << "Foot Moment: " << footMoment.transpose() << std::endl;
}

void MPC_interface::solveMpcForFootForces_multithread(Vec6<double> &footForce, Vec6<double> &footMoment, Vec12<double> &dX_cal)
{
  std::vector<std::future<MpcOutput>> futures;
  MpcOutput result;

  std::cout << "There are still " << pool.get_tasks_total() << " tasks remaining." << std::endl;

  // Adding MPC parallel computation tasks
  for (int i = 0; i < MPC_THREADS; i++)
  {
    std::future<MpcOutput> future = pool.submit_task(
      [i,this]{
        return _solver_vec[i]->solve_mpc_threaded(_update_data,_problem_setup);
      }
    );
    futures.push_back(std::move(future));
  }

  while (true)
  {
    auto it = std::find_if(futures.begin(), futures.end(), [](std::future<MpcOutput>& future) {
            return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        });
      
    if (it != futures.end()) {
            result = it->get();
            std::cout << "Task " << std::distance(futures.begin(), it) << " completed first." << std::endl;
            break;
        }
  }
  
  
  // TODO: Get the solution from the fastest thread
  mfp* qs = result.q_soln;
  dX_cal = result.dX_cal.block(0,0,12,1).cast<double>();

  for (int i = 0; i < 6; i++)
  {
    footForce.data()[i] = qs[i];
    footMoment.data()[i] = qs[i+6];
  }

  std::cout << "Foot Force: " << footForce.transpose() << std::endl;
  std::cout << "Foot Moment: " << footMoment.transpose() << std::endl;
}