#ifndef _MPC_interface
#define _MPC_interface

#include "../include/common/cppTypes.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <iostream>
#include <stdio.h>

// The computer has 20 CPUs and the maximum number of threads in the thread pool is set to 20.
#define MAX_THREADS 20
#define POOL_THREADS 8
#define MPC_THREADS 1

// 多个MPC线程，每个线程
class MPC_interface
{
  public:
    problem_setup_t *_problem_setup;
    update_data_t *_update_data;
    MPC_Solver* _solver;

    std::vector<MPC_Solver*> _solver_vec;
    
    MPC_interface(){}
    MPC_interface(problem_setup_t *problem_setup, update_data_t *update_data, MPC_Solver *solver, std::vector<MPC_Solver*> solver_vec)
      :_problem_setup(problem_setup), _update_data(update_data), _solver(solver), _solver_vec(solver_vec){}

    void setup_problem(double dt, int horizon, double mu, double f_max);
    void solveMpcForFootForces(Vec6<double> &footForce, Vec6<double> &footMoment, Vec12<double> &dX_cal);
    void solveMpcForFootForces_multithread(Vec6<double> &footForce, Vec6<double> &footMoment, Vec12<double> &dX_cal);

    ~MPC_interface(){}
};

double get_solution(int index);

#endif