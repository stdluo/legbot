#ifndef _solver_mpc
#define _solver_mpc

#include <eigen3/Eigen/Dense>
#include "common_types.h"
#include "ProblemSetupAndUpdate.h"
#include "RobotStateMpc.h"
#include "../third_party/qpOASES/include/qpOASES.hpp"
#include "../third_party/BS_thread_pool/include/BS_thread_pool.hpp"
#include "../third_party/BS_thread_pool/include/BS_thread_pool_utils.hpp"

using Eigen::Matrix;
using Eigen::Quaternionf;
using Eigen::Quaterniond;

using Eigen::Dynamic;

class MPC_Solver
{
    public:
        MPC_Solver(){}
        ~MPC_Solver(){}

        void resize_qp_mats(s16 horizon);
        void solve_mpc(update_data_t* update, problem_setup_t* setup);
        mfp* get_q_soln();

        MpcOutput solve_mpc_threaded(update_data_t* update, problem_setup_t* setup);
        
    private:
        RobotStateMpc rs;

        Matrix<fpt, Dynamic, 13> A_qp;
        Matrix<fpt, Dynamic, Dynamic> B_qp;
        Matrix<fpt, 13, 12> Bdt;
        Matrix<fpt, 13, 13> Adt;
        Matrix<fpt, 25, 25> ABc, expmm;
        Matrix<fpt, Dynamic, Dynamic> S;
        Matrix<fpt, Dynamic, 1> X_d;
        Matrix<fpt, Dynamic, 1> U_b;
        Matrix<fpt, Dynamic, 1> L_b;
        Matrix<fpt, Dynamic, Dynamic> fmat;
        Matrix<fpt, Dynamic, Dynamic> qH;
        Matrix<fpt, Dynamic, 1> qg;
        Matrix<fpt, 18, 12> F_control;
        Matrix<fpt, 1, 3> tx_F;
        Matrix<fpt, 1, 3> ty_F;
        Matrix<fpt, 1, 3> DNFG;

        // Matrix<fpt,Dynamic,Dynamic> eye_12h;
        Matrix<fpt, Dynamic, Dynamic> Alpha_diag;
        Matrix<fpt, Dynamic, Dynamic> Alpha_rep;

        qpOASES::real_t *H_qpoases;
        qpOASES::real_t *g_qpoases;
        qpOASES::real_t *A_qpoases;
        qpOASES::real_t *lb_qpoases;
        qpOASES::real_t *ub_qpoases;
        qpOASES::real_t *q_soln;

        qpOASES::real_t *H_red;
        qpOASES::real_t *g_red;
        qpOASES::real_t *A_red;
        qpOASES::real_t *lb_red;
        qpOASES::real_t *ub_red;
        qpOASES::real_t *q_red;
        u8 real_allocated = 0;

        qpOASES::real_t *max_time;

        char var_elim[2000];
        char con_elim[2000];

        Matrix<fpt, 13, 1> x_0;
        Matrix<fpt, 3, 3> I_world;
        Matrix<fpt, 13, 13> A_ct;
        Matrix<fpt, 13, 12> B_ct_r;
        Vec12<fpt> U_opt;

        Eigen::Matrix<fpt, 13, 1> dX_cal;

        void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,2> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B);
        void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon);
};

#endif
