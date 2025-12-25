#include "SolverMPC.h"
#include "common_types.h"
#include "MPC_interface.h"
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>

// TODO: 务必在创建 BS::thread_pool 对象之前创建 BS::synced_stream
BS::synced_stream sync_out;

// #define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
// big enough to act like infinity, small enough to avoid numerical weirdness.

// Returns QP solution
mfp *MPC_Solver::get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.0001 && a > -.0001);
}

s8 near_two(fpt a)
{
  return near_zero(a - 2);
}

// Sets parameter matrices to qpOASES type:
void matrix_to_real(qpOASES::real_t *dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}

void MPC_Solver::c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon)
{
#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n"
       << Adt << "\nBdt:\n"
       << Bdt << endl;
#endif
  // cout<< "----------------------------------------"<<endl;
  // cout << "Ac: \n"
  //      << Ac << "\nBc:\n"
  //      << Bc << endl;
  // cout<< "----------------------------------------"<<endl;
  if (horizon > 19)
  {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt, 13, 13> Acd = Matrix<fpt, 13, 13>::Identity() + dt * Ac;
  Matrix<fpt, 13, 12> Bcd = dt * Bc;

  // TODO: A_qp and B_qp should not have a fixed horizon of 10
  for (int i = 0; i < 10; i++)
  {
    Eigen::Matrix<fpt, 13, 13> Acdm;
    Acdm = Matrix<fpt, 13, 13>::Identity();
    for (int j = 0; j < i + 1; j++)
    {
      Acdm *= Acd;
    }

    A_qp.block<13, 13>(i * 13, 0) << Acdm;
  }
  
  Eigen::Matrix<fpt, 13, 13> Acdp;
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < i + 1; j++)
    {
      Acdp = Matrix<fpt, 13, 13>::Identity();
      for (int k = 0; k < i - j; k++)
      {
        Acdp *= Acd;
      }

      if (i - j == 0)
      {
        Acdp = Matrix<fpt, 13, 13>::Identity();
      }

      B_qp.block<13, 12>(i * 13, j * 12) << Acdp * Bcd;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      B_qp.block<13, 12>(i * 13, j * 12) << Matrix<fpt, 13, 12>::Zero();
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout << "AQP:\n"
       << A_qp << "\nBQP:\n"
       << B_qp << endl;
#endif
}

// Resizing & initaliztation:
void MPC_Solver::resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon * horizon;

  A_qp.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon * 1;

  B_qp.resize(13 * horizon, 12 * horizon);
  mcount += 13 * h2 * 12;

  S.resize(13 * horizon, 13 * horizon);
  mcount += 13 * 13 * h2;

  X_d.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon;

  U_b.resize(18 * horizon, Eigen::NoChange);
  mcount += 18 * horizon;

  L_b.resize(18 * horizon, Eigen::NoChange);
  mcount += 18 * horizon;

  fmat.resize(18 * horizon, 12 * horizon);
  mcount += 18 * 12 * h2;

  qH.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * h2;

  qg.resize(12 * horizon, Eigen::NoChange);
  mcount += 12 * horizon;

  // eye_12h.resize(12*horizon, 12*horizon);
  // mcount += 12*12*horizon;

  Alpha_rep.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  L_b.setZero();
  fmat.setZero();
  qH.setZero();
  // eye_12h.setIdentity();
  Alpha_rep.setZero();

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);

    free(max_time);
  }

  H_qpoases = (qpOASES::real_t *)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  g_qpoases = (qpOASES::real_t *)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  A_qpoases = (qpOASES::real_t *)malloc(12 * 18 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 18 * h2;
  lb_qpoases = (qpOASES::real_t *)malloc(18 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 18 * horizon;
  ub_qpoases = (qpOASES::real_t *)malloc(18 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 18 * horizon;
  q_soln = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t *)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  g_red = (qpOASES::real_t *)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  A_red = (qpOASES::real_t *)malloc(12 * 18 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 18 * h2;
  lb_red = (qpOASES::real_t *)malloc(18 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 18 * horizon;
  ub_red = (qpOASES::real_t *)malloc(18 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 18 * horizon;
  q_red = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  max_time = (qpOASES::real_t *)malloc(1 * horizon * sizeof(qpOASES::real_t));
  mcount += 1;
  real_allocated = 1;

  // sync_out.println("malloc'd",mcount,"floating point numbers.");
  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

// Matrix Operations:
inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
  Matrix<fpt, 3, 3> cm;
  cm << 0.f, -r(2), r(1),
      r(2), 0.f, -r(0),
      -r(1), r(0), 0.f;
  return I_inv * cm;
}

// continuous time state space matrices.
void MPC_Solver::ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 2> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13> &A, Matrix<fpt, 13, 12> &B)
{
  A.setZero();
  A.block<3, 3>(0, 6) << R_yaw;
  A.block<3, 3>(3, 9) << Matrix<fpt, 3, 3>::Identity();
  A.block<3, 1>(9, 12) << 0, 0, -1.f;

  B.setZero();
  Matrix<fpt, 3, 3> I_inv = I_world.inverse();
  Matrix<fpt, 3, 3> Im;
  Im << 0, 0, 0, 0, 1.f, 0, 0, 0, 1.f;
  for (s16 b = 0; b < 2; b++)
  {
    B.block<3, 3>(6, b * 3) << cross_mat(I_inv, r_feet.col(b));
  }
  B.block<3, 3>(6, 6) << I_inv;
  B.block<3, 3>(6, 9) << I_inv;                             // switch
  B.block<3, 3>(9, 0) << Matrix<fpt, 3, 3>::Identity() / m; // switch
  B.block<3, 3>(9, 3) << Matrix<fpt, 3, 3>::Identity() / m;
}

// #define K_PRINT_EVERYTHING

// Main function:
void MPC_Solver::solve_mpc(update_data_t *update, problem_setup_t *setup)
{
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw, update->p_com, update->offset ,update->Interia);
#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  rs.print();
  print_update_data(update, setup->horizon);
#endif

  // rs.print();

  // roll pitch yaw
  Matrix<fpt, 3, 1> rpy;
  quat_to_rpy(rs.q, rpy);
  Matrix<fpt, 3, 3> Rb;
  Rb = euler_to_rotation(rpy(0), rpy(1), rpy(2)); // use this instead of rs.R_yaw, this is not Rotation matrix

  // initial state (13 state representation)
  x_0 << rpy(0), rpy(1), rpy(2), rs.p, rs.w, rs.v, 9.81f;
  // sync_out.println("x_0: ", x_0.transpose());
  // std::cout << "x_0: " << x_0.transpose() << std::endl;
  I_world = rs.R * rs.I_body * rs.R.transpose(); // original

  // ct_ss_mats(I_world, 47.0, rs.r_feet, Rb, A_ct, B_ct_r);
  ct_ss_mats(I_world, rs.mass , rs.r_feet, Rb, A_ct, B_ct_r);

  // Rotation of Foot:
  Matrix<fpt, 3, 3> R_foot_L;
  Matrix<fpt, 3, 3> R_foot_R;
  R_foot_L = update->R_foot[0];
  R_foot_R = update->R_foot[1];

  // sync_out.println("r_feet: ", BS::synced_stream::endl, rs.r_feet);
  // sync_out.println("R_foot_L: ", BS::synced_stream::endl, R_foot_L);
  // sync_out.println("rR_foot_R: ", BS::synced_stream::endl, R_foot_R);

  // std::cout << "r_feet: " << std::endl << rs.r_feet << std::endl;
  // std::cout << "R_foot_L: " << std::endl << R_foot_L << std::endl;
  // std::cout << "R_foot_R: " << std::endl << R_foot_R << std::endl;

#ifdef K_PRINT_EVERYTHING
  cout << "Initial state: \n"
       << x_0 << endl;
  cout << "World Inertia: \n"
       << I_world << endl;
  cout << "A CT: \n"
       << A_ct << endl;
  cout << "B CT (simplified): \n"
       << B_ct_r << endl;
#endif
  // QP matrices
  c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);

  // weights
  Matrix<fpt, 13, 1> full_weight;
  for (u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon, 1);

  // for (int i = 0; i < 12; i++)
  // {
  //   std::cout << "x_d: ";
  //   std::cout << update->traj[i] << " ";
  // }
  // std::cout << std::endl;

  // trajectory
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 12; j++)
      X_d(13 * i + j, 0) = update->traj[12 * i + j];
  }

  for (s16 leg = 0; leg < 2; leg++)
  {

    for (s16 i = 0; i < setup->horizon; i++)
    {
      for (s16 j = 0; j < 4; j++)
      { // bound
        U_b(9 * leg + j + 18 * i) = BIG_NUMBER;
        L_b(9 * leg + j + 18 * i) = 0.0f;
      }
      U_b(9 * leg + 4 + 18 * i) = 0.0f;
      U_b(9 * leg + 5 + 18 * i) = 0.0f;
      U_b(9 * leg + 6 + 18 * i) = 0.0f;
      U_b(9 * leg + 7 + 18 * i) = 0.0f;
      U_b(9 * leg + 8 + 18 * i) = setup->f_max * update->gait[2 * i + leg];
      L_b(9 * leg + 4 + 18 * i) = -BIG_NUMBER;
      L_b(9 * leg + 5 + 18 * i) = -BIG_NUMBER;
      L_b(9 * leg + 6 + 18 * i) = -BIG_NUMBER;
      L_b(9 * leg + 7 + 18 * i) = -BIG_NUMBER;
      L_b(9 * leg + 8 + 18 * i) = 0.0f;
    }
  }

  // QP Lower Bound

  // Initalization of Line Contact Constraint Parameters
  // lt——length to toe
  // lh——length to heel
  // ll——length to left
  // lr——length to right
  fpt mu = 2.0;
  fpt lt = 0.17257;
  fpt lh = 0.09265;
  fpt fh = 0.073;
  fpt ll = 0.05;
  fpt lr = 0.05;

  // Matrix<fpt,5,3> f_block;
  Matrix<fpt, 10, 12> f_blockz;
  Matrix<fpt, 18, 12> F_control;

  Matrix<fpt, 1, 3> lt_vec;
  lt_vec << 0, 0, lt;

  Matrix<fpt, 1, 3> lh_vec;
  lh_vec << 0, 0, lh;

  Matrix<fpt, 1, 3> fh_vec;
  fh_vec << 0, fh, 0;

  Matrix<fpt, 1, 3> ll_vec;
  ll_vec << 0, 0, ll;

  Matrix<fpt, 1, 3> lr_vec;
  lr_vec << 0, 0, lr;

  Matrix<fpt, 1, 3> Mx_vec;
  Mx_vec << 0, 1.0, 0;

  Matrix<fpt, 1, 3> My_vec;
  My_vec << 1.0, 0, 0;

  Matrix<fpt, 1, 3> Moment_selection(1.f, 0, 0);

  // TODO: 修改足端Mx=0的约束，Hector采用Mx=0是由于足端近似为一条直线。
  // exo_humanoid由于足端存在宽度，因此Mx和My应该采用同样的约束方式。
  F_control.setZero();
  // leg 1
  F_control.block<1, 12>(0, 0) // Friction leg 1
      << -mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(1, 0)
      << mu, 0, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(2, 0)
      << 0, -mu, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(3, 0)
      << 0, mu, 1.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // F_control.block<1, 12>(4, 0) //Mx Leg 1
  //     << fh_vec * R_foot_L.transpose()* rs.R.transpose(), 0, 0, 0, Mx_vec * R_foot_L.transpose()* rs.R.transpose(),  0, 0, 0;
  F_control.block<1, 12>(4, 0) // Mx Leg 1
      << -lr_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0, Mx_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0;
  F_control.block<1, 12>(5, 0) 
      << -ll_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0, -Mx_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0;
  F_control.block<1, 12>(6, 0) // My Leg 1
      << -lt_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0, My_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0;
  F_control.block<1, 12>(7, 0) 
      << -lh_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0, -My_vec * R_foot_L.transpose() * rs.R.transpose(), 0, 0, 0;
  F_control.block<1, 12>(8, 0) // Fz Leg 1
      << 0, 0, 2.f, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // std::cout << "——————————————————R:——————————————————" <<std::endl;
  // std::cout <<  (R_foot_R.transpose()* rs.R.transpose())<<std::endl;
  // leg 2
  F_control.block<1, 12>(9, 0) // Friction leg 2
      << 0, 0, 0, -mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(10, 0)
      << 0, 0, 0, mu, 0, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(11, 0)
      << 0, 0, 0, 0, -mu, 1.f, 0, 0, 0, 0, 0, 0;
  F_control.block<1, 12>(12, 0)
      << 0, 0, 0, 0, mu, 1.f, 0, 0, 0, 0, 0, 0;
  // F_control.block<1, 12>(12, 0) //Mx Leg 1
  //     << 0, 0, 0, fh_vec * R_foot_R.transpose()* rs.R.transpose(), 0, 0, 0, Moment_selection * R_foot_R.transpose()* rs.R.transpose();
  F_control.block<1, 12>(13, 0) // Mx Leg 1
      << 0, 0, 0, -lr_vec * R_foot_R.transpose() * rs.R.transpose(), 0, 0, 0, Mx_vec * R_foot_R.transpose() * rs.R.transpose();
  F_control.block<1, 12>(14, 0) // Mx Leg 1
      << 0, 0, 0, -ll_vec * R_foot_R.transpose() * rs.R.transpose(), 0, 0, 0, -Mx_vec * R_foot_R.transpose() * rs.R.transpose();
  F_control.block<1, 12>(15, 0) // My Leg 2
      << 0, 0, 0, -lt_vec * R_foot_R.transpose() * rs.R.transpose(), 0, 0, 0, My_vec * R_foot_R.transpose() * rs.R.transpose();
  F_control.block<1, 12>(16, 0)
      << 0, 0, 0, -lh_vec * R_foot_R.transpose() * rs.R.transpose(), 0, 0, 0, -My_vec * R_foot_R.transpose() * rs.R.transpose();
  F_control.block<1, 12>(17, 0) // Fz Leg 2
      << 0, 0, 0, 0, 0, 2.f, 0, 0, 0, 0, 0, 0;

  // Set to fmat QP
  for (s16 i = 0; i < setup->horizon; i++)
  {
    fmat.block(i * 18, i * 12, 18, 12) = F_control;
  }
  // Construct K:
  Alpha_diag.resize(12, 12);
  Alpha_diag.setZero();

  for (s16 i = 0; i < 12; i++)
  {
    Alpha_diag.block(i, i, 1, 1) << update->Alpha_K[i];
  }
  for (s16 i = 0; i < setup->horizon; i++)
  {
    Alpha_rep.block(i * 12, i * 12, 12, 12) << Alpha_diag;
  }
  // Equivalent to Matlab Formulaion
  qH = 2 * (B_qp.transpose() * S * B_qp + Alpha_rep);
  qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

  // Calls function that sets parameters matrices in qpOASES types
  matrix_to_real(H_qpoases, qH, setup->horizon * 12, setup->horizon * 12);
  matrix_to_real(g_qpoases, qg, setup->horizon * 12, 1);
  matrix_to_real(A_qpoases, fmat, setup->horizon * 18, setup->horizon * 12);
  matrix_to_real(ub_qpoases, U_b, setup->horizon * 18, 1);
  matrix_to_real(lb_qpoases, L_b, setup->horizon * 18, 1);

  s16 num_constraints = 18 * setup->horizon;
  s16 num_variables = 12 * setup->horizon;

  // Max # of working set recalculations
  qpOASES::int_t nWSR = 500;

  int new_vars = num_variables;
  int new_cons = num_constraints;

  for (int i = 0; i < num_constraints; i++)
    con_elim[i] = 0;

  for (int i = 0; i < num_variables; i++)
    var_elim[i] = 0;

  for (int i = 0; i < num_constraints; i++)
  {
    if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
      continue;

    double *c_row = &A_qpoases[i * num_variables];

    for (int j = 0; j < num_variables; j++)
    {

      if (near_two(c_row[j]))
      {
        new_vars -= 6;
        new_cons -= 9;

        int cs;
        if (j % 2 == 0)
        {
          cs = (j + 4) / 6 * 9 - 1;
        }
        else
        {
          cs = (j + 1) / 6 * 9 + 8;
        }

        var_elim[j + 6] = 1;
        var_elim[j + 5] = 1;
        var_elim[j + 4] = 1;
        var_elim[j - 2] = 1;
        var_elim[j - 1] = 1;
        var_elim[j] = 1;

        con_elim[cs - 0] = 1;
        con_elim[cs - 1] = 1;
        con_elim[cs - 2] = 1;
        con_elim[cs - 3] = 1;
        con_elim[cs - 4] = 1;
        con_elim[cs - 5] = 1;
        con_elim[cs - 6] = 1;
        con_elim[cs - 7] = 1;
        con_elim[cs - 8] = 1;
      }
    }
  }

  // sync_out.println("newvars: ", new_vars);
  // sync_out.println("newcons: ", new_cons);
  // std::cout << "newvars" << new_vars << std::endl;
  // std::cout << "newcons" << new_cons << std::endl;

  if (1 == 1)
  {
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (!var_elim[i])
      {
        if (!(vc < new_vars))
        {
          sync_out.println("BAD ERROR 1");
          // printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      if (!con_elim[i])
      {
        if (!(vc < new_cons))
        {
          sync_out.println("BAD ERROR 2");
          // printf("BAD ERROR 2\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }
    for (int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      g_red[i] = g_qpoases[olda];
      for (int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for (int st = 0; st < new_vars; st++)
      {
        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
        A_red[con * new_vars + st] = cval;
      }
    }

    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }

    Timer solve_timer;

    // qpOASES problem
    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);

    // QS: QProblem::init中的cputime并没有生效？但通过qpOASES::getCPUtime()测试发现系统时间是可用的
    *max_time = 0.004;
    // printf("current time: %f s\n", qpOASES::getCPUtime());
    // QP initialized
    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR, max_time);
    // int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    (void)rval;

    int rval2 = problem_red.getPrimalSolution(q_red);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    {
      sync_out.println("failed to solve!");
      // printf("failed to solve!\n");
    }

    // sync_out.println("solve time: " ,solve_timer.getMs()," ms, size " ,new_vars ,", " ,new_cons);
    // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

    // Reformats solution and stores into q_red
    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (var_elim[i])
      {
        q_soln[i] = 0.0f;
      }
      else
      {
        q_soln[i] = q_red[vc];
        vc++;
      }
    }

    for (int i = 0; i < 12; i++)
    {
      U_opt(i) = q_soln[i];
    }
  }

  dX_cal = A_ct * x_0 + B_ct_r * U_opt;

#ifdef K_PRINT_EVERYTHING
  cout << "fmat:\n"
       << fmat << endl;
#endif
}

MpcOutput MPC_Solver::solve_mpc_threaded(update_data_t *update, problem_setup_t *setup)
{
  resize_qp_mats(setup->horizon);

  solve_mpc(update, setup);

  MpcOutput result;
  result.dX_cal = dX_cal;
  result.q_soln = q_soln;

  return result;
}