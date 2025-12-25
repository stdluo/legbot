#include "ProblemSetupAndUpdate.h"
#include "../include/common/Math/orientation_tools.h"
#include "RobotStateMpc.h"

void problem_setup_t::setup_problem(double dt, int horizon, double mu, double f_max)
{
#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
            horizon,f_max,mu,dt);
#endif

  this->horizon = horizon;
  this->f_max = f_max;
  this->mu = mu;
  this->dt = dt;
}

void problem_setup_t::print_problem_setup(void)
{
  printf("DT: %.3f\n", dt);
  printf("Mu: %.3f\n", mu);
  printf("F_Max: %.3f\n", f_max);
  printf("Horizon: %d\n", horizon);
}

void update_data_t::update_problem_data(update_data_t &update_data)
{
    for (int i = 0; i < 3; i++)
    {
        p[i] = update_data.p[i];
        v[i] = update_data.v[i];
        q[i] = update_data.q[i];
        w[i] = update_data.w[i];
        v_des_world[i] = update_data.v_des_world[i];
    }

    for (int i = 0; i < 6; i++)
    {
        r[i] = update_data.r[i];
    }

    q[3] = update_data.q[3];
    yaw = update_data.yaw;
    wz_des = update_data.wz_des;

    for (int i = 0; i < 120; i++)
    {
        traj[i] = update_data.traj[i];
    }

    for(int i = 0; i < 20; i++)
    {
        gait[i] = update_data.gait[i];
    }

    for(int i = 0; i < 2; i++)
    {
        R_foot[i] = update_data.R_foot[i];
    }
}

void update_data_t::update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* traj, int* gait, Mat3<float>* R_foot, Vec3<float> p_com, Mat3<float> Interia)
{
    for (int i = 0; i < 3; i++)
    {
        this->p[i] = p[i];
        this->v[i] = v[i];
        this->q[i] = q[i];
        this->w[i] = w[i];
        this->p_com[i] = p_com[i];
        this->offset[i] = p_com[i] - p[i];
    }

    for (int i = 0; i < 6; i++)
    {
        this->r[i] = r[i];
    }

    this->q[3] = q[3];
    this->yaw = yaw;

    for (int i = 0; i < 120; i++)
    {
        this->traj[i] = traj[i];
    }

    for (int i = 0; i < 10; i++)
    {
      // Transform the position from the trunk frame to the center of mass frame
      this->traj[12*i +3] += this->offset(0);
      this->traj[12*i +4] += this->offset(1);
      this->traj[12*i +5] += this->offset(2);
      // this->traj[12*i +3] += RobotStateMpc().offset(0);
      // this->traj[12*i +4] += RobotStateMpc().offset(1);
      // this->traj[12*i +5] += RobotStateMpc().offset(2);
    }

    for(int i = 0; i < 20; i++)
    {
        this->gait[i] = gait[i];
    }

    for(int i = 0; i < 2; i++)
    {
        this->R_foot[i] = R_foot[i];
    }

    this->Interia = Interia;
}

void update_data_t::update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp) {
  this->max_iterations = max_iter;
  this->rho = rho;
  this->sigma = sigma;
  this->solver_alpha = solver_alpha;
  this->terminate = terminate;
}

void update_data_t::print_update_data(s16 horizon)
{
  print_named_array("p", p, 1, 3);
  print_named_array("v", v, 1, 3);
  print_named_array("q", q, 1, 4);
  print_named_array("w", w, 1, 3);
  print_named_array("r", r, 3, 2);
  pnv("Yaw", yaw);
  print_named_array("weights", weights, 1, 12);
  print_named_array("trajectory", traj, horizon, 12);
  print_named_array("Alpha", Alpha_K, 1, 12);
  print_named_array("gait", gait, horizon, 2);
}

void update_data_t::update_traj_all()
{
    Eigen::Quaternionf quat;
    Vec3<float> rpy;
    quat.w() = q[0];
    quat.x() = q[1];
    quat.y() = q[2];
    quat.z() = q[3];
    quat_to_rpy(quat, rpy);

    for (int i = 0; i < 10; i++)
    {
      for (int j = 0; j < 12; j++)
        traj[12 * i + j] = trajinit[j];

      if(i == 0) // start at current position  TODO consider not doing this
      {
        traj[0] = rpy[0];
        traj[1] = rpy[1];
        traj[2] = rpy[2];
        traj[3] = p[0];
        traj[4] = p[1];
        traj[5] = p[2];
      }
      else
      {
        if (v_des_world[0] == 0) {
        traj[12*i + 3] = trajinit[3] + i * dtMPC * v_des_world[0];
        }
        else{
         traj[12*i + 3] = p[0] + i * dtMPC * v_des_world[0]; 
        }
        if (v_des_world[1] == 0) {
        traj[12*i + 4] = trajinit[4] + i * dtMPC * v_des_world[1];
        }
        else{
         traj[12*i + 4] = p[1] + i * dtMPC * v_des_world[1]; 
        }
        if (wz_des == 0){
        traj[12*i + 2] = trajinit[2];
         }
        else{
        // traj[12*i + 2] = yaw + i * dtMPC * wz_des;
        // std::cout << "yaw traj" <<  traj[12*i + 2] << std::endl;
        }
      }

      // Transform the position from the trunk frame to the center of mass frame
      traj[12*i +3] += RobotStateMpc().offset(0);
      traj[12*i +4] += RobotStateMpc().offset(1);
      traj[12*i +5] += RobotStateMpc().offset(2);

      // traj[12*i +3] += offset(0);
      // traj[12*i +4] += offset(1);
      // traj[12*i +5] += offset(2);

      // std::cout << "traj " << i << std::endl;
      // for (int j = 0; j < 12; j++) {
      //   std::cout << traj[12 * i + j] << "  ";
      // }
      // std::cout<< " " <<std::endl;

    }
}