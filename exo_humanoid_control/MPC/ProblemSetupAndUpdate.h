#ifndef PROBLEM_SETUP_AND_UPDATE_H
#define PROBLEM_SETUP_AND_UPDATE_H

#define K_MAX_GAIT_SEGMENTS 36

#include "../include/common/cppTypes.h"
#include <iostream>
#include "common_types.h"

template <class T>
void print_array(T* array, u16 rows, u16 cols)
{
    for(u16 r = 0; r < rows; r++)
    {
        for(u16 c = 0; c < cols; c++)
            std::cout<<(fpt)array[c+r*cols]<<" ";
        printf("\n");
    }
}

template <class T>
void print_named_array(const char* name, T* array, u16 rows, u16 cols)
{
    printf("%s:\n",name);
    print_array(array,rows,cols);
}

//print named variable
template <class T>
void pnv(const char* name, T v)
{
    printf("%s: ",name);
    std::cout<<v<<std::endl;
}

class problem_setup_t
{
  public:
    // dtMPC = dt(0.001) * iterationsBetweenMPC(40)
    float dt = 0.04;
    float mu = 0.25;
    float f_max = 1000;
    int horizon = 10;
    
    void setup_problem(double dt, int horizon, double mu, double f_max);
    void print_problem_setup();
};

class update_data_t
{
  public:
    // feedback
    float p[3];
    float v[3];
    float q[4];
    float w[3];
    float r[6];
    float yaw;
    // float weights[12] = {250, 1000, 500,  3000, 1000, 2000,  1, 1, 1,  1, 1, 1}; 
    // float Alpha_K[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    float weights[12] = {100, 100, 250,  200, 200, 300,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    float Alpha_K[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    unsigned char gait[2 * K_MAX_GAIT_SEGMENTS];
    Mat3<float> R_foot[2];

    Vec3<float> offset;
    Vec3<float> p_com;
    Mat3<float> Interia;

    // target
    float traj[12 * K_MAX_GAIT_SEGMENTS];
    float trajinit[12];
    float v_des_world[3];
    float wz_des;

    float dtMPC = 0.04;
    
    int max_iterations;
    double rho, sigma, solver_alpha, terminate;

    void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
    void update_problem_data(update_data_t &update_data);
    void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* traj, int* gait, Mat3<float>* R_foot, Vec3<float> p_com, Mat3<float> Interia);
    void print_update_data(s16 horizon);

    void update_traj_all();
};

struct MpcOutput
{
    Eigen::Matrix<fpt, 13, 1> dX_cal;
    mfp *q_soln = nullptr;
};



#endif