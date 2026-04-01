#ifndef Dynamics_h
#define Dynamics_h

#include "AerodynamicForces.h"

double compute_Fx(AerodynamicForces& f, double alpha, double thrust);
double compute_Fy(AerodynamicForces& f);
double compute_Fz(AerodynamicForces& f, double alpha, double mass);

double compute_u_dot(double Fx, double mass, double q, double w);
double compute_v_dot(double Fy, double mass);
double compute_w_dot(double Fz, double mass, double q, double u);

double compute_q_dot(double M, double Iy);
double compute_theta_dot(double q);
#endif
