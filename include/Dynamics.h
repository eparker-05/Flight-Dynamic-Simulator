#ifndef Dynamics_h
#define Dynamics_h

#include "AerodynamicForces.h"

double compute_Fx(const AerodynamicForces& f, double alpha, double thrust);
double compute_Fy(const AerodynamicForces& f);
double compute_Fz(const AerodynamicForces& f, double alpha, double mass);

double compute_u_dot(double Fx, double mass, double q, double w, double r, double v);
double compute_v_dot(double Fy, double mass, double p, double w, double r, double u);
double compute_w_dot(double Fz, double mass, double q, double u, double p, double v);

double compute_q_dot(double M, double Ix, double Iy, double Iz, double p, double r);
double compute_theta_dot(double q, const AircraftState& state, double r);
double compute_p_dot(double L, double Ix, double Iy, double Iz, double q, double r);
double compute_phi_dot(double p, double q, double r, const AircraftState& state);
double compute_r_dot(double N, double Ix, double Iy, double Iz, double p, double q);
double compute_psi_dot(double r, double q, const AircraftState& state);
double compute_x_dot(const AircraftState& state, double u, double v, double w);
double compute_y_dot(const AircraftState& state, double u, double v, double w);
double compute_z_dot(const AircraftState& state, double u, double v, double w);
#endif
