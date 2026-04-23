#include <iostream> 
#include <cmath> 
#include "AircraftParameters.h"
#include "AircraftState.h"
#include "Aerodynamics.h"
#include "AerodynamicForces.h"
#include "Dynamics.h"

using namespace std;

void printdynamics(){

AircraftState state; 

state.u = 179.932; // m/s
state.v = 0.0; // m/s
state.w = 4.951; // m/s
double delta_e = 0.009;

AerodynamicForces forces;

double V = compute_velocity(state);
double alpha = compute_alpha(state);
double beta = compute_beta(V, state);

double CL = compute_CL(alpha, delta_e);
double CD = compute_CD(CL);
double CY = compute_CY(beta);

forces.drag = compute_drag(CD, V);
forces.lift = compute_lift(CL, V);
forces.sideforce = compute_sideforce(CY, V);

//AerodynamicForces forces = compute_forces(state);

double thrust = 11106.4; //N
double mass = 7000.0; //kg

//double alpha = compute_alpha(state);
double Fx = compute_Fx(forces, alpha, thrust);
double Fy = compute_Fy(forces);
double Fz = compute_Fz(forces, alpha, mass);

//Printing results
cout << "The force along the x body axis is " << Fx << "N." << endl;
cout << "The force along the y body axis is " << Fy << "N." << endl;
cout << "The force along the z body axis is " << Fz << "N." << endl;

//Initiallizing values to 0
double q = 0.0;
double p = 0.0;
double r = 0.0;
double u_dot = compute_u_dot(Fx, mass, q, state.w, r, state.v);
double v_dot = compute_v_dot(Fy, mass, p, state.w, r, state.u);
double w_dot = compute_w_dot(Fz, mass, q, state.u, p, state.v);

//Printing computed accelerations
cout << "Accelerations: " << endl;
cout << "u_dot = " << u_dot << endl;
cout << "v_dot = " << v_dot << endl;
cout << "w_dot = " << w_dot << endl;


}

//Function definitions to compute body forces/state derivatives 
double compute_Fx(const AerodynamicForces& f, double alpha, double thrust)
{
return thrust - f.drag * cos(alpha) - f.lift * sin(alpha);
}
double compute_Fy(const AerodynamicForces& f)
{
return f.sideforce;
}
double compute_Fz(const AerodynamicForces& f, double alpha, double mass)
{
return - f.drag * sin(alpha) - mass * 9.81 + f.lift * cos(alpha);
}

double compute_u_dot(double Fx, double mass, double q, double w, double r, double v)
{
    return Fx / mass - q * w + r * v;
}
double compute_v_dot(double Fy, double mass, double p, double w, double r, double u)
{
    return Fy / mass + p * w - r * u;
}
double compute_w_dot(double Fz, double mass, double q, double u, double p, double v)
{
    return Fz / mass + q * u - p * v;
}
double compute_q_dot(double M, double Ix, double Iy, double Iz, double p, double r)
{
    return (M - (Ix - Iz) * p * r) / Iy;
}

double compute_theta_dot(double q, const AircraftState& state, double r)
{
    return q * cos(state.phi) - r * sin(state.phi);
}
double compute_p_dot(double L, double Ix, double Iy, double Iz, double q, double r)
{
    return (L - ( Iz - Iy) * q * r) / Ix;
}
double compute_phi_dot(double p, double q, double r, const AircraftState& state)
{
    return p + q * sin(state.phi) * tan(state.theta) + r * cos(state.phi) * tan(state.theta);
}
double compute_r_dot(double N, double Ix, double Iy, double Iz, double p, double q)
{
    return (N - (Iy - Ix) * p * q) / Iz;
}
double compute_psi_dot(double r, double q, const AircraftState& state)
{
    return (r * cos(state.phi ) + q * sin(state.phi)) / cos(state.theta);
}
double compute_x_dot(const AircraftState& state, double u, double v, double w)
{
    return u * cos(state.theta) * cos(state.psi) + v * (sin(state.phi) * sin(state.theta) * cos(state.psi) - cos(state.phi) * sin(state.psi)) + w * ( cos(state.phi) * sin(state.theta) * cos(state.psi) + sin(state.phi) * sin(state.psi));
}
double compute_y_dot(const AircraftState& state, double u, double v, double w)
{
    return u * cos(state.theta) * sin(state.psi) + v * (sin(state.phi) * sin(state.theta) * sin(state.psi) + cos(state.phi) * cos(state.psi)) + w * (cos(state.phi) * sin(state.theta) * sin(state.psi) - sin(state.phi) * cos(state.psi));
}
double compute_z_dot(const AircraftState& state, double u, double v, double w)
{
    return - u * sin(state.theta) + v * sin(state.phi) * cos(state.theta) + w * cos(state.phi) * cos(state.theta);
}
