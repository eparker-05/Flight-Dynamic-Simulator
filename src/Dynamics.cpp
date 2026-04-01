#include <iostream> 
#include <cmath> 
#include "AircraftParameters.h"
#include "AircraftState.h"
#include "Aerodynamics.h"
#include "AerodynamicForces.h"

using namespace std;

double compute_Fx(AerodynamicForces& f, double, double);
double compute_Fy(AerodynamicForces& f);
double compute_Fz(AerodynamicForces& f, double, double);
double compute_u_dot(double, double, double, double);
double compute_v_dot(double, double);
double compute_w_dot(double, double, double, double);
double compute_q_dot(double, double);
double compute_theta_dot(double);

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

cout << "The force along the x body axis is " << Fx << "N." << endl;
cout << "The force along the y body axis is " << Fy << "N." << endl;
cout << "The force along the z body axis is " << Fz << "N." << endl;

double q = 0.0;
double u_dot = compute_u_dot(Fx, mass, q, state.w);
double v_dot = compute_v_dot(Fy, mass);
double w_dot = compute_w_dot(Fz, mass, q, state.u);

cout << "Accelerations: " << endl;
cout << "u_dot = " << u_dot << endl;
cout << "v_dot = " << v_dot << endl;
cout << "w_dot = " << w_dot << endl;


}

double compute_Fx(AerodynamicForces& f, double alpha, double thrust)
{
return thrust - f.drag * cos(alpha) - f.lift * sin(alpha);
}
double compute_Fy(AerodynamicForces& f)
{
return f.sideforce;
}
double compute_Fz(AerodynamicForces& f, double alpha, double mass)
{
return - f.drag * sin(alpha) - mass * 9.81 + f.lift * cos(alpha);
}
//double AerodynamicForces compute_forces(const AircraftState& state)
//{
    
 //   return forces;
//}
double compute_u_dot(double Fx, double mass, double q, double w)
{
    return Fx / mass - q * w;
}
double compute_v_dot(double Fy, double mass)
{
    return Fy / mass;
}
double compute_w_dot(double Fz, double mass, double q, double u)
{
    return Fz / mass + q * u;
}
double compute_q_dot(double M, double Iy)
{
    return M / Iy;
}

double compute_theta_dot(double q)
{
    return q;
}
