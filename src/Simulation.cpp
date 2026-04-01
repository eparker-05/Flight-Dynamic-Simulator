#include <iostream>
#include <cmath> 
#include <fstream>
#include <iomanip>
#include <vector>
#include "AircraftState.h"
#include "AircraftParameters.h"
#include "Aerodynamics.h"
#include "AerodynamicForces.h"
#include "Dynamics.h"
using namespace std;

struct StepPhysics
{
    double V;
    double alpha;
    double beta;

    AerodynamicForces forces;

    double Fx;
    double Fy;
    double Fz;

    double u_dot;
    double v_dot; 
    double w_dot;

    double Cm;
    double M;
    double q_dot;
    double theta_dot;
};

StepPhysics compute_step_physics(const AircraftState& state, double, double, double, double);
AircraftState update_state(const AircraftState& state, const StepPhysics& physics, double);
double compute_trim_alpha(AircraftParameters& f5);

int main(){
AircraftState state;
state.u =  179.936; // m/s
state.v = 0.00; // m/s 
state.w = 4.81234; // m/s

state.x = 0.0; // m
state.y = 0.0; // m
state.z = 10000.0; // m

double dt = 0.01; // s
double mass = 7000.0; // kg
double thrust = 11106.4; //N
double tol = 0.17; // rad

state.theta = 0.0267384;
state.q = 0.0;
double delta_e_trim = 0.009;
//AerodynamicForces forces;

double u_gust = 0.0;
double v_gust = 0.0;
//vector<double> gust = {u_gust, v_gust, w_gust};
double gust = 0.0;
cout << "Enter wind strength:" << endl;
cin >> gust;

ofstream outFile("simulation_output.csv");

outFile << "time,u,v,w,x,y,z,alpha,V,Fx,Fz,u_dot,w_dot,q,theta\n";
outFile << fixed << setprecision(6);

for(double t=0.0; t<=10; t+=0.01){
double w_gust = 0.0;
if(t>=2.0){
    w_gust = gust;
}
StepPhysics physics = compute_step_physics(state, w_gust, mass, thrust, delta_e_trim);
double alpha_rel = physics.alpha;

if (std::isnan(physics.V) || std::isnan(physics.alpha) || std::isnan(physics.beta) ||
    std::isnan(physics.Fx) || std::isnan(physics.Fy) || std::isnan(physics.Fz) ||
    std::isnan(physics.u_dot) || std::isnan(physics.v_dot) || std::isnan(physics.w_dot)) {
    cout << "NaN in physics at t = " << t << endl;
    cout << "u = " << state.u << endl;
    cout << "v = " << state.v << endl;
    cout << "w = " << state.w << endl;
    cout << "w_gust = " << w_gust << endl;
    break;
}

//-------Updating Velocity & Position States---------
state = update_state(state, physics, dt);
outFile << t << ","
        << state.u << ","
        << state.v << ","
        << state.w << ","
        << state.x << ","
        << state.y << ","
        << state.z << ","
        << physics.alpha << ","
        << physics.V << ","
        << physics.Fx << ","
        << physics.Fz << ","
        << physics.u_dot << ","
        << physics.w_dot << ","
        << state.q << ","
        << state.theta << "\n";

cout << "Updated Velocities: " << endl;
cout << "u_new = " << state.u << endl;
cout << "v_new = " << state.v << endl;
cout << "w_new = " << state.w << endl;
cout << "Updated Position: " << endl;
cout << "x = " << state.x << "m." << endl;
cout << "y = " << state.y << "m." << endl;
cout << "z = " << state.z << "m." << endl;



}
outFile.close();
return 0;
}
StepPhysics compute_step_physics(const AircraftState& state, double w_gust, double mass, double thrust, double delta_e)
{
    StepPhysics physics;

    double u_rel = state.u;
    double v_rel = state.v;
    double w_rel = state.w - w_gust;

    physics.V = sqrt(u_rel * u_rel + v_rel * v_rel + w_rel * w_rel);
    physics.alpha = atan2(w_rel, u_rel);
    physics.beta = asin(v_rel / physics.V);

    double CL = compute_CL(physics.alpha, delta_e);
    double CD = compute_CD(CL);
    double CY = compute_CY(physics.beta);
    

    physics.forces.drag = compute_drag(CD, physics.V);
    physics.forces.lift = compute_lift(CL, physics.V);
    physics.forces.sideforce = compute_sideforce(CY, physics.V);

    physics.Fx = compute_Fx(physics.forces, physics.alpha, thrust);
    physics.Fy = compute_Fy(physics.forces);
    physics.Fz = compute_Fz(physics.forces, physics.alpha, mass);

    const double g = 9.81;
    physics.u_dot = compute_u_dot(physics.Fx, mass, state.q, state.w);
    physics.v_dot = compute_v_dot(physics.Fy, mass);
    physics.w_dot = compute_w_dot(physics.Fz, mass, state.q, state.u);
    AircraftParameters f5;
    physics.Cm = compute_Cm(physics.alpha, state.q, physics.V, delta_e);
    physics.M = compute_pitch_moment(physics.Cm, physics.V);

    physics.q_dot = compute_q_dot(physics.M, f5.Iy);
    physics.theta_dot = compute_theta_dot(state.q);

    return physics;
}
AircraftState update_state(const AircraftState& state, const StepPhysics& physics, double dt)
{
    AircraftState new_state = state;
    new_state.u = state.u + physics.u_dot * dt;
    new_state.v = state.v + physics.v_dot * dt;
    new_state.w = state.w + physics.w_dot * dt;

    new_state.x = state.x + (state.u * cos(state.theta) - state.w * sin(state.theta)) * dt;
    new_state.y = state.y + state.v * dt;
    new_state.z = state.z + (state.u * sin(state.theta) + state.w * cos(state.theta)) * dt;

    new_state.q = state.q + physics.q_dot * dt;
    new_state.theta = state.theta + physics.theta_dot * dt;

    return new_state;
}
