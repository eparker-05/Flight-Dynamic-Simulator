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
//structure containing all variables tht need updating 
struct StepPhysics
{
    double V;
    double alpha;
    double beta;
    double p_hat;
    double r_hat;

    AerodynamicForces forces;

    double Fx;
    double Fy;
    double Fz;

    double u_dot;
    double v_dot; 
    double w_dot;
    double w_rel;

    double Cm;
    double M;
    double q_dot;
    double theta_dot;

    double Cl;
    double L;
    double p_dot;
    double phi_dot;

    double Cn;
    double N;
    double r_dot;
    double psi_dot;
};

StepPhysics compute_step_physics(const AircraftState& state, const AircraftParameters& f5, double, double, double, double);
AircraftState update_state(const AircraftState& state, const StepPhysics& physics, double);
double compute_trim_alpha(AircraftParameters& f5);

int main(){
AircraftState state;
AircraftParameters f5;
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

state.theta = 0.0267384; //rad
state.q = 0.0;
state.p = 0.0;
state.r = 0.01;
state.psi = 0.0; //rad
state.phi = 0.0; //rad
double delta_e_trim = 0.009;
double theta_command = 0.0267384;
double delta_e = delta_e_trim;
double Kp_theta = 0.015;
double Kd = 0.04;
//AerodynamicForces forces;

double u_gust = 0.0;
double v_gust = 0.0;
//vector<double> gust = {u_gust, v_gust, w_gust};
double gust = 0.0;
cout << "Enter wind strength:" << endl;
cin >> gust;

ofstream outFile("simulation_output.csv");

//Beginning our CSV file
outFile << "time,u,v,w,x,y,z,alpha,beta,V,Fx,Fz,Fy,u_dot,w_dot,q,theta, delta_e, p, phi_dot, Cl, phi, r, Cn, N, psi_dot, r_dot, psi, w_rel, Cm\n";
outFile << fixed << setprecision(6);

for(double t=0.0; t<=10; t+=0.01){
double w_gust = 0.0;
if(t>=2.0){
    w_gust = gust;
}

double theta_error = theta_command - state.theta;

// P-D controller
double delta_e_correction = Kp_theta * theta_error - Kd * state.q;

//total elevator command = trim + correction
delta_e = delta_e_trim + delta_e_correction;

// optional elevator saturation
if (delta_e > 0.35) delta_e = 0.35;
if (delta_e < -0.35) delta_e = -0.35;

StepPhysics physics = compute_step_physics(state, f5, w_gust, mass, thrust, delta_e);
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

//---------List of all printed variables--------
outFile << t << ","
        << state.u << ","
        << state.v << ","
        << state.w << ","
        << state.x << ","
        << state.y << ","
        << state.z << ","
        << physics.alpha << ","
        << physics.beta << ","
        << physics.V << ","
        << physics.Fx << ","
        << physics.Fz << ","
        << physics.Fy << ","
        << physics.u_dot << ","
        << physics.w_dot << ","
        << state.q << ","
        << state.theta << ","
        << delta_e << "," 
        << state.p << ","
        << physics.phi_dot << ","
        << physics.Cl << ","
        << state.phi << ","
        << state.r << ","
        << physics.Cn << ","
        << physics.N << ","
        << physics.psi_dot << ","
        << physics.r_dot << ","
        << state.psi << ","
        << physics.w_rel << ","
        << physics.Cm << "\n";

}
outFile.close(); //Closing the CSV file
return 0;
}
//This massive thing is where I update all of my forces and parameters each iteration
StepPhysics compute_step_physics(const AircraftState& state, const AircraftParameters& f5, double w_gust, double mass, double thrust, double delta_e)
{
    StepPhysics physics;

    double u_rel = state.u;
    double v_rel = state.v;
    physics.w_rel = state.w - w_gust;

    physics.V = sqrt(u_rel * u_rel + v_rel * v_rel + physics.w_rel * physics.w_rel);
    physics.alpha = atan2(physics.w_rel, u_rel);
    physics.beta = asin(v_rel / physics.V);
    physics.p_hat = compute_p_hat(state, f5, physics.V);
    physics.r_hat = compute_r_hat(f5, state, physics.V);

    double CL = compute_CL(physics.alpha, delta_e);
    double CD = compute_CD(CL);
    double CY = compute_CY(physics.beta);
    

    physics.forces.drag = compute_drag(CD, physics.V);
    physics.forces.lift = compute_lift(CL, physics.V);
    physics.forces.sideforce = compute_sideforce(CY, physics.V);

    physics.Fx = compute_Fx(physics.forces, physics.alpha, thrust);
    physics.Fy = compute_Fy(physics.forces);
    physics.Fz = compute_Fz(physics.forces, physics.alpha, mass);

    physics.u_dot = compute_u_dot(physics.Fx, mass, state.q, state.w, state.r, state.v);
    physics.v_dot = compute_v_dot(physics.Fy, mass, state.p, state.w, state.r, state.u);
    physics.w_dot = compute_w_dot(physics.Fz, mass, state.q, state.u, state.p, state.v);
    physics.Cm = compute_Cm(physics.alpha, state.q, physics.V, delta_e);
    physics.M = compute_pitch_moment(physics.Cm, physics.V);
    physics.Cl = compute_Cl(f5, physics.beta, physics.p_hat);
    physics.L = compute_roll_moment(f5, physics.V, physics.Cl);
    physics.Cn = compute_Cn(f5, physics.beta, physics.r_hat);
    physics.N = compute_yaw_moment(f5, physics.V, physics.Cn);

    physics.q_dot = compute_q_dot(physics.M, f5.Ix, f5.Iy, f5.Iz, state.p, state.r);
    physics.theta_dot = compute_theta_dot(state.q, state, state.r);
    physics.p_dot = compute_p_dot(physics.L, f5.Ix, f5.Iy, f5.Iz, state.q, state.r);
    physics.phi_dot = compute_phi_dot(state.p, state.q, state.r, state);
    physics.r_dot = compute_r_dot(physics.N, f5.Ix, f5.Iy, f5.Iz, state.p, state.q);
    physics.psi_dot = compute_psi_dot(state.r, state.q, state);

    return physics;
}
//This is where I specifically update the aircraft's state - not forces
AircraftState update_state(const AircraftState& state, const StepPhysics& physics, double dt)
{
    AircraftState new_state = state;
    new_state.u = state.u + physics.u_dot * dt;
    new_state.v = state.v + physics.v_dot * dt;
    new_state.w = state.w + physics.w_dot * dt;

    double x_dot = compute_x_dot(state, state.u, state.v, state.w);
    double y_dot = compute_y_dot(state, state.u, state.v, state.w);
    double z_dot = compute_z_dot(state, state.u, state.v, state.w);
    new_state.x = state.x + x_dot * dt;
    new_state.y = state.y + y_dot * dt;
    new_state.z = state.z + z_dot * dt;

    new_state.q = state.q + physics.q_dot * dt;
    new_state.theta = state.theta + physics.theta_dot * dt;
    new_state.p = state.p + physics.p_dot * dt;
    new_state.phi = state.phi + physics.phi_dot * dt;
    new_state.r = state.r + physics.r_dot * dt;
    new_state.psi = state.psi + physics.psi_dot * dt;

    return new_state;
}
