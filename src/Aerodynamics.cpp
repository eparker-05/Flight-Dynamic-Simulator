#include <iostream>
#include <cmath>
#include "AircraftParameters.h" //Including structure of parameters
#include "AircraftState.h" //Including structure of the states
#include "AerodynamicForces.h"
using namespace std;
//Function Declarations 
AircraftParameters f5;
double compute_velocity(const AircraftState& state);
double compute_beta(double, const AircraftState& state);
double compute_alpha(const AircraftState& state);
double compute_CL(double, double);
double compute_CD(double);
double compute_CY(double);
double compute_drag(double, double);
double compute_lift(double, double);
double compute_sideforce(double, double);
double compute_Cm(double, double, double, double);
double compute_pitch_moment(double, double);
double compute_p_hat(const AircraftState& state, const AircraftParameters& f5, double);
double compute_Cl(const AircraftParameters& f5, double, double);
double compute_roll_moment(const AircraftParameters& f5, double, double);
double compute_r_hat(const AircraftParameters& f5, const AircraftState& state, double);
double compute_Cn(const AircraftParameters& f5, double, double);
double compute_yaw_moment(const AircraftParameters& f5, double, double);
AerodynamicForces compute_forces(const AircraftState& state);

void print(){
    AircraftState state; //defining state variable to pass through functions

    //Setting test velocity components
    state.u = 179.932; // m/s
    state.v = 0.0; // m/s
    state.w = 4.951; // m/s
    double delta_e = 0.0133725;

    //Function calls
    double V = compute_velocity(state);
    double beta = compute_beta(V, state);
    double alpha = compute_alpha(state);
    double CL = compute_CL(alpha, delta_e);
    double CD = compute_CD(CL);
    double CY = compute_CY(beta);
    double drag = compute_drag(CD, V);
    double lift = compute_lift(CL, V);
    double sideforce = compute_sideforce(CY, V);

    //Printing results
    cout << "The aircraft's velocity is " << V << endl;
    cout << "Beta is " << beta << endl;
    cout << "The angle of attack is " << alpha << endl;
    cout << "CL is " << CL << endl;
    cout << "CD is " << CD << endl;
    cout << "CY is " << CY << endl;
    cout << "The drag force is " << drag << "N." << endl;
    cout << "The lift is " << lift << "N."<< endl;
    cout << "The sideforce is " << sideforce << "N." << endl;


//return 0;
}
//-----------Velocity-------------
double compute_velocity(const AircraftState& state)
{
    double V = sqrt(state.u * state.u +
                    state.v * state.v + 
                    state.w * state.w);
    return V;
}

//-----------Angles----------

double compute_beta(double V, const AircraftState& state)
{
    return asin(state.v/V);
}
double compute_alpha(const AircraftState& state)
{
    return atan2(state.w, state.u);
}

//------------Aerodynamic Coefficients---------------

double compute_CL(double alpha, double delta_e)
{
    return f5.CL0 + f5.CL_alpha * alpha + f5.CL_delta_e * delta_e;
}
//^We use alpha instead of trim_alpha to create generic functions^
double compute_CD(double CL)
{
    double k = 0.05;
    return f5.CD0 + k * CL*CL; 
}
double compute_CY(double beta)
{
    return f5.CY_beta * beta;
}

//------------Forces------------

double compute_drag(double CD, double V)
{
    return 0.5 * f5.rho * V * V * f5.wingArea * CD;
}
double compute_lift(double CL, double V)
{
    return 0.5 * f5.rho * V * V * f5.wingArea * CL;
}
double compute_sideforce(double CY, double V)
{
    return 0.5 * f5.rho * V * V * f5.wingArea * CY;
}
double compute_Cm(double alpha, double q, double V, double delta_e)
{
    double q_hat = 0.0;

    if (V > 1e-8) {
        q_hat = (f5.chord / (2.0 * V)) * q;
    }

    return f5.Cm0 + f5.Cm_alpha * alpha + f5.Cm_q * q_hat + f5.Cm_delta_e * delta_e;
}

double compute_pitch_moment(double Cm, double V)
{
    return 0.5 * f5.rho * V * V * f5.wingArea * f5.chord * Cm;
}
double compute_p_hat(const AircraftState& state, const AircraftParameters& f5, double V)
{
    return (f5.wingspan / (2 * V)) * state.p;
}
double compute_Cl(const AircraftParameters& f5, double beta, double p_hat)
{
    return f5.Cl_beta * beta + f5.Cl_p * p_hat;
}
double compute_roll_moment(const AircraftParameters& f5, double V, double Cl)
{
    return 0.5 * f5.rho * f5.wingArea * V * V * f5.wingspan * Cl;
}
double compute_r_hat(const AircraftParameters& f5, const AircraftState& state, double V)
{
    return (state.r * f5.wingspan) / (2 * V);
}
double compute_Cn(const AircraftParameters& f5, double beta, double r_hat)
{
    return f5.Cn_beta * beta + f5.Cn_r * r_hat;
}
double compute_yaw_moment(const AircraftParameters& f5, double V, double Cn)
{
    return 0.5 * f5.rho * f5.wingArea * f5.wingspan * V * V * Cn;
}
