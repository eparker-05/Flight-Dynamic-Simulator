#include <iostream>
#include <cmath>
#include "AircraftParameters.h" //Including structure of parameters
#include "AircraftState.h" //Including structure of the states
using namespace std;
//Function Declarations 
AircraftParameters f5;
double compute_velocity(const AircraftState& state);
double compute_beta(double, const AircraftState& state);
double compute_alpha(const AircraftState& state);
double compute_CL(double);
double compute_CD(double);
double compute_CY(double);
double compute_drag(double, double);
double compute_lift(double, double);
double compute_sideforce(double, double);


int main(){
    AircraftState state; //defining state variable to pass through functions

    //Setting test velocity components
    state.u = 180.0; // m/s
    state.v = 0.01; // m/s
    state.w = 6.0; // m/s

    //Function calls
    double V = compute_velocity(state);
    double beta = compute_beta(V, state);
    double alpha = compute_alpha(state);
    double CL = compute_CL(alpha);
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

return 0;
}
//All function definitions
double compute_velocity(const AircraftState& state)
{
    double V = sqrt(state.u * state.u +
                    state.v * state.v + 
                    state.w * state.w);
    return V;
}
double compute_beta(double V, const AircraftState& state)
{
    return asin(state.v/state.u);
}
double compute_alpha(const AircraftState& state)
{
    return atan2(state.w, state.u);
}
double compute_CL(double alpha) //We use alpha instead of trim_alpha to create generic functions
{
    return f5.CL0 + f5.CL_alpha * alpha;
}
double compute_CD(double CL)
{
    double k = 0.05;
    return f5.CD0 + k * CL*CL; 
}
double compute_CY(double beta)
{
    return f5.CY_beta * beta;
}
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
