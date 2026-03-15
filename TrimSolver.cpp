#include <iostream>
#include <cmath>
#include "AircraftParameters.h" //File containing parameter struct
#include "AircraftState.h" //File containing state struct
using namespace std;

//Function declarations 
double compute_trim_alpha(AircraftParameters& f5);
double compute_CD(double, AircraftParameters& f5);
double compute_CL(double, AircraftParameters& f5);

//Trim solver constants 
//const double v_trim = 180.0; // m/s
const double g = 9.81; // m/s^2

int main()
{
    AircraftParameters f5;
    //Computing (calling) our constants and alpha
    double trim_alpha = compute_trim_alpha(f5);
    double CL = compute_CL(trim_alpha, f5);
    double CD = compute_CD(CL, f5);
    double V = 180.0; // m/s
    double Drag = 0.5 * f5.rho * f5.v_trim * f5.v_trim * CD * f5.wingArea;
    double Thrust_req = Drag; //Condition required for steady flight

    //Printing results
    cout << "The correct angle of attack is: " << trim_alpha << " rad." << endl;
    cout << "CL = " << CL << endl;
    cout << "CD = " << CD << endl;
    cout << "The required thrust is: " << Thrust_req << "N." << endl;
   
    

return 0;
}
//Function definitions
double compute_CD(double CL, AircraftParameters& f5)
{
    double k = 0.05;
    return f5.CD0 + k * CL*CL; 
}
double compute_CL(double alpha, AircraftParameters& f5)
{
    return f5.CL0 + f5.CL_alpha * alpha;
}
double compute_trim_alpha(AircraftParameters& f5)
{
double alpha = 0.05; //Setting initial guess
double tol = 100; // N; with respect to the lift
double Weight = f5.mass * g;

for(int i = 0; i < 10000; i++){
double CL = compute_CL(alpha,f5);
double Lift = 0.5 * f5.rho * f5.v_trim * f5.v_trim* CL * f5.wingArea;
double error = Lift - Weight; //Condition required for steady flight

if (fabs(error) < tol){ //Stops when we get close enough (within 100N)
    break;
}
alpha = alpha - 0.0000001 * error; //Updating our alpha each time

}
return alpha;
}
