#include <iostream>
#include <cmath>
#include "AircraftParameters.h"
#include "AircraftState.h"

using namespace std;

const double g = 9.81; //gravity constant

// Function declarations
double compute_CL(double alpha, double delta_e, AircraftParameters& f5);
double compute_CD(double CL, AircraftParameters& f5);
double compute_Cm(double alpha, double delta_e, AircraftParameters& f5);
double compute_pitch_moment(double Cm, AircraftParameters& f5);
double compute_Fz_trim(double alpha, double Lift, double Drag, double Weight);
double compute_thrust_req(double Drag, double Lift, double alpha);

int main()
{
    AircraftParameters f5;

    double alpha = 0.03;     // initial guess, rad
    double delta_e = 0.0;    // elevator trim guess, rad

    double Weight = f5.mass * g; // N
    double tol_Fz = 10.0;      // N
    double tol_M  = 100.0;     // N*m

    for (int i = 0; i < 20000; i++) {
        double CL = compute_CL(alpha, delta_e, f5);
        double CD = compute_CD(CL, f5);
        double Cm = compute_Cm(alpha, delta_e, f5);

        double qbar = 0.5 * f5.rho * f5.v_trim * f5.v_trim;
        double Lift = qbar * f5.wingArea * CL;
        double Drag = qbar * f5.wingArea * CD;
        double Fz = compute_Fz_trim(alpha, Lift, Drag, Weight);
        double M = compute_pitch_moment(Cm, f5);

        if (fabs(Fz) < tol_Fz && fabs(M) < tol_M) {
            break;
        }

        // Simple two-variable iteration:
        // alpha mostly fixes vertical force
        // delta_e mostly fixes pitching moment
        alpha   -= 1.0e-8 * Fz;
        delta_e -= 2.0e-8 * M;

        // Keep the guesses in a reasonable range
        if (alpha > 0.20) alpha = 0.20;
        if (alpha < -0.10) alpha = -0.10;

        if (delta_e > 0.35) delta_e = 0.35;
        if (delta_e < -0.35) delta_e = -0.35;
    }

    double CL = compute_CL(alpha, delta_e, f5);
    double CD = compute_CD(CL, f5);
    double Cm = compute_Cm(alpha, delta_e, f5);

    double qbar = 0.5 * f5.rho * f5.v_trim * f5.v_trim;
    double Lift = qbar * f5.wingArea * CL;
    double Drag = qbar * f5.wingArea * CD;
    double M = compute_pitch_moment(Cm, f5);
    double Fz = compute_Fz_trim(alpha, Lift, Drag, Weight);
    double Thrust_req = compute_thrust_req(Drag, Lift, alpha);

    double u_trim = f5.v_trim * cos(alpha);
    double v_trim = 0.0;
    double w_trim = f5.v_trim * sin(alpha);

    cout << "Trim results:" << endl;
    cout << "alpha_trim = " << alpha << " rad" << endl;
    cout << "delta_e_trim = " << delta_e << " rad" << endl;
    cout << "CL = " << CL << endl;
    cout << "CD = " << CD << endl;
    cout << "Cm = " << Cm << endl;
    cout << "Fz = " << Fz << " N" << endl;
    cout << "M = " << M << " N*m" << endl;
    cout << "Thrust_req = " << Thrust_req << " N" << endl;
    cout << "u_trim = " << u_trim << " m/s" << endl;
    cout << "v_trim = " << v_trim << " m/s" << endl;
    cout << "w_trim = " << w_trim << " m/s" << endl;

    return 0;
}

double compute_CL(double alpha, double delta_e, AircraftParameters& f5)
{
    return f5.CL0 + f5.CL_alpha * alpha + f5.CL_delta_e * delta_e;
}

double compute_CD(double CL, AircraftParameters& f5)
{
    double k = 0.05;
    return f5.CD0 + k * CL * CL;
}

double compute_Cm(double alpha, double delta_e, AircraftParameters& f5)
{
    return f5.Cm0 + f5.Cm_alpha * alpha + f5.Cm_delta_e * delta_e;
}

double compute_pitch_moment(double Cm, AircraftParameters& f5)
{
    double qbar = 0.5 * f5.rho * f5.v_trim * f5.v_trim;
    return qbar * f5.wingArea * f5.chord * Cm;
}

double compute_Fz_trim(double alpha, double Lift, double Drag, double Weight)
{
    return -Drag * sin(alpha) - Weight + Lift * cos(alpha);
}

double compute_thrust_req(double Drag, double Lift, double alpha)
{
    return Drag * cos(alpha) + Lift * sin(alpha);
}
