//Created this file for the use by Dynamics.cpp
#include "AircraftState.h"
#include "AerodynamicForces.h"
#ifndef Aerodynamics_h
#define Aerodynamics_h
//Aerodynamic force declarations
double compute_velocity(const AircraftState& state);
double compute_beta(double, const AircraftState& state);
double compute_alpha(const AircraftState& state);
double compute_CL(double, double);
double compute_CD(double);
double compute_CY(double);
double compute_drag(double, double);
double compute_lift(double, double);
double compute_sideforce(double, double);
double compute_Cm(double alpha, double q, double V, double delta_e);
double compute_pitch_moment(double Cm, double V);

AerodynamicForces compute_forces(AircraftState& state);
#endif 
