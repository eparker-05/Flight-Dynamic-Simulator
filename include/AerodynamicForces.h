#ifndef AerodynamicForces_h
#define AerodynamicForces_h
#include "AircraftState.h"

struct AerodynamicForces
{
double lift;
double drag; 
double sideforce;

};

AerodynamicForces compute_forces(const AircraftState& state);
#endif
