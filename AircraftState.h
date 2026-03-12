#ifndef Aircraft_State_h
#define Aircraft_State_h

struct AircraftState
{
    //Body velocities
    double u;
    double v; 
    double w;

    //Angular rates
    double p;
    double q;
    double r;

    //Orientation
    double phi;
    double theta;
    double psi;

    //Position
    double x;
    double y; 
    double z;
};
#endif