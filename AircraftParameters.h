#ifndef Aircraft_Parameters_h
#define Aircraft_Parameters_h

struct AircraftParameters
{
    //Geometry
    double length = 15.7; //m
    double height = 4.0; //m
    double wingArea = 17.28; //m^2 
    double wingspan = 8.13; //m 
    double chord = 2.5; //m

    //Propulsion
    double thrust = 44000.0; //N for twin engines

    //Mass Properties
    double mass = 7000.0; //kg, with partial fuel
    double Ix = 4.0e4; //kg*m^2
    double Iy = 1.2e5; //kg*m^2
    double Iz = 1.6e5; //kg*m^2 

    //Aerodynamics
    double CL0 = 0.05;
    double CD0 = 0.02;
    double CL_alpha = 5.5; //rad
};
#endif