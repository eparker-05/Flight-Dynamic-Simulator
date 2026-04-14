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
    //double thrust = 44000.0; //N for twin engines
    double v_trim = 180.0;

    //Mass Properties
    double mass = 7000.0; //kg, with partial fuel
    double Ix = 4.0e4; //kg*m^2
    double Iy = 1.2e5; //kg*m^2
    double Iz = 1.6e5; //kg*m^2 

    //Aerodynamics
    double CL0 = 0.05;
    double CD0 = 0.025;
    double CL_alpha = 5.5; //rad
    double CY_beta = -0.7; //rad
    double Cm0 = 0.0;
    double Cm_alpha = -1.0;
    double Cm_q = -40.0;
    double Cm_delta_e = 3.0;
    double CL_delta_e = 0.3;
    double Cl_beta = -0.08;
    double Cl_p = -0.50;
    double Cn_beta = 0.12;
    double Cn_r = -0.15;


   //Universal Constants
   double rho = 1.225; // kg/m^3
};
#endif
