# Flight-Dynamic-Simulator
This project implements a nonlinear 6-degree-of-freedom (6DOF) flight dynamics simulator for a rigid-body aircraft using C++.

The model simulates how an aircraft responds to disturbances by solving the full set of coupled translational and rotational equations of motion, along with aerodynamic force and moment models.

Key Features:
* Full rigid-body dynamics:
  * Translational motion: u,v,w
  * Rotational motion: p,q,r
* Euler-angle kinematics:
  * ϕ,θ,ψ
* Inertial position propagation:
  * x,y,z
* Aerodynamic modeling:
  * Lift, drag, and side force
  * Roll, pitch, and yaw moments
* Wind gust disturbance modeling
* Trim-based initialization

Assumptions:
* Constant gravity
* Flat Earth approximation
* Simplified aerodynamic coefficient models
* Principal inertia (no cross-inertia terms)

About the aircraft: 
The simulation uses approximate parameters for the Northrop F-5 Tiger II, a lightweight supersonic fighter aircraft. The F-5 features a conventional aerodynamic layout with straight wings, a narrow fuselage, and two small turbojet engines, which makes it relatively easy to maintain and operate. Typical flight conditions include medium to high altitude operation, roughly 25,000-40,000ft. Aircraft typically cruises around Mach 0.8-0.9 while max speed is around Mach 1.6. With its light weight and small wing area design, the aircraft is capable of rapid acceleration and high maneuverability. 

Reccommended Directions: 
* To compile - g++ Simulation.cpp Aerodynamics.cpp Dynamics.cpp -o sim
* To run - ./sim
