# Introduction
xxx is a MATLAB tool that allows to simulate the dynamics of an aircraft, giving its dynamic model as an input. By defining the aircraft dynamic model, it is possible to study its longitudinal dynamic stability and to simulate a flight course setting waypoints. 
This code has been developed as an assignment for the Master's Degree course in Aircraft Dynamics and Simulation. 
## Basic Usage
The aicraft dynamic model requires a linear aerodynamic model, its inertial properties and a propulsive model. The folllowing scripts are used to define the .mat files that will define the models:
- [BuildInertia](BuildInertia.m) to obtain the [inertia properties file](Data/inertia_tensor.mat)
- [BuildAeroModel](BuildAeroModel.m) to obtain the [mat file with aerodynamic coefficients](Data/aero_database_long.mat). These values are obtained using DATCOM;
- [BuildThrustModel](BuildThrustModel.m) to obtain the [mat file with the thrust and power coefficients](Data/propeller_data.mat). This implies that the aicraft must be propeller driven.
These files are defined in a [aero file](Data/Q100_comp.aero) that is used to define an object of the ACClass. 
## Longitudinal Dynamics 
The script [ModalAnalysisMain](ModalAnalysisMain.m) defines the response of the aicraft in terms of longitudinal dynamics. It finds an equilibrium point and evaluates the NACA stability derivatives around that point. It also compares the response of the full 6 Degree-of-Freedom dynamic model with the linearised longitudinal dynamic model.
## Waypoint Simulation
These scripts are a MATLAB implementation of [1]. The aicraft is modelled with a reduced set of state variables using the Nonlinear Aicraft Performance Equations (NAPE). A set of waypoints can be defined in terms of longitude and latitude and speed and altitude. [BuildInertia](BuildInertia.m) allows to select a set of gains for the longitudinal controllers that will be used in all the conditions for the different controllers used based of the error relative to the current waypoint. The simulation is executed with the script [complete_dynamics_setup](complete_dynamics_setup.m), while a wind map can also be defined using [WindData](WindData.m).
## Reference
[1] Mark Peters e Michael A Konyak. “The engineering analysis and design of the aircraft dynamics model for the FAA target generation facility”. In: _Seagull Technology, Inc._, Los Gatos, CA (2012).
