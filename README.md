# DSV-Sim - Aircraft Dynamics and Simulation Tool

## Introduction
DSV-Sim is a MATLAB tool that allows to study and simulate aircraft dynamics based on a user-defined dynamic model. By inputting the aircraft's specific model, it is possible to analyze its longitudinal dynamic stability and simulate a flight course by setting custom waypoints. 

This project was originally developed as an assignment for the Master's Degree course in *Aircraft Dynamics and Simulation*.

Requires MATLAB 2019b or newer, along with the following toolboxes:
- Mapping Toolbox
- Optimization Toolbox
- Control System Toolbox

---

## Basic Usage
To set up the aircraft dynamic model, a linear aerodynamic model, its inertial properties, and a propulsive model are needed. The following scripts are used to generate the necessary `.mat` files:

* **[`BuildInertia.m`](BuildInertia.m):** generates the [inertia properties file](Data/inertia_tensor.mat);
* **[`BuildAeroModel.m`](BuildAeroModel.m):** generates the [aerodynamic coefficients mat file](Data/aero_database_long.mat) using data obtained from DATCOM;
* **[`BuildThrustModel.m`](BuildThrustModel.m):** generates the [thrust and power coefficients mat file](Data/propeller_data.mat). *Note: This tool assumes the aircraft is propeller-driven.*

Once generated, these files are referenced inside a configuration [.aero file](Data/Q100_comp.aero), which is then used to initialize an object of the `ACClass`.

---

## Longitudinal Dynamics 
The [`ModalAnalysisMain.m`](ModalAnalysisMain.m) script allows to investigate the longitudinal dynamic response of the aicraft. It computes an equilibrium point, evaluates the NACA stability derivatives around that point, and compares the response of the full 6-Degree-of-Freedom dynamic model against the linearized longitudinal dynamic model.

---

## Waypoint Simulation
The waypoint simulation scripts are a MATLAB implementation of the framework described in [1]. The aircraft is modeled with a reduced set of state variables using the Nonlinear Aircraft Performance Equations (NAPE). 
A set of waypoints can be defined in terms of longitude, latitude, speed (IAS) and altitude.


* **Controller Gains:** The [`Gain_Choice.m`](Gain_Choice.m) script allows to select a set of gains for the longitudinal controllers. These gains are applied across different control states depending on the tracking error relative to the current waypoint.
* **Running the Simulation:** Execute the simulation using the [`complete_dynamics_setup.m`](complete_dynamics_setup.m) script. 
* **Wind Conditions:** A wind map can be optionally defined using [`WindData.m`](WindData.m).

---

## References
[1] Mark Peters and Michael A. Konyak. “The engineering analysis and design of the aircraft dynamics model for the FAA target generation facility”. *Seagull Technology, Inc.*, Los Gatos, CA (2012).
