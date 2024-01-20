## Mixed-Vehicle Platooning Control with a Gaussian Process Learning-Based Model

This repository hosts the code accompanying our research on enhancing safety in mixed-traffic platooning, which involves both autonomous and human-driven vehicles. Our work is detailed in the paper:

* [Enhancing Safety in Mixed Traffic: Learning-Based Modeling and Efficient Control of Autonomous and Human-Driven Vehicles](https://arxiv.org/abs/2211.04665) 

### Please check our preliminary work at [GP-MPC-of-Platooning](https://github.com/CL2-UWaterloo/GP-MPC-of-Platooning).

### ARX_simulation
This directory contains code for simulating a vehicle platoon using Model Predictive Control (MPC) with an AutoRegressive with eXogenous inputs (ARX) model combined with a Gaussian Process (GP). It includes:

* `MPC_platoon_simulation.m`: Main simulation script.
* `MPC_platoon.m`: Implementation of the MPC.
* `GP_sysmodel.m`: Vehicle dynamics model for simulations.
* `gpr_medium.mat`: Standard GP model trained on field data.
* `gpCallback.m`: Callback function for CasADi to integrate the GP model (ARX+GP)
* `wltp_velocity_profile.m`: Script for the Worldwide Harmonized Light Vehicles Test Procedure (WLTP) velocity profile.


### GP_ARX_simulation
This folder contains the code for simulating a platoon of vehicles using MPC with a sparseGP+ARX model, while the simulated vehicle is a GP+ARX model. Specifically, the following files are included:

* `GP_MPC_platoon_simulation.m`: Main script for sparseGP-MPC-based simulations.
* `GP_MPC_platoon.m`: Gaussian Process Model Predictive Control (GP-MPC) implementation.
* `GP_sysmodel.m`: Vehicle dynamics model for simulations.
* `gpr_medium.mat`: Standard GP model trained on field data.
* `gpr_sparse.mat`: Sparse GP model, trained using `GP_RE_trainForSpares.m` in the `GP_training` folder.
* `gpCallback.m`: Callback for CasADi, loading the sparseGP+ARX model for control purposes.
* `gpCallback_sys.m`: Callback for CasADi, integrating the GP+ARX model for simulating autonomous vehicles.
* `wltp_velocity_profile.m`: WLTP velocity profile script.

### GP_training
Contains scripts for training Gaussian Process (GP) models:

* `GP_training`: Script for training standard GP models.
* `GP_RE_trainForSpares`: Script for developing sparse GP models.
* `plot_to_pdf`: Utility to export plot visualizations as PDFs.

