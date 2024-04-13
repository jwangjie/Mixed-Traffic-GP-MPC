# Mixed-Vehicle Platooning Control with a Gaussian Process Learning-Based Model

This repository contains the source code for research on enhancing safety in mixed-traffic platooning through a hybrid model. This approach integrates Gaussian Process (GP) learning with traditional first-principles modeling to predict behaviors of human-driven vehicles in mixed traffic scenarios.
### Features
- **Hybrid Modeling:** Combines GP learning with first-principles models for accurate predictions.
- **Real-Time Performance:** Optimized for real-time applications with minimal computational load.
- **Safety Enhancements:** Demonstrates improved vehicle distances and platoon speeds in simulations.

This work is detailed in the paper:
* [Enhancing Safety in Mixed Traffic: Learning-Based Modeling and Efficient Control of Autonomous and Human-Driven Vehicles](https://arxiv.org/abs/2404.06732)

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

## Citing Our Work
If our work aids your research or you use the GP-MPC framework, please consider citing:

```
@article{wang2024enhancing,
  title={Enhancing safety in mixed traffic: Learning-based modeling and efficient control of autonomous and human-driven vehicles},
  author={Wang, Jie and Pant, Yash Vardhan and Zhao, Lei and Antkiewicz, Micha{\l} and Czarnecki, Krzysztof},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  year={2024},
  publisher={IEEE}
}
```

## Please also check our preliminary work at [GP-MPC-of-Platooning](https://github.com/CL2-UWaterloo/GP-MPC-of-Platooning).

```
@article{wang2024improving,
  title={Improving safety in mixed traffic: A learning-based model predictive control for autonomous and human-driven vehicle platooning},
  author={Wang, Jie and Jiang, Zhihao and Pant, Yash Vardhan},
  journal={Knowledge-Based Systems},
  volume={293},
  pages={111673},
  year={2024},
  publisher={Elsevier}
}
```
```
@article{wang2024learning,
  title={Learning-based modeling of human-autonomous vehicle interaction for improved safety in mixed-vehicle platooning control},
  author={Wang, Jie and Pant, Yash Vardhan and Jiang, Zhihao},
  journal={Transportation Research Part C: Emerging Technologies},
  volume={162},
  pages={104600},
  year={2024},
  publisher={Elsevier}
}
```
