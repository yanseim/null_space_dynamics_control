# pd_plus_gravity_control

This repository contains the code for simulation of pd+gravity control with KUKA iiwa14 in **MATLAB** and **CoppeliaSim**(Vrep).



#### Prerequisites

* MATLAB (later than r2020a)
* Coppeliasim edu v4.4.0 (The latest version as of the time of writing this example. October 2020. ）The `remApi.m`, `remoteApiProto.m` and `remoteApi.dll` in folder `libs` are copied from `CoppeliaRobotics\CoppeliaSimEdu\programming\legacyRemoteApi\remoteApiBindings\matlab\matlab` and `CoppeliaRobotics\CoppeliaSimEdu\programming\legacyRemoteApi\remoteApiBindings\lib\lib\Windows`)



#### Usage

* Run `pd_plus_gravity.m` in MATLAB first, then `pd_plus_gravity.ttt` in CoppeliaSim. There may be a warning appeared in CoppeliaSim, just ignore it.
* If you want to stop the simulation, just stop the simulation in CoppeliaSim, after that the MATLAB part will automatically stop, followed by a figure.

* In hw2, the coriolis matrix can be obtained through function `coriolis_kuka/autogen/C_mtrx_fcn.m`. To understand how this code is generated, check `coriolis_kuka/main.m`.



