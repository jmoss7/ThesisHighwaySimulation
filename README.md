# Thesis code Repository

- This repository contains the code for my simulations in my Master Thesis: Simulating Ethical Dilemmas in Autonomous Vehicles

## How to run scenarios:
- Requires installation of CARLA WindowsNoEditor version 0.9.15, and a matching installation of ScenarioRunner (0.9.15 Windows)
- In one terminal window, start the simulator by navigating to the CARLA directory `cd path_to_carla/CARLA_0.9.15`, then running the Carla executable `./CarlaUE4.exe`
- In a separate terminal window, change the map to `Town04`. First, navigate to the appropriate directory: `cd path_to_carla/CARLA_0.9.15/PythonAPI/util`, then running `python config.py -m Town04`
- Finally, in a separate terminal window, run the desired scenario, replacing `[level_suffix]` with the desired version, "L1", "L2", or "L3", and replacing `X` with the number of trials. Navigate to the `scenario_runner` dir: `cd path_to_scenario_runner/scenario_runner`, then run the command `python scenario_runner.py --scenario ThesisHighway_[level_suffix] --sync --repetitions X`

## The code behind each scenario level:
- The scenario and behavior setup for each scenario level are found in the directory `scenario_runner/srunner/scenarios`, where each level of behavior has an associated `thesis_highway_scenario_LX.py` file.

### Special Thanks
Special thanks to the HPAI team for their support in designing the scenario and providing feedback on the final paper.

# Thesis Document
The full text of the thesis will be added here upon gaining permission from the UPC. Thanks!
