# RocketSim
This is the supporting code for a research project I performed in Fall of 2023 for my Computational Engineering Degree at Juniata College. It simulates a Single-Engine Thrust vector controlled rocket along with 4 control systems: PID, MPC, Feed Forward, and LQR. The goal was to compare the performance of each control system for self guidance of the simple rocket model.

## Installation
The project requires a number of dependencies: 
- [Chrono](https://projectchrono.org/) - A physics engine used to simulate the rocket

You need to build project chrono and follow the instructions to understand how to link it to your program. Then this porject should be loadable as a Visual Studio 2022 Solution. You may need to change folders and such to get it to load dependencies correctly.

## Running
The program has 3 run modes: Default, CLI, and CSV. I primarily used and developed for the CSV mode which utilizes the runs.csv file to dictate the parameters of the simulation. The CLI mode is used for one off runs. The default mode is used for running the simulation with a single set of parameters(useful to test builds). To run the csv pass the -f paramter followed by the path to the csv file. Results are automatically stored to the results.csv file for general results and into specific csv files for each run/stage. 

