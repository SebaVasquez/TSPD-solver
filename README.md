# TSPD-solver
Exact TSP-D solver based on Benders' Decomposition.

## Requirements
This program uses Gurobi to solve MIPs. Therefore, it must be installed.
An installation guide can be found [here](https://www.gurobi.com/documentation/9.0/).

Other libraries must be installed and are included in the 'requirements' file.
If using pip, they can be installed by executing ```pip install -r requirements.txt``` from the command line.

## How to run it?
From the command line, execute ```python main.py <n> <alpha> <L> <instance_id>```, where

* ```n``` is the instance size to generate.
* ```alpha``` is the drone relative speed. Must be greater or equal to 1.
* ```L``` is the drone flight time limit.
* ```instance_id``` is the seed to generate the instance.

Note that instances are randomly generated over a 100x100 grid.