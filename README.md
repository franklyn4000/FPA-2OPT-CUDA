# FPA-2OPT-CUDA
### Efficient trajectory planning algorithm using the Flower Pollination Algorithm for Global Optimization [^1]. augmented with some 2-OPT iterations.



## Overview

Vincent Roberge & Mohammed Tarbouchi proposed this algorithm in their paper 
**Hybrid deterministic non-deterministic data-parallel algorithm for real-time 
unmanned aerial vehicle trajectory planning in CUDA** [^2]. This projects tries to re-recreate 
their results with a special focus on finding ways to optimize the CUDA implementation and
exploring possibilities of deploying it on current mobile hardware. We also include scripts
for parsing .xyz heightmaps and visualizing the generated trajectories as well as benchmarking
the algorithm itself with varying configuration parameters.

## Prerequisites

- Computer with a CUDA capable GPU
- CUDA libraries
- python3

Compilation has only been tested on linux-based systems.

## Usage

Parse the provided heightmaps (or alter to use custom heightmaps.)
The algorithm will fail without having done this once.

`python3 heightmapper/heightMapParser.py`

Edit config files (and their copies!) as needed. Parameters are explained in the next section.

`config/config.json`
`config/init.json`
`config/drone.json`

Run the algorithm:

`algorithm [-cuda] [-omp] [-t] [-o output_file_name]`

- [-omp]: run the GPU version of the algorithm
- [-cuda]: run the GPU version of the algorithm
- [-t]: optional flag that makes the CUDA algorithm write its results to files. Slightly, aster without.
- [-o]: an optional string to be appended to all output files to facilitate separating different runs.

Note: If both -omp and -cuda are set, the algorithms will run sequentially.

Use the visualizer to show a browser-based 3D representation of the generated paths

`python3 heightmapper/heightMapVisualizer.py`

## Config Parameters
### config.json

| Parameter             | Description                                                                                                |
| --------------------- |------------------------------------------------------------------------------------------------------------|
| time_limit            | Make alg. stop iterating after n seconds.                                                                  |
| iter_max              | Make alg. stop iterating after n iterations.                                                               |
| population_parallel   | population for the OMP version.                                                                            |
| population_cuda       | population for the CUDA version. Note, this will spawn n threads for FPA and n * 32 threads for the 2-OPT  |
| two_opt_freq          | Run the 2-OPT every nth iteration.                                                                         |
| path_length           | Amount of waypoints in a path including start and goal.                                                    |
| n_pi                  | Amount of waypoints needed to describe an 180° arch.                                                       |
| resolution            | The algorithm traces each path and computes on every nth grid cell certain fitness criteria like altitude. |
| w1                    | Weight for fitness criterium 1: Path shortness                                                             |
| w2                    | Weight for fitness criterium 2: Low average altitude                                                       |
| p_switch              | FPA constant that determines how many paths are globally vs. locally pollinated                            |

### init.json

| Parameter       | Description                                               |
| --------------- |-----------------------------------------------------------|
| heightmap_file  | Filename for parsed heightmap in /heightMapper/heightmaps |
| width           | heightmap with                                            |
| height          | heightmap heigth                                          |
| x_min           | Minimum x for generated waypoints                         |
| x_max           | Maximum x for generated waypoints                         |
| z_min           | Minimum y for generated waypoints                         |
| y_min           | Maximum y for generated waypoints                         |
| y_max           | Minimum z for generated waypoints                         |
| z_max           | Maximum z for generated waypoints                         |
| x1              | x value of first waypoint                                 |
| y1              | y value of first waypoint                                 |
| z1              | z value of first waypoint                                 |
| xn              | x value of last waypoint                                  |
| yn              | y value of last waypoint                                  |
| zn              | z value of last waypoint                                  |

### drone.json

| Parameter     | Description                         |
| ------------- |-------------------------------------|
| max_asc_angle | Maximum ascent angle for the drone  |
| max_desc_angle| Maximum descent angle for the drone |
| turn_radius   | minimum turn radius for the drone   |
| min_altitude  | minimum altitude for viable paths   |

## References

[^1] Yang, Xin-She. "Flower pollination algorithm for global optimization." International conference on unconventional computing and natural computation. Berlin, Heidelberg: Springer Berlin Heidelberg, 2012.

[^2] Roberge, Vincent, and Mohammed Tarbouchi. "Hybrid deterministic non-deterministic data-parallel algorithm for real-time unmanned aerial vehicle trajectory planning in CUDA." e-Prime-Advances in Electrical Engineering, Electronics and Energy 2 (2022): 100085.