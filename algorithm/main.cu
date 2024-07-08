
#include "iolib.cuh"
#include <iostream>
#include <iomanip>  // for std::setprecision and std::fixed
#include "parallel/fpa_parallel.h"
#include "cuda/fpa_cuda.cuh"
#include "omp.h"
#include "objects/config.h"
#include "objects/drone.h"
#include "objects/initialConditions.h"
#include "math.h"
#include "json.hpp"

int main() {
    nlohmann::json initFile = readJsonFile("../heightMapper/init.json");
    nlohmann::json droneFile = readJsonFile("../heightMapper/drone.json");
    nlohmann::json configFile = readJsonFile("../heightMapper/config.json");

    std::string heightmap_path = "../heightMapper/" + (std::string)initFile["heightmap_file"];

    Config config;

    config.iter_max = (int)configFile["iter_max"];
    config.population = (int)configFile["population"];
    config.two_opt_freq = (int)configFile["two_opt_freq"];
    config.path_length = (int)configFile["path_length"];
    config.resolution = 1 / (float)configFile["resolution"];
    config.p_switch = (float)configFile["p_switch"];
    config.epsilon_init = (float)configFile["epsilon_init"];
    config.epsilon_final = (float)configFile["epsilon_final"];
    config.heightMap_cols = (int)initFile["width"];
    config.heightMap_rows = (int)initFile["height"];
    config.heightMap = load_height_map(heightmap_path);
	float* heightMap_h = load_height_map_cuda(heightmap_path, config.heightMap_cols, config.heightMap_rows);

    Drone drone;

    drone.max_asc_angle = (float)droneFile["max_asc_angle"] * M_PI / 180;
    drone.max_desc_angle = (float)droneFile["max_desc_angle"] * M_PI / 180;
    drone.turn_radius = (float)droneFile["turn_radius"];
    drone.min_altitude = (float)droneFile["min_altitude"];

    InitialConditions init;

    init.x_min = (float)initFile["x_min"];
    init.x_max = (float)initFile["x_max"];
    init.z_min = (float)initFile["z_min"];
    init.y_min = (float)initFile["y_min"];
    init.y_max = (float)initFile["y_max"];
    init.z_max = (float)initFile["z_max"];
    init.x1 = (float)initFile["x1"];
    init.y1 = (float)initFile["y1"];
    init.z1 = (float)initFile["z1"];
    init.xn = (float)initFile["xn"];
    init.yn = (float)initFile["yn"];
    init.zn = (float)initFile["zn"];

    printf("%i\n",config.iter_max);

    printf("OMP\n");
    computeFPA_parallel(config, drone, init);
    printf("--------------------------------------------\n");
    printf("CUDA\n");
    computeFPA_cuda(config, heightMap_h, drone, init);

    return 0;
}

