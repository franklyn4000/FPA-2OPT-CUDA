
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

    Config config_p;
    Config config_c;

	config_p.time_limit = (float)configFile["time_limit"];
    config_p.iter_max = (int)configFile["iter_max"];
    config_p.population = (int)configFile["population_parallel"];
    config_p.two_opt_freq = (int)configFile["two_opt_freq"];
    config_p.path_length = (int)configFile["path_length"];
	config_p.n_pi = (int)configFile["n_pi"];
    config_p.resolution = 1 / (float)configFile["resolution"];
	config_p.w1 = (float)configFile["w1"];
	config_p.w2 = (float)configFile["w2"];
    config_p.p_switch = (float)configFile["p_switch"];
    config_p.epsilon_init = (float)configFile["epsilon_init"];
    config_p.epsilon_final = (float)configFile["epsilon_final"];
    config_p.heightMap_cols = (int)initFile["width"];
    config_p.heightMap_rows = (int)initFile["height"];
    config_p.heightMap = load_height_map(heightmap_path);

	config_c.time_limit = (float)configFile["time_limit"];
	config_c.iter_max = (int)configFile["iter_max"];
	config_c.population = (int)configFile["population_cuda"];
	config_c.two_opt_freq = (int)configFile["two_opt_freq"];
	config_c.path_length = (int)configFile["path_length"];
	config_c.n_pi = (int)configFile["n_pi"];
	config_c.resolution = 1 / (float)configFile["resolution"];
	config_c.w1 = (float)configFile["w1"];
	config_c.w2 = (float)configFile["w2"];
	config_c.p_switch = (float)configFile["p_switch"];
	config_c.epsilon_init = (float)configFile["epsilon_init"];
	config_c.epsilon_final = (float)configFile["epsilon_final"];
	config_c.heightMap_cols = (int)initFile["width"];
	config_c.heightMap_rows = (int)initFile["height"];
	config_c.heightMap = load_height_map(heightmap_path);


	float* heightMap_h = load_height_map_cuda(heightmap_path, config_p.heightMap_cols, config_p.heightMap_rows);

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


    printf("OMP\n");
  	computeFPA_parallel(config_p, drone, init);
    printf("--------------------------------------------\n");
    printf("CUDA\n");
    computeFPA_cuda(config_c, heightMap_h, drone, init);

	free(heightMap_h);

	for(auto &innerVec : config_c.heightMap) {
		std::vector<double>().swap(innerVec);
	}

	std::vector<std::vector<double>>().swap(config_c.heightMap);

    return 0;
}

