
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

int main() {

    std::string filename = "../heightMapper/height_map.csv";

    Config config;

    config.iter_max = 10;
    config.population = 50;
    config.two_opt_freq = 25;
    config.path_length = 7;
    config.resolution = 1 / 2.0f;
    config.p_switch = 0.8;
    config.epsilon_init = 0.25;
    config.epsilon_final = 0.02;
    config.heightMap = load_height_map(filename);

    Drone drone;

    drone.max_asc_angle = 15.0f * M_PI / 180;
    drone.max_desc_angle = -30.0f * M_PI / 180;
    drone.turn_radius = 100.0f;
    drone.min_altitude = 20.0f;

    InitialConditions init;

    size_t x_mid = config.heightMap.size() / 2;
    size_t y_mid = (config.heightMap.empty()) ? 0 : config.heightMap[0].size() / 2;

    init.x_min = 0.0f;
    init.x_max = config.heightMap.size() - 1;
    init.z_min = -500;
    init.y_min = 0.0f;
    init.y_max = config.heightMap[0].size() - 1;
    init.z_max = 500;
    init.x1 = 5.0f;
    init.y1 = (float) y_mid;
    init.z1 = config.heightMap[init.y1][init.x1] + 60;
    init.xn = config.heightMap.size() - 5.0f;
    init.yn = (float) y_mid;
    init.zn = config.heightMap[init.yn][init.xn] + 60;

    printf("OMP\n");
    computeFPA_parallel(config, drone, init);
    printf("--------------------------------------------\n");
    printf("CUDA\n");
    computeFPA_cuda(config, drone, init);

    return 0;
}

