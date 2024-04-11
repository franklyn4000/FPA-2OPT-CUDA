
#include "iolib.cuh"
#include <iostream>
#include <iomanip>  // for std::setprecision and std::fixed
#include "fpa.cuh"
#include "omp.h"
#include "config.h"
#include "drone.h"
#include "math.h"

int main() {

    std::string filename = "../heightMapper/height_map.csv";

    Config config;

    config.iter_max = 160;
    config.population = 45000;
    config.two_opt_freq = 20;
    config.path_length = 7;
    config.resolution = 1 / 2.0f;
    config.p_switch = 0.8;
    config.epsilon_init = 0.25;
    config.epsilon_final = 0.02;
    config.heightMap = load_height_map(filename);

    Drone drone;

    drone.max_asc_angle = 15.0f * M_PI / 180;
    drone.max_desc_angle = -30.0f * M_PI / 180;
    drone.turn_radius = 120.0f;


    computeFPA(config, drone);

    return 0;
}

