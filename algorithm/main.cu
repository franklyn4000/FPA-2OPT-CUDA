
#include "iolib.h"
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
#include <unistd.h>
#include <cstdlib>
#include <vector>

int main(int argc, char** argv) {
	bool t_flag = false;
	bool cuda_flag = false;
	bool omp_flag = false;
	char* output_file_name;


	for (int i = 1; i < argc; i++) {
		std::string arg = argv[i];
		if(arg=="-t"){
			t_flag = true;
		}
		else if(arg=="-cuda"){
			cuda_flag = true;
		}
		else if(arg=="-omp"){
			omp_flag = true;
		}
		else if(arg=="-o" && i+1<argc){ // Make sure we do not go out of bounds
			output_file_name = argv[++i];
		}
		else {
			std::cerr << "Invalid argument: " << arg <<"\n";
			return 1;
		}
	}

	for (int index = optind; index < argc; index++) {
		if(std::string(argv[index])=="-cuda") cuda_flag = true;
		if(std::string(argv[index])=="-omp") omp_flag = true;
		if(std::string(argv[index])=="-t") t_flag = true;
	}

    nlohmann::json initFile = readJsonFile("../config/init.json.copy");
    nlohmann::json droneFile = readJsonFile("../config/drone.json.copy");
    nlohmann::json configFile = readJsonFile("../config/config.json.copy");

    std::string heightmap_path = "../heightMapper/heightmaps/" + (std::string)initFile["heightmap_file"];

    createDirectory("../plotdata");
    createDirectory("../data");
    createDirectory("../data/paths");

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


	char* buffer = new char[100];
	char* omp_filename_timings = new char[100];
	char* omp_filename_fitnesses = new char[100];
	char* omp_filename_fitness_evolution = new char[100];

	char* cuda_filename_timings = new char[100];
	char* cuda_filename_fitnesses = new char[100];
	char* cuda_filename_fitness_evolution = new char[100];

	sprintf(omp_filename_timings, "../data/OMP_timings-%s.dat", output_file_name);
	sprintf(omp_filename_fitnesses, "../data/OMP_fitnesses-%s.dat", output_file_name);
	sprintf(omp_filename_fitness_evolution, "../plotdata/OMP_fitness_evolution-%s.dat", output_file_name);

	sprintf(cuda_filename_timings, "../data/CUDA_timings-%s.dat", output_file_name);
	sprintf(cuda_filename_fitnesses, "../data/CUDA_fitnesses-%s.dat", output_file_name);
	sprintf(cuda_filename_fitness_evolution, "../plotdata/CUDA_fitness_evolution-%s.dat", output_file_name);

	if(omp_flag) {
		createEmptyFile(omp_filename_timings);
		createEmptyFile(omp_filename_fitnesses);
		Results omp_run_result = computeFPA_parallel(config_p, drone, init, t_flag);
		sprintf(buffer, "%f %f %f %f %f %f", omp_run_result.total_time, omp_run_result.setup_and_transfer_time, omp_run_result.pollination_time, omp_run_result.smoothing_time, omp_run_result.fitness_time, omp_run_result.twoopt_time);
		appendLineToFile(omp_filename_timings, buffer);
		sprintf(buffer, "%f", omp_run_result.best_fitness);
		appendLineToFile(omp_filename_fitnesses, buffer);
	}

	if(cuda_flag) {
		createEmptyFile(cuda_filename_timings);
		createEmptyFile(cuda_filename_fitnesses);
		Results cuda_run_result = computeFPA_cuda(config_c, heightMap_h, drone, init, t_flag);
		sprintf(buffer, "%f %f %f %f %f %f", cuda_run_result.total_time, cuda_run_result.setup_and_transfer_time, cuda_run_result.pollination_time, cuda_run_result.smoothing_time, cuda_run_result.fitness_time, cuda_run_result.twoopt_time);
		appendLineToFile(cuda_filename_timings, buffer);
		sprintf(buffer, "%f", cuda_run_result.best_fitness);
		appendLineToFile(cuda_filename_fitnesses, buffer);
		if(t_flag) {
			createAndReplaceEmptyFile(cuda_filename_fitness_evolution);
			for (int i = 0; i < cuda_run_result.fitnesses.size(); i++) {
				sprintf(buffer, "%i %f", i, cuda_run_result.fitnesses[i]);
				appendLineToFile(cuda_filename_fitness_evolution, buffer);
			}
		}
	}

	free(buffer);
	free(omp_filename_timings);
	free(omp_filename_fitnesses);
	free(omp_filename_fitness_evolution);

	free(cuda_filename_timings);
	free(cuda_filename_fitnesses);
	free(cuda_filename_fitness_evolution);

	free(heightMap_h);

	for(auto &innerVec : config_c.heightMap) {
		std::vector<double>().swap(innerVec);
	}

	std::vector<std::vector<double>>().swap(config_c.heightMap);

    return 0;
}

