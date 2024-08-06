//
// Created by franklyn on 3/17/24.
//

#include "fpa_parallel.h"
#include <iostream>
#include <random>
#include <math.h>
#include "omp.h"
#include "../iolib.cuh"
#include "../initialSolutionGenerator.cuh"
#include "pathSmoother_parallel.h"
#include "fitnessComputer_parallel.h"
#include "../utils.cuh"
#include "twoOpt_parallel.h"
#include "pollinator_parallel.h"
#include "../objects/paths.h"


Results computeFPA_parallel(
        Config &config, Drone &drone, InitialConditions &init, bool t_flag) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    float a_utopia = drone.min_altitude;
    float f_utopia = calculateFUtopia(init);

    double pollination_start_time = 0;
    double pollination_time_taken = 0;

    	double data_transfer_start_time = 0;
    double data_transfer_time_taken = 0;
    double twoopt_start_time = 0;
    double twoopt_time_taken = 0;
    double total_start_time = 0;
	double total_time_taken = 0;
    double iteration_start_time = 0;
	double iteration_time_taken = 0;
    int iterations = 0;

    total_start_time = omp_get_wtime();

    float nwp = 0;

    Paths paths(config.population);

    data_transfer_start_time = omp_get_wtime();

    //Initialize a population of n flowers/pollen gametes with random solutions
    paths.rawPaths = generateSolutions(init,
                                       config.path_length, config.population);

    paths.pollinatedPaths = generateSolutions(init,
                                              config.path_length, config.population);


    double smoothing_start_time = omp_get_wtime();

    smoothPaths(paths, drone.turn_radius, config.n_pi);

    double smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

    double fitness_start_time = omp_get_wtime();

    computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                     config.resolution, config.w1, config.w2);

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;
    total_time_taken += omp_get_wtime() - total_start_time;

    data_transfer_time_taken = omp_get_wtime() - data_transfer_start_time;

    printf("Iteration: ");
    while (iterations < config.iter_max && total_time_taken < config.time_limit) {
        iteration_start_time = omp_get_wtime();
        pollination_start_time = omp_get_wtime();

        pollinate_parallel(paths, config.p_switch);

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths(paths, drone.turn_radius, config.n_pi);

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                         config.resolution, config.w1, config.w2);

        fitness_time_taken += omp_get_wtime() - fitness_start_time;


        int quarter = std::ceil(config.iter_max / 4.0);
        int half = std::ceil(config.iter_max / 2.0);
        int eight = std::ceil(config.iter_max / 8.0);

        if (iterations == eight) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius,  config.n_pi, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest1.csv");
        } else if (iterations == quarter) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius,  config.n_pi, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest2.csv");
        } else if (iterations == half) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius,  config.n_pi, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest3.csv");
        }


        if (iterations % config.two_opt_freq == 0) {
            printf("%i ", iterations);
            twoopt_start_time = omp_get_wtime();

            twoOptParallel(paths, drone.turn_radius, config.n_pi, config.heightMap, drone.max_asc_angle,
                           drone.max_desc_angle, a_utopia, f_utopia, config.resolution, config.w1, config.w2);

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;
        }

        computeBestFitness(paths);

        iterations++;
        iteration_time_taken = omp_get_wtime() - iteration_start_time;
		total_time_taken += iteration_time_taken;

    }
    printf("\n");

    std::vector<float> smoothedPath = smoothPath(
            paths.fittestPath,
            drone.turn_radius, config.n_pi, nwp);

    save_to_csv(smoothedPath, "../heightMapper/fittest4.csv");

    double totalTime = pollination_time_taken + smoothing_time_taken + fitness_time_taken + twoopt_time_taken;

    printf("\nPollination, Smoothing, Fitness, 2-opt:\n%.2f, %.2f, %.2f, %.2f\n", pollination_time_taken / totalTime, smoothing_time_taken / totalTime, fitness_time_taken / totalTime, twoopt_time_taken / totalTime);

    printf("PARA Algorithm time: %f %f Reached Fitness: %f after %i iterations\n", total_time_taken, totalTime, paths.bestFitness, iterations);

	Results results;
	results.setup_and_transfer_time = data_transfer_time_taken;
    results.pollination_time = pollination_time_taken;
    results.smoothing_time = smoothing_time_taken;
    results.fitness_time = fitness_time_taken;
    results.twoopt_time = twoopt_time_taken;
    results.total_time = total_time_taken;
	results.best_fitness = paths.bestFitness;

	return results;
}

