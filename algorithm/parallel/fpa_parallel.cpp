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


void computeFPA_parallel(
        Config &config, Drone &drone, InitialConditions &init) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    float a_utopia = drone.min_altitude;
    float f_utopia = calculateFUtopia(init);

    double pollination_start_time = 0;
    double pollination_time_taken = 0;

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


/*
    std::vector<float> testPath;

    testPath.push_back(0);
    testPath.push_back(0);
    testPath.push_back(550);

    testPath.push_back(0);
    testPath.push_back(120);
    testPath.push_back(552);

    testPath.push_back(0);
    testPath.push_back(300);
    testPath.push_back(400);

    testPath.push_back(400);
    testPath.push_back(300);
    testPath.push_back(800);

    testPath.push_back(0);
    testPath.push_back(50);
    testPath.push_back(500);

    testPath.push_back(600);
    testPath.push_back(600);
    testPath.push_back(200);

    double test_start_time;
    double test_time_taken = 0;

    std::vector<float> testSmoothed = smoothPath(
            testPath,
            100, 400, nwp);

    for (int i = 0; i < 25000; i++) {
        test_start_time = omp_get_wtime();

        float F =
                computeFitness(testSmoothed,
                               config.heightMap,
                               nwp,
                               drone.max_asc_angle,
                               drone.max_desc_angle,
                               a_utopia, f_utopia, config.resolution);

        test_time_taken += omp_get_wtime() - test_start_time;
    }





    printf("Test Path Smoothing: %f\n", test_time_taken);
*/
}

