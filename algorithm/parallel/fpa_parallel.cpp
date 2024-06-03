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

    float nwp = 0;

    Paths paths(config.population);

    //Initialize a population of n flowers/pollen gametes with random solutions
    paths.rawPaths = generateSolutions(init,
                                       config.path_length, config.population);

    paths.pollinatedPaths = generateSolutions(init,
                                              config.path_length, config.population);


    double smoothing_start_time = omp_get_wtime();

    smoothPaths(paths, drone.turn_radius, drone.turn_radius * 2);

    double smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

    double fitness_start_time = omp_get_wtime();

    computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                     config.resolution);

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;


    printf("Iteration:\n");
    for (int i = 0; i < config.iter_max; i++) {

        pollination_start_time = omp_get_wtime();

        pollinate_parallel(paths, config.p_switch);

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths(paths, drone.turn_radius, drone.turn_radius * 2);

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                         config.resolution);

        fitness_time_taken += omp_get_wtime() - fitness_start_time;


        int quarter = std::ceil(config.iter_max / 4.0);
        int half = std::ceil(config.iter_max / 2.0);
        int eight = std::ceil(config.iter_max / 8.0);

        if (i == eight) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius, drone.turn_radius * 2, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest1.csv");
        } else if (i == quarter) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius, drone.turn_radius * 2, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest2.csv");
        } else if (i == half) {
            std::vector<float> smoothedPath = smoothPath(
                    paths.fittestPath,
                    drone.turn_radius, drone.turn_radius * 2, nwp);
            save_to_csv(smoothedPath, "../heightMapper/fittest3.csv");
        }


        if (i % config.two_opt_freq == 0) {
            printf("%i \n", i);
            twoopt_start_time = omp_get_wtime();

            twoOptParallel(paths, drone.turn_radius, drone.turn_radius * 2, config.heightMap, drone.max_asc_angle,
                           drone.max_desc_angle, a_utopia, f_utopia, config.resolution);

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;
        }

        computeBestFitness(paths);

    }
    printf("\n");

    std::vector<float> smoothedPath = smoothPath(
            paths.fittestPath,
            drone.turn_radius, drone.turn_radius * 2, nwp);

    save_to_csv(smoothedPath, "../heightMapper/fittest4.csv");

    double totalTime = pollination_time_taken + smoothing_time_taken + fitness_time_taken + twoopt_time_taken;

    printf("\nPollination, Smoothing, Fitness, 2-opt:\n%.2f, %.2f, %.2f, %.2f\n", pollination_time_taken / totalTime, smoothing_time_taken / totalTime, fitness_time_taken / totalTime, twoopt_time_taken / totalTime);

    printf("PARA Algorithm time: %f Reached Fitness: %f\n", totalTime, paths.bestFitness);


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

