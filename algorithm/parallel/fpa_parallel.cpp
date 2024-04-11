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
#include "../pollinator.cuh"
#include "twoOpt_parallel.h"
#include "pollinator_parallel.h"
#include "../fitnessComputer_seq.cuh"
#include "../objects/paths.h"


void computeFPA(
        Config &config, Drone &drone) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    size_t x_mid = config.heightMap.size() / 2;
    size_t y_mid = (config.heightMap.empty()) ? 0 : config.heightMap[0].size() / 2;

    float x_min = 0.0f;
    float x_max = config.heightMap.size() - 1;
    float z_min = -300;
    float y_min = 0.0f;
    float y_max = config.heightMap[0].size() - 1;
    float z_max = 300;
    float x1 = 5.0f;
    float y1 = (float) y_mid;
    float z1 = config.heightMap[y1][x1] + 60;
    float xn = config.heightMap.size() - 5.0f;
    float yn = (float) y_mid;
    float zn = config.heightMap[yn][xn] + 60;

    float a_utopia = drone.min_altitude;
    float f_utopia = calculateFUtopia(x1, y1, z1, xn, yn, zn);


    double pollination_start_time = 0;
    double pollination_time_taken = 0;

    double twoopt_start_time = 0;
    double twoopt_time_taken = 0;

    float nwp = 0;

    Paths paths(config.population);

    //Initialize a population of n flowers/pollen gametes with random solutions
    paths.rawPaths = generateSolutions(
            x1, y1, z1,
            xn, yn, zn,
            x_min, x_max, y_min,
            y_max, z_min, z_max,
            config.path_length, config.population);


    double smoothing_start_time = omp_get_wtime();

    smoothPaths(paths, drone.turn_radius, drone.turn_radius * 2);

    double smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

    double fitness_start_time = omp_get_wtime();

    computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia, config.resolution);

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;

    for (int i = 0; i < config.iter_max; i++) {

        pollination_start_time = omp_get_wtime();

        pollinate_parallel(paths, config.p_switch);

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths(paths, drone.turn_radius, drone.turn_radius * 2);

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia, config.resolution);

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
            printf("Iteration: %i\n", i);
            twoopt_start_time = omp_get_wtime();

            twoOptParallel(paths, drone.turn_radius, drone.turn_radius * 2, config.heightMap, drone.max_asc_angle,
                           drone.max_desc_angle, a_utopia, f_utopia, config.resolution);

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;

            computeBestFitness(paths);
        }


    }

    std::vector<float> smoothedPath = smoothPath(
            paths.fittestPath,
            drone.turn_radius, drone.turn_radius * 2, nwp);

    save_to_csv(smoothedPath, "../heightMapper/fittest4.csv");

    double totalTime = pollination_time_taken + smoothing_time_taken + fitness_time_taken + twoopt_time_taken;

    printf("\nSolution Generation: %.2f\n", pollination_time_taken / totalTime);

    printf("Path smoothing: %.2f\n", smoothing_time_taken / totalTime);

    printf("Fitness computation: %.2f\n", fitness_time_taken / totalTime);

    printf("2-opt computation: %.2f\n", twoopt_time_taken / totalTime);

    printf("-----------------------------------\n");

    printf("Algorithm time: %f Reached Fitness: %f\n", totalTime, paths.bestFitness);


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

