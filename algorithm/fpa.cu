//
// Created by franklyn on 3/17/24.
//

#include "fpa.cuh"
#include <iostream>
#include <random>
#include <math.h>
#include "omp.h"
#include "iolib.cuh"
#include "initialSolutionGenerator.cuh"
#include "pathSmoother_parallel.h"
#include "fitnessComputer.h"
#include "utils.cuh"
#include "pollinator.cuh"
#include "twoOpt_parallel.h"
#include "pollinator_parallel.h"
#include "fitnessComputer_seq.cuh"
#include "paths.h"


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
    float z1 = config.heightMap[y1][x1] + 10;
    float xn = config.heightMap.size() - 5.0f;
    float yn = (float) y_mid;
    float zn = config.heightMap[yn][xn] + 10;

    float a_utopia = 10.0f;
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

    computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia);

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;

    for (int i = 0; i < config.iter_max; i++) {

        pollination_start_time = omp_get_wtime();

        pollinate_parallel(paths, config.p_switch);

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths(paths, drone.turn_radius, drone.turn_radius * 2);

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia);

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
            twoopt_start_time = omp_get_wtime();

            twoOptParallel(paths, drone.turn_radius, drone.turn_radius * 2, config.heightMap, drone.max_asc_angle,
                           drone.max_desc_angle, a_utopia, f_utopia);

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;
            // smoothedPaths = smoothPaths(initialSolutions, turn_radius, turn_radius * 2, N_wps);

            //  bestFitness = computeFitnesses(smoothedPaths, &fittestPath, bestFitness, initialSolutions,  heightMap, N_wps, max_asc_angle, max_desc_angle, a_utopia, f_utopia);
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

}

