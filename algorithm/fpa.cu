//
// Created by franklyn on 3/17/24.
//

#include "fpa.cuh"
#include <vector>
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

std::vector <std::vector<float>> computeFPA(
        const std::vector <std::vector<double>> &heightMap,
        int iter_max,
        int population,
        float p_switch,
        float epsilon_init,
        float epsilon_final,
        int two_opt_freq) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    size_t x_mid = heightMap.size() / 2;
    size_t y_mid = (heightMap.empty()) ? 0 : heightMap[0].size() / 2;

    float x_min = 0.0f;
    float x_max = heightMap.size() - 1;
    float z_min = -300;
    float y_min = 0.0f;
    float y_max = heightMap[0].size() - 1;
    float z_max = 300;
    float x1 = 5.0f;
    float y1 = (float) y_mid;
    float z1 = heightMap[y1][x1] + 10;
    float xn = heightMap.size() - 5.0f;
    float yn = (float) y_mid;
    float zn = heightMap[yn][xn] + 10;

    int path_length = 8;

    float turn_radius = 36.0f;

    float a_utopia = 1.0f;
    float f_utopia = calculateFUtopia(x1, y1, z1, xn, yn, zn);

    float max_asc_angle_deg = 10.0f;
    float max_desc_angle_deg = -30.0f;
    float max_asc_angle = max_asc_angle_deg * M_PI / 180;
    float max_desc_angle = max_desc_angle_deg * M_PI / 180;

    double init_start_time = omp_get_wtime();

    Paths paths(population);

    //Initialize a population of n flowers/pollen gametes with random solutions
    paths.rawPaths = generateSolutions(
            x1, y1, z1,
            xn, yn, zn,
            x_min, x_max, y_min,
            y_max, z_min, z_max,
            path_length, population);

    double init_time_taken = omp_get_wtime() - init_start_time;

    //std::vector <std::vector<float>> testSolutions = generateTestSolutions();

    double smoothing_start_time = omp_get_wtime();

    smoothPaths(paths, turn_radius, turn_radius * 2);

    double smoothing_time_taken = omp_get_wtime() - smoothing_start_time;
    //save_to_csv(N_wps, "../heightMapper/N_wps.csv");

    double fitness_start_time = omp_get_wtime();

    computeFitnesses(paths, heightMap, max_asc_angle, max_desc_angle, a_utopia, f_utopia);

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;

    //std::vector<float> smoothPath1 = smoothPathSingle(fittestPath, turn_radius, turn_radius * 2, N_wps);

    save_to_csv(paths.rawPaths[1], "../heightMapper/fittest.csv");

    int logIndex = 0;

    for(int i = 0; i < iter_max; i++) {

        printf("ITERATION %i - ", i);

        double pollination_start_time = omp_get_wtime();

        pollinate_parallel(paths, p_switch);

        double pollination_time_taken = omp_get_wtime() - pollination_start_time;

        printf("pollinated - %f - ", pollination_time_taken);

        smoothing_start_time = omp_get_wtime();

        smoothPaths(paths, turn_radius, turn_radius * 2);

        smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

        printf("smoothed - %f - ", smoothing_time_taken);

        fitness_start_time = omp_get_wtime();

        computeFitnesses(paths, heightMap, max_asc_angle, max_desc_angle, a_utopia, f_utopia);

        fitness_time_taken = omp_get_wtime() - fitness_start_time;

        printf("fitnessed - %f - best fitness: %f\n", fitness_time_taken, paths.bestFitness);


        int quarter = std::ceil(iter_max / 4.0);
        int half = std::ceil(iter_max / 2.0);
        int eight = std::ceil(iter_max / 8.0);

        if (i == eight) {
            save_to_csv(paths.fittestPath, "../heightMapper/fittest1.csv");
        } else if (i == quarter) {
            save_to_csv(paths.fittestPath, "../heightMapper/fittest2.csv");
        } else if (i == half) {
            save_to_csv(paths.fittestPath, "../heightMapper/fittest3.csv");
        }

/*
        if(i % two_opt_freq == 0) {
            twoOptParallel(initialSolutions, fittestPath, p_switch);

            smoothedPaths = smoothPaths(initialSolutions, turn_radius, turn_radius * 2, N_wps);

            bestFitness = computeFitnesses(smoothedPaths, &fittestPath, bestFitness, initialSolutions,  heightMap, N_wps, max_asc_angle, max_desc_angle, a_utopia, f_utopia);

        }
        */

    }


   // std::vector<float> smoothPath = smoothPathSingle(fittestPath, turn_radius, turn_radius * 2, N_wps);


   // save_to_csv(smoothPath, "../heightMapper/fittest2.csv");

   save_to_csv(paths.fittestPath, "../heightMapper/fittest4.csv");


    //Find the best solution g∗ in the initial population

    //while (t <MaxGeneration)
    //   for i = 1 : n (all n flowers in the population)
    //if rand < p,
    //           Draw a (d-dimensional) step vector L which obeys a L´evy distribution
    //Global pollination via x

    //  else
    //  Draw ǫ from a uniform distribution in [0,1]
    //  Randomly choose j and k among all the solutions
    // Do local pollination via x

    //   end if
    //      Evaluate new solutions
    //  If new solutions are better, update them in the population
    //  end for
    //  Find the current best solution g∗
    //  end while


    printf("\nSolution Generation time: %f\n", init_time_taken);

    printf("Path smoothing time: %f\n", smoothing_time_taken);

    printf("Fitness computation time: %f\n", fitness_time_taken);

    return paths.smoothedPaths;
}

