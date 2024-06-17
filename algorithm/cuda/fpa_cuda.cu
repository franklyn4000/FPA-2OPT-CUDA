//
// Created by franklyn on 4/11/24.
//

#include "fpa_cuda.cuh"
#include <iostream>
#include <random>
#include <math.h>

#include "fitnessComputer_cuda.cuh"
#include "init_generator_cuda.cuh"
#include "omp.h"
#include "../iolib.cuh"

#include "../utils.cuh"

#include "../parallel/twoOpt_parallel.h"
#include "../parallel/fitnessComputer_parallel.h"
#include "../parallel/twoOpt_parallel.h"
#include "../parallel/pollinator_parallel.h"
#include "../objects/paths.h"
#include "pathSmoother_cuda.cuh"

#define CHECK_CUDA(call)                                            \
{                                                                   \
const cudaError_t error = call;                                     \
if (error != cudaSuccess)                                           \
{                                                                   \
printf("ERROR:: File: %s, Line: %d, ", __FILE__, __LINE__);         \
printf("code: %d, reason: %s\n", error, cudaGetErrorString(error)); \
exit(EXIT_FAILURE);                                                 \
}                                                                   \
}

void computeFPA_cuda(
        Config &config, float* heightMap_h, Drone &drone, InitialConditions &init) {
    float a_utopia = drone.min_altitude;
    float f_utopia = calculateFUtopia(init);

    double pollination_start_time = 0;
    double pollination_time_taken = 0;

    double twoopt_start_time = 0;
    double twoopt_time_taken = 0;

    float nwp = 0;

    int max_waypoints_smoothed = config.path_length * drone.turn_radius * 2 * 2; // upper bound

    Paths_cuda paths;
   // paths.bestFitness = -1.0f;

    dim3 dimBlock(256);
    dim3 dimGrid((config.population + dimBlock.x - 1) / dimBlock.x);

    //Initialize a population of n flowers/pollen gametes with random solutions
    // paths.rawPaths = generateSolutions(init,
    //                                    config.path_length, config.population);

    //   paths.pollinatedPaths = generateSolutions(init,
    //                                            config.path_length, config.population);


    // float* hostPtr = new float[config.population * max_waypoints];

    float *hostPtr = generateSolutions_cuda(init, config.path_length, config.population);

    printf("%i \n", config.population * config.path_length * 3 * sizeof(float));

    float *hostPtr2 = new float[config.population * max_waypoints_smoothed * 3];


    paths.smoothedPaths.n_waypoints = config.path_length;
    paths.smoothedPaths.n_paths = config.population;

    paths.rawPaths.n_waypoints = config.path_length;
    paths.rawPaths.n_paths = config.population;

    size_t raw_paths_size = paths.rawPaths.n_waypoints * 3 * paths.rawPaths.n_paths * sizeof(float);
    size_t smoothed_paths_size = config.population * max_waypoints_smoothed * 3 * sizeof(float);

    printf("size %lu \n", (unsigned long) smoothed_paths_size);
/*
    for (int i = 0; i < paths.rawPaths.n_paths; i++)
        for (int j = 0; j < paths.rawPaths.n_waypoints; j++) {
            hostPtr[i * paths.rawPaths.n_waypoints + j] = i + 1.0;
            // printf("row %i column %i value %f \n", i, j, hostPtr[Ncols * j + i] );
        }*/
    /*
        cudaMallocPitch(&paths.rawPaths, &pitch, Ncols * sizeof(float), Nrows);
        cudaMemcpy2D(&paths.rawPaths, pitch, hostPtr, Ncols*sizeof(float), Ncols*sizeof(float), Nrows, cudaMemcpyHostToDevice);
    */

    size_t pitch = 0;

    CHECK_CUDA(cudaMalloc(&config.heightMap_cuda, config.heightMap_rows * config.heightMap_cols * sizeof(float)));
    cudaMemcpy(config.heightMap_cuda, heightMap_h, config.heightMap_rows * config.heightMap_cols * sizeof(float),
               cudaMemcpyHostToDevice);

    CHECK_CUDA(cudaMalloc(&paths.smoothedPaths.elements, smoothed_paths_size));
    cudaMemcpy(paths.smoothedPaths.elements, hostPtr2, smoothed_paths_size,
               cudaMemcpyHostToDevice);

    CHECK_CUDA(cudaMalloc(&paths.smoothedPaths.used_waypoints, config.population * sizeof(int)));


    CHECK_CUDA(cudaMalloc(&paths.rawPaths.elements, raw_paths_size));
    cudaMemcpy(paths.rawPaths.elements, hostPtr, raw_paths_size,
               cudaMemcpyHostToDevice);




    //  CHECK_CUDA(cudaMalloc((void **) &paths.rawPaths, config.path_length * 3 * config.population * sizeof(float)));
    //  CHECK_CUDA(cudaMalloc((void **) &paths.pollinatedPaths, config.path_length * 3 * config.population * sizeof(float)));
    CHECK_CUDA(cudaMalloc((void **) &paths.fitnesses, config.population * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.bestFitness, 1 * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.N_wps, config.population * sizeof(float)));

    /*
        float** rawPaths;

        //need to do malloc2D stuff

        cudaMallocHost(&rawPaths, config.path_length * 3 * config.population * sizeof(float));
        if (rawPaths == NULL) {j
            fprintf(stderr, "amemory allocation failed!\n");
            return;
        }

        rawPaths = generateSolutions_cuda(init,config.path_length, config.population);

        cudaMemcpy(paths.rawPaths, rawPaths, config.path_length * 3 * config.population * sizeof(float), cudaMemcpyHostToDevice);

    */
    float *test_Nwps;
    CHECK_CUDA(cudaMallocHost(&test_Nwps, config.population * sizeof(float)));

    for (int i = 0; i < config.population; i++) {
        test_Nwps[i] = i + 0.0;
    }
    CHECK_CUDA(cudaMemcpy(paths.fitnesses, test_Nwps, config.population * sizeof(float), cudaMemcpyHostToDevice));


    double smoothing_start_time = omp_get_wtime();

    smoothPaths_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, drone.turn_radius, drone.turn_radius * 2,
                                            pitch);
    cudaDeviceSynchronize();

    double smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

    double fitness_start_time = omp_get_wtime();


        computeFitnesses_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, config.heightMap_cuda, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                     config.resolution
                     );
    cudaDeviceSynchronize();

    double fitness_time_taken = omp_get_wtime() - fitness_start_time;


    printf("Iteration: ");
    for (int i = 0; i < config.iter_max; i++) {
        pollination_start_time = omp_get_wtime();

        //pollinate_parallel(paths, config.p_switch);

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, drone.turn_radius,
                                                drone.turn_radius * 2, pitch);
        cudaDeviceSynchronize();

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        //computeFitnesses(paths, config.heightMap, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
        //               config.resolution);

        computeFitnesses_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, config.heightMap_cuda, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia,
                             config.resolution
                             );
        cudaDeviceSynchronize();

        fitness_time_taken += omp_get_wtime() - fitness_start_time;


        int quarter = std::ceil(config.iter_max / 4.0);
        int half = std::ceil(config.iter_max / 2.0);
        int eight = std::ceil(config.iter_max / 8.0);
        /*
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
        */

        if (i % config.two_opt_freq == 0) {
            printf("%i ", i);
            /*
            twoopt_start_time = omp_get_wtime();

            twoOptParallel(paths, drone.turn_radius, drone.turn_radius * 2, config.heightMap, drone.max_asc_angle,
                           drone.max_desc_angle, a_utopia, f_utopia, config.resolution);

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;

            computeBestFitness(paths);
            */
        }
        //   computeBestFitness(paths);
    }
    printf("\n");
    /*
        std::vector<float> smoothedPath = smoothPath_cuda(
                paths.fittestPath,
                drone.turn_radius, drone.turn_radius * 2, nwp);

        save_to_csv(smoothedPath, "../heightMapper/fittest4.csv");

    */

    float* bestFitness_h;

    CHECK_CUDA(cudaMallocHost(&bestFitness_h, 1 * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(bestFitness_h, paths.bestFitness, 1 * sizeof(float),
                              cudaMemcpyDeviceToHost));

    float* fitnesses_h;

    CHECK_CUDA(cudaMallocHost(&fitnesses_h, paths.rawPaths.n_paths * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(fitnesses_h, paths.fitnesses, paths.rawPaths.n_paths * sizeof(float),
                              cudaMemcpyDeviceToHost));

    double totalTime = pollination_time_taken + smoothing_time_taken + fitness_time_taken + twoopt_time_taken;

    printf("\nPollination, Smoothing, Fitness, 2-opt:\n%.2f, %.2f, %.2f, %.2f\n", pollination_time_taken / totalTime,
           smoothing_time_taken / totalTime, fitness_time_taken / totalTime, twoopt_time_taken / totalTime);

    printf("CUDA Algorithm time: %f Reached Fitness: %f\n", totalTime, bestFitness_h[0]);

    for (int i = 0; i < paths.rawPaths.n_paths; i++) {
        //printf("%f \n", fitnesses_h[i]);
    }

    float *hostSmoothedPaths = new float[smoothed_paths_size];

    CHECK_CUDA(cudaMallocHost(&hostSmoothedPaths, smoothed_paths_size));
    CHECK_CUDA(cudaMemcpy(hostSmoothedPaths, paths.smoothedPaths.elements, smoothed_paths_size,
                          cudaMemcpyDeviceToHost));

    float *res;

    cudaMallocHost(&res, config.population * sizeof(float));
    if (res == NULL) {
        fprintf(stderr, "bmemory allocation failed!\n");
        return;
    }

    cudaMemcpy(res, paths.N_wps, config.population * sizeof(float), cudaMemcpyDeviceToHost);

    save_to_csv_cuda(hostSmoothedPaths, max_waypoints_smoothed * 3, "../heightMapper/fittest5.csv");

    for (int i = 0; i < config.population; i++) {
        /*
        printf("RAW path: ");


        for (int j = 0; j < paths.rawPaths.n_waypoints * 3; j++) {
            if (hostPtr[i *  paths.rawPaths.n_waypoints * 3 + j] != 0.0) {
                printf("%.2f ", hostPtr[i * paths.rawPaths.n_waypoints * 3 + j]);
            } else {
                printf(".");
            }

        }

        //printf("%f  ", rawPaths[i][5]);
         printf("\npath: ");

        for (int j = 0; j < max_waypoints_smoothed * 3; j++) {
            if (hostSmoothedPaths[i * max_waypoints_smoothed * 3 + j] != 0.0) {
                printf("%.2f ", hostSmoothedPaths[i * max_waypoints_smoothed * 3 + j]);
            } else {
                printf(".");
            }

        }
*/
        //if (hostSmoothedPaths[i * paths.smoothedPaths.n_waypoints * 3] > -1.0) {

        // }
         //printf("\n");
    }

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


    //delete[] hostPtr;
    //delete[] hostPtr2;
   // delete[] hostSmoothedPaths;
    cudaFree(hostSmoothedPaths);
    cudaFree(hostPtr);
    cudaFree(hostPtr2);
}
