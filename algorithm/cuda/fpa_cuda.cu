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
#include "pollinator_cuda.cuh"
#include "twoOpt_cuda.cuh"
#include "../parallel/pathSmoother_parallel.h"

#include <thrust/copy.h>
#include <thrust/device_ptr.h>

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

struct is_not_zero
{
    __host__ __device__
    bool operator()( double x)
    {
        return (x >= 0);
    }
};

Results computeFPA_cuda(
        Config &config, float* heightMap_h, Drone &drone, InitialConditions &init) {
    float a_utopia = drone.min_altitude;
    float f_utopia = calculateFUtopia(init);

	double data_transfer_start_time = 0;
    double data_transfer_time_taken = 0;
    double pollination_start_time = 0;
    double pollination_time_taken = 0;
    double twoopt_start_time = 0;
    double twoopt_time_taken = 0;
	double smoothing_start_time = 0;
	double smoothing_time_taken = 0;
	double fitness_start_time = 0;
	double fitness_time_taken = 0;
	double total_start_time = 0;
	double total_time_taken = 0;
    double iteration_start_time = 0;
	double iteration_time_taken = 0;
    int iterations = 0;

	total_start_time = omp_get_wtime();

    curandStatePhilox4_32_10_t *devPHILOXStates;

    int max_waypoints_smoothed = config.path_length * config.n_pi + 1; // upper bound

    Paths_cuda paths;

    dim3 dimBlock(256);
    dim3 dimGrid((config.population + dimBlock.x - 1) / dimBlock.x);

    data_transfer_start_time = omp_get_wtime();

    paths.smoothedPaths.n_waypoints = max_waypoints_smoothed;
    paths.smoothedPaths.n_paths = config.population;

    paths.rawPaths.n_waypoints = config.path_length;
    paths.rawPaths.n_paths = config.population;

    size_t raw_paths_size = paths.rawPaths.n_waypoints * 3 * paths.rawPaths.n_paths * sizeof(float);
    size_t smoothed_paths_size = config.population * max_waypoints_smoothed * 3 * sizeof(float);
    size_t temp_paths_size = config.population * 32 * paths.rawPaths.n_waypoints * 3 * sizeof(float);
    size_t temp_smoothed_paths_size = config.population * 32 * max_waypoints_smoothed * 3 * sizeof(float);

    printf("VRAM allocated %lu \n", (unsigned long) raw_paths_size + smoothed_paths_size + temp_paths_size + temp_smoothed_paths_size);

    CHECK_CUDA(cudaMalloc(&config.heightMap_cuda, config.heightMap_rows * config.heightMap_cols * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(config.heightMap_cuda, heightMap_h, config.heightMap_rows * config.heightMap_cols * sizeof(float), cudaMemcpyHostToDevice));

    CHECK_CUDA(cudaMalloc(&paths.tempPaths.elements, temp_paths_size));
    CHECK_CUDA(cudaMalloc(&paths.tempSmoothedPaths.elements, temp_smoothed_paths_size));

    CHECK_CUDA(cudaMalloc(&paths.smoothedPaths.elements, smoothed_paths_size));
    CHECK_CUDA(cudaMalloc(&paths.smoothedPaths.used_waypoints, config.population * sizeof(int)));

    CHECK_CUDA(cudaMalloc(&paths.rawPaths.elements, raw_paths_size));

    CHECK_CUDA(cudaMalloc(&paths.pollinatedPaths.elements, raw_paths_size));

    CHECK_CUDA(cudaMalloc(&paths.fittestPath, paths.rawPaths.n_waypoints * 3 * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.fitnesses, config.population * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.bestFitness, 1 * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.N_wps, config.population * sizeof(float)));

    CHECK_CUDA(cudaMalloc((void **) &paths.twoOptFinishedSolutions, config.population * sizeof(int)));

    CHECK_CUDA(cudaMalloc((void **) &paths.twoOptCurrentI, config.population * sizeof(int)));
    CHECK_CUDA(cudaMalloc((void **) &paths.twoOptCurrentJ, config.population * sizeof(int)));

    // allocate one curandState per thread on device
    CHECK_CUDA(cudaMalloc((void **)&devPHILOXStates, dimBlock.x * dimGrid.x * sizeof(curandStatePhilox4_32_10_t)));
    setup_curand_kernel<<<dimGrid, dimBlock>>>(devPHILOXStates);

    generateSolutions_cuda<<<dimGrid, dimBlock>>>(paths, init, config.path_length, config.population, devPHILOXStates);
    cudaDeviceSynchronize();

    smoothing_start_time = omp_get_wtime();

    smoothPaths_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, drone.turn_radius, config.n_pi);

    cudaDeviceSynchronize();

    smoothing_time_taken = omp_get_wtime() - smoothing_start_time;

    fitness_start_time = omp_get_wtime();

    computeFitnesses_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, config.heightMap_cuda, config.heightMap_rows, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia, config.resolution, config.w1, config.w2);
    cudaDeviceSynchronize();

    computeBestFitness_cuda<<<dimGrid, dimBlock>>>(paths);

    fitness_time_taken = omp_get_wtime() - fitness_start_time;
    total_time_taken += omp_get_wtime() - total_start_time;

    data_transfer_time_taken = omp_get_wtime() - data_transfer_start_time;

    printf("Iteration: ");
    while (iterations < config.iter_max && total_time_taken < config.time_limit) {
         iteration_start_time = omp_get_wtime();
        pollination_start_time = omp_get_wtime();

        //pollinate_parallel(paths, config.p_switch);
        pollinate_cuda<<<dimGrid, dimBlock>>>(paths, config.p_switch, config.heightMap_cols, config.heightMap_rows, devPHILOXStates);
        cudaDeviceSynchronize();

       // paths.fittestPathIndex = 0;

        CHECK_CUDA(cudaMemcpy(paths.rawPaths.elements, paths.pollinatedPaths.elements, raw_paths_size, cudaMemcpyDeviceToDevice));

        pollination_time_taken += omp_get_wtime() - pollination_start_time;

        smoothing_start_time = omp_get_wtime();

        smoothPaths_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, drone.turn_radius, config.n_pi);
        cudaDeviceSynchronize();

        smoothing_time_taken += omp_get_wtime() - smoothing_start_time;

        fitness_start_time = omp_get_wtime();

        computeFitnesses_cuda<<<dimGrid, dimBlock>>>(paths, max_waypoints_smoothed * 3, config.heightMap_cuda, config.heightMap_rows, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia, config.resolution, config.w1, config.w2);
        cudaDeviceSynchronize();
        computeBestFitness_cuda<<<dimGrid, dimBlock>>>(paths);

        fitness_time_taken += omp_get_wtime() - fitness_start_time;


        if (iterations % config.two_opt_freq == 0) {

            printf("%i ", iterations);

            twoopt_start_time = omp_get_wtime();

            int* initFinishedSolutions = new int[config.population];

            for(int i = 0; i < config.population; i++) {
                initFinishedSolutions[i] = i;
            }

            CHECK_CUDA(cudaMalloc(&paths.twoOptFinishedSolutions, config.population * sizeof(int)));
            CHECK_CUDA(cudaMemcpy(paths.twoOptFinishedSolutions, initFinishedSolutions, config.population * sizeof(int), cudaMemcpyHostToDevice));

            int* twoOptCountFinishedSolutions_h = new int[1];
            twoOptCountFinishedSolutions_h[0] = config.population;

            CHECK_CUDA(cudaMalloc(&paths.twoOptCountFinishedSolutions, 1 * sizeof(int)));
            CHECK_CUDA(cudaMemcpy(paths.twoOptCountFinishedSolutions, twoOptCountFinishedSolutions_h, 1 * sizeof(int), cudaMemcpyHostToDevice));



            while(twoOptCountFinishedSolutions_h[0] > 0) {
                dim3 twoOptBlock(32);
                dim3 twoOptGrid(twoOptCountFinishedSolutions_h[0]);

               	twoOptCuda<<<twoOptGrid, twoOptBlock>>>(paths, config, drone, a_utopia, f_utopia, max_waypoints_smoothed);
				cudaDeviceSynchronize();

				thrust::device_ptr<int> output = thrust::device_pointer_cast(paths.twoOptFinishedSolutions);
				thrust::copy_if(paths.twoOptFinishedSolutions, paths.twoOptFinishedSolutions + twoOptCountFinishedSolutions_h[0], output, is_not_zero());

                cudaMemcpy(twoOptCountFinishedSolutions_h, paths.twoOptCountFinishedSolutions, 1 * sizeof(int), cudaMemcpyDeviceToHost);
				cudaMemcpy(initFinishedSolutions, paths.twoOptFinishedSolutions, config.population * sizeof(int), cudaMemcpyDeviceToHost);
            }

            computeBestFitness_cuda<<<dimBlock, dimGrid>>>(paths);

            free(initFinishedSolutions);
            free(twoOptCountFinishedSolutions_h);
            CHECK_CUDA(cudaFree(paths.twoOptCountFinishedSolutions));
            CHECK_CUDA(cudaFree(paths.twoOptFinishedSolutions));

            twoopt_time_taken += omp_get_wtime() - twoopt_start_time;

        }

        iterations++;
        iteration_time_taken = omp_get_wtime() - iteration_start_time;
		total_time_taken += iteration_time_taken;
    }
	iteration_start_time = omp_get_wtime();
    printf("\n");



    float* bestFitness_h;

    CHECK_CUDA(cudaMallocHost(&bestFitness_h, 1 * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(bestFitness_h, paths.bestFitness, 1 * sizeof(float), cudaMemcpyDeviceToHost));

    float* fitnesses_h;

    CHECK_CUDA(cudaMallocHost(&fitnesses_h, paths.rawPaths.n_paths * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(fitnesses_h, paths.fitnesses, paths.rawPaths.n_paths * sizeof(float), cudaMemcpyDeviceToHost));

    double totalTime = data_transfer_time_taken + pollination_time_taken + smoothing_time_taken + fitness_time_taken + twoopt_time_taken;

	iteration_time_taken = omp_get_wtime() - iteration_start_time;
		total_time_taken += iteration_time_taken;


	float *hostPollinatedPaths = new float[raw_paths_size];

    CHECK_CUDA(cudaMallocHost(&hostPollinatedPaths, raw_paths_size));
    CHECK_CUDA(cudaMemcpy(hostPollinatedPaths, paths.pollinatedPaths.elements, raw_paths_size,
                          cudaMemcpyDeviceToHost));

    float *smoothedPaths_h = new float[smoothed_paths_size];

    CHECK_CUDA(cudaMallocHost(&smoothedPaths_h, smoothed_paths_size));
    CHECK_CUDA(cudaMemcpy(smoothedPaths_h, paths.smoothedPaths.elements, smoothed_paths_size,
                          cudaMemcpyDeviceToHost));

    float *bestPath_h = new float[paths.rawPaths.n_waypoints * 3 * sizeof(float)];

    CHECK_CUDA(cudaMallocHost(&bestPath_h, paths.rawPaths.n_waypoints * 3 * sizeof(float)));
    CHECK_CUDA(cudaMemcpy(bestPath_h, paths.fittestPath, paths.rawPaths.n_waypoints * 3 * sizeof(float),
                          cudaMemcpyDeviceToHost));

	std::vector<float> smoothedPath = smoothPath_fromArray(
            bestPath_h, paths.rawPaths.n_waypoints,
            drone.turn_radius, config.n_pi);



    printf("\nPollination, Smoothing, Fitness, 2-opt:\n%.3f, %.3f, %.3f, %.3f, %.3f\n", data_transfer_time_taken / totalTime, pollination_time_taken / totalTime,
           smoothing_time_taken / totalTime, fitness_time_taken / totalTime, twoopt_time_taken / totalTime);

    printf("CUDA Algorithm time: %f %f Reached Fitness: %f after %i iterations\n", total_time_taken, totalTime, bestFitness_h[0], iterations);

    save_to_csv(smoothedPath, "../heightMapper/fittest5.csv");

	Results results;
	results.total_time = total_time_taken;

    cudaFreeHost(smoothedPaths_h);
    cudaFreeHost(bestPath_h);
    CHECK_CUDA(cudaFree(devPHILOXStates));
    CHECK_CUDA(cudaFree(paths.fittestPath));
    CHECK_CUDA(cudaFree(paths.rawPaths.elements));
    CHECK_CUDA(cudaFree(paths.pollinatedPaths.elements));
    CHECK_CUDA(cudaFree(paths.smoothedPaths.elements));
    CHECK_CUDA(cudaFree(paths.tempPaths.elements));
    CHECK_CUDA(cudaFree(paths.tempSmoothedPaths.elements));
	return results;
}
