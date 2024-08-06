//
// Created by franklyn on 6/5/24.
//

#ifndef INIT_GENERATOR_CUDA_CUH
#define INIT_GENERATOR_CUDA_CUH

#include "../objects/initialConditions.h"
#include "../objects/paths.h"
#include <curand_kernel.h>

__global__ void generateSolutions_cuda(
    Paths_cuda paths,
    InitialConditions init,
    int n_waypoints, int population,
    curandStatePhilox4_32_10_t *curandState);

#endif //INIT_GENERATOR_CUDA_CUH
