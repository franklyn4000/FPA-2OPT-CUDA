//
// Created by franklyn on 6/17/24.
//

#ifndef POLLINATOR_CUDA_CUH
#define POLLINATOR_CUDA_CUH

#include "../objects/paths.h"
#include <curand_kernel.h>

__global__ void pollinate_cuda(
        Paths_cuda paths,
        float p_switch,
        int heightMapWidth,
        int heightMapHeight,
        curandStatePhilox4_32_10_t *curandState);

__global__ void setup_curand_kernel(curandStatePhilox4_32_10_t *state);

#endif //POLLINATOR_CUDA_CUH
