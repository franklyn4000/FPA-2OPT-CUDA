//
// Created by franklyn on 7/25/24.
//

#ifndef ALGORITHM_TWOOPT_CUDA_CUH
#define ALGORITHM_TWOOPT_CUDA_CUH

#include "../objects/paths.h"
#include "../objects/config.h"
#include "../objects/drone.h"

__global__ void twoOptCuda(Paths_cuda paths, Config config, Drone drone, float a_utopia, float f_utopia, int max_elements);

#endif //ALGORITHM_TWOOPT_CUDA_CUH
