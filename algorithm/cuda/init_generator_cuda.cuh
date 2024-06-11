//
// Created by franklyn on 6/5/24.
//

#ifndef INIT_GENERATOR_CUDA_CUH
#define INIT_GENERATOR_CUDA_CUH

#include "../objects/initialConditions.h"

float* generateSolutions_cuda(
    InitialConditions& init,
    int n_waypoints, int population);

#endif //INIT_GENERATOR_CUDA_CUH
