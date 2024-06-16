//
// Created by franklyn on 4/11/24.
//

#ifndef ALGORITHM_FPA_CUDA_CUH
#define ALGORITHM_FPA_CUDA_CUH

#include <vector>
#include "../objects/config.h"
#include "../objects/drone.h"
#include "../objects/initialConditions.h"

void computeFPA_cuda(
        Config &config, float* heightMap_h, Drone &drone, InitialConditions &init);

#endif //ALGORITHM_FPA_CUDA_CUH
