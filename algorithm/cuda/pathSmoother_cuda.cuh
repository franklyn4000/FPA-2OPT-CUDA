//
// Created by franklyn on 4/11/24.
//

#ifndef ALGORITHM_PATHSMOOTHER_CUDA_CUH
#define ALGORITHM_PATHSMOOTHER_CUDA_CUH

#include <vector>
#include "../objects/paths.h"

__global__ void smoothPaths_cuda(
    Paths_cuda paths,
    int max_elements,
    float turnRadius, int n_pi, size_t pitch);

__device__ void smoothPath_cuda(
    Paths_cuda paths,
    int smooth_startIndex,
    int raw_startIndex,
    float turnRadius, int n_pi);

#endif //ALGORITHM_PATHSMOOTHER_CUDA_CUH
