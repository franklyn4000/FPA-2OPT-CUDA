//
// Created by franklyn on 4/11/24.
//

#ifndef ALGORITHM_PATHSMOOTHER_CUDA_CUH
#define ALGORITHM_PATHSMOOTHER_CUDA_CUH

#include <vector>
#include "../objects/paths.h"

__global__ void smoothPaths_cuda(
        Paths_cuda paths,
        float turnRadius, int n_pi, size_t pitch);

std::vector<float> smoothPath_cuda(
        std::vector<float> &path,
        float turnRadius, int n_pi, float &N_wp);

#endif //ALGORITHM_PATHSMOOTHER_CUDA_CUH
