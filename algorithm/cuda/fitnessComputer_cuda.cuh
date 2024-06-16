//
// Created by franklyn on 6/15/24.
//

#ifndef FITNESSCOMPUTER_CUDA_CUH
#define FITNESSCOMPUTER_CUDA_CUH

#include "../objects/paths.h"

__global__ void computeFitnesses_cuda(
        Paths_cuda paths,
        int max_elements,
        const float* heightMap, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution);

__device__ float computeFitness_cuda(Paths_cuda paths,
         const float* heightMap,
         int path_index,
         int startIndex,
         float N_wp, float max_asc_angle,
         float max_desc_angle, float a_utopia, float f_utopia, float resolution);

__device__ void computeBestFitness_cuda(Paths_cuda paths);

#endif //FITNESSCOMPUTER_CUDA_CUH
