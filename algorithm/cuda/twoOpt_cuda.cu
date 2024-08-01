//
// Created by franklyn on 7/25/24.
//

#include "twoOpt_cuda.cuh"
#include <iostream>

__global__ void twoOptCuda(
        Paths_cuda paths) {

    int n_points = paths.smoothedPaths.n_waypoints;
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int path_index = blockIdx.x;


    __shared__ int is[32];
    __shared__ int js[32];

    if(threadIdx.x % 32 == 0) {

        int index = 0;

        for (int i = 1; i < n_points - 1; i++) {
            for(int j = i + 1; j < n_points; j++) {
                is[index] = i;
                js[index] = j;
                index++;

                if(index >= 32) {
                    paths.twoOptCurrentI[path_index] = i;
                    paths.twoOptCurrentJ[path_index] = j;
                    break;
                }

            }
            if(index >= 32) {
                break;
            }
        }
    }
    __syncwarp();



    printf("%i %i %i %i %i\n", path_index, threadIdx.x, n_points, is[threadIdx.x], js[threadIdx.x]);


    __syncwarp();

    // if GOTO is reached
    if(threadIdx.x % 32 == 0) {
        paths.twoOptCurrentI = 0;
        paths.twoOptCurrentJ = 0;
    }
}