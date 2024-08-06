#include "init_generator_cuda.cuh"
#include <random>

__global__ void generateSolutions_cuda(
    Paths_cuda paths,
    InitialConditions init,
    int n_waypoints, int population,
    curandStatePhilox4_32_10_t *curandState) {

    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    curandStatePhilox4_32_10_t localState = curandState[idx];

    if (idx < population) {
        paths.rawPaths.elements[idx * n_waypoints * 3 + 0] = init.x1;
        paths.rawPaths.elements[idx * n_waypoints * 3 + 1] = init.y1;
        paths.rawPaths.elements[idx * n_waypoints * 3 + 2] = init.z1;

        for (int j = 1; j < n_waypoints - 1; ++j) {
            float4 dis = curand_uniform4(&localState);

            paths.rawPaths.elements[idx * n_waypoints * 3 + (3 * j)] = dis.x * (init.x_max - init.x_min) + init.x_min;
            paths.rawPaths.elements[idx * n_waypoints * 3 + (3 * j + 1)] = dis.y * (init.y_max - init.y_min) + init.y_min;
            paths.rawPaths.elements[idx * n_waypoints * 3 + (3 * j + 2)] = dis.z * (init.z_max - init.z_min) + init.z_min;
        }

        paths.rawPaths.elements[idx * n_waypoints * 3 +  n_waypoints * 3 - 3] = init.xn;
        paths.rawPaths.elements[idx * n_waypoints * 3 +  n_waypoints * 3 - 2] = init.yn;
        paths.rawPaths.elements[idx * n_waypoints * 3 +  n_waypoints * 3 - 1] = init.zn;
    }

    curandState[idx] = localState;
}