//
// Created by franklyn on 6/17/24.
//

#include "pollinator_cuda.cuh"
#include <iostream>

__global__ void setup_curand_kernel(curandStatePhilox4_32_10_t *state) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(1234, idx, 0, &state[idx]);
}

__global__ void pollinate_cuda(
        Paths_cuda paths,
        float p_switch,
        int heightMapWidth,
        int heightMapHeight,
        curandStatePhilox4_32_10_t *curandState) {

    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if (idx < paths.rawPaths.n_paths) {
        curandStatePhilox4_32_10_t localState = curandState[idx];

        int n = paths.rawPaths.n_waypoints;

        float4 dis = curand_uniform4(&localState);



        // printf("%f\n", dis.w);

        int pathIndex = idx * paths.rawPaths.n_waypoints * 3;

        if (dis.w < p_switch) {
            //global pollinate

            // TODO precompute large array of levy numbers?
            //float *L = levy_p(paths.rawPaths.n_waypoints * 3, dis.x);
            //  float gamma = 0.1;

            paths.pollinatedPaths.elements[pathIndex + 3 * 0] = paths.rawPaths.elements[pathIndex + 3 * 0];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 1] = paths.rawPaths.elements[pathIndex + 3 * 0 + 1];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 2] = paths.rawPaths.elements[pathIndex + 3 * 0 + 2];
            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 3];
            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 2];
            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 1];


            for (int i = 1; i < paths.rawPaths.n_waypoints - 1; i++) {

                float coord = paths.rawPaths.elements[pathIndex + 3 * i + 0];
                float coord1 = paths.rawPaths.elements[pathIndex + 3 * i + 1];
                float coord2 = paths.rawPaths.elements[pathIndex + 3 * i + 2];
                //   paths.pollinatedPaths.elements[pathIndex + 3 * i] = paths.rawPaths.elements[pathIndex + 3 * i];
                //   paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = paths.rawPaths.elements[pathIndex + 3 * i + 1];
                //     paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = paths.rawPaths.elements[pathIndex + 3 * i + 2];

                float boundedCoord = max(min(coord, (float)heightMapWidth - 1), 0.0f);
                float boundedCoord1 = max(min(coord1, (float)heightMapHeight - 1), 0.0f);
                float boundedCoord2 = max(min(coord2, 2799.9f), 1900.0f);

                paths.pollinatedPaths.elements[pathIndex + 3 * i + 0] =
                        boundedCoord + dis.z * (paths.fittestPath[3 * i + 0] - boundedCoord);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] =
                        boundedCoord1 + dis.z * (paths.fittestPath[3 * i + 1] - boundedCoord1);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] =
                        boundedCoord2 + dis.z * (paths.fittestPath[3 * i + 2] - boundedCoord2);

                // paths.pollinatedPaths[pathIndex][3 * i + 0] =
                //         coord + L[3 * i] * (paths.fittestPath[3 * i + k] - coord);


            }

        } else {
            int j = __float2int_rd(dis.y * (paths.rawPaths.n_waypoints * 3 - 1));
            int l = __float2int_rd(dis.z * (paths.rawPaths.n_waypoints * 3 - 1));

            paths.pollinatedPaths.elements[pathIndex + 3 * 0] = paths.rawPaths.elements[pathIndex + 3 * 0];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 1] = paths.rawPaths.elements[pathIndex + 3 * 0 + 1];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 2] = paths.rawPaths.elements[pathIndex + 3 * 0 + 2];

            for (int i = 1; i < paths.rawPaths.n_waypoints - 1; i++) {
                float inc = paths.rawPaths.elements[pathIndex + 3 * i + 0] + dis.x * (paths.rawPaths.elements[j + 3 * i + 0] - paths.rawPaths.elements[l + 3 * i + 0]);
                float boundedInc = max(min(inc, (float)heightMapWidth - 1), 0.0f);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 0] = boundedInc;

                inc = paths.rawPaths.elements[pathIndex + 3 * i + 1] + dis.x * (paths.rawPaths.elements[j + 3 * i + 1] - paths.rawPaths.elements[l + 3 * i + 1]);
                boundedInc = max(min(inc, (float)heightMapHeight - 1), 0.0f);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = boundedInc;

                inc = paths.rawPaths.elements[pathIndex + 3 * i + 2] + dis.x * (paths.rawPaths.elements[j + 3 * i + 2] - paths.rawPaths.elements[l + 3 * i + 2]);
                boundedInc = max(min(inc, 2799.9f), 1900.0f);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = boundedInc;

                //  paths.pollinatedPaths.elements[0] = 1.0;// + dis.x * (paths.rawPaths.elements[j + 3 * i + k] - paths.rawPaths.elements[k + 3 * i + k]);}
            }

            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 3];
            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 2];
            paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1] = paths.rawPaths.elements[
                    pathIndex + 3 * paths.rawPaths.n_waypoints - 1];

        }


        //  paths.rawPaths = paths.pollinatedPaths;
        curandState[idx] = localState;
    }
}