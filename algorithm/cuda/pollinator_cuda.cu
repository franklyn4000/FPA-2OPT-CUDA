//
// Created by franklyn on 6/17/24.
//

#include "pollinator_cuda.cuh"
#include <iostream>

__global__ void setup_curand_kernel(curandStatePhilox4_32_10_t *state)
{
        int idx = threadIdx.x + blockIdx.x * blockDim.x;
        /* Each thread gets same seed, a different sequence
           number, no offset */
        curand_init(1234, idx, 0, &state[idx]);
}
/*
__global__ void generate_normal_kernel(curandStatePhilox4_32_10_t *state,
                                int n,
                                unsigned int *result)
{
        int id = threadIdx.x + blockIdx.x * blockDim.x;
        unsigned int count = 0;
        float2 x;

        curandStatePhilox4_32_10_t localState = state[id];
        for(int i = 0; i < n/2; i++) {
                x = curand_normal2(&localState);

                if((x.x > -1.0) && (x.x < 1.0)) {
                        count++;
                }
                if((x.y > -1.0) && (x.y < 1.0)) {
                        count++;
                }
        }

        state[id] = localState;

        result[id] += count;
}
*/

__global__ void pollinate_cuda(
        Paths_cuda paths,
        float p_switch,
        curandStatePhilox4_32_10_t *curandState) {

        int idx = threadIdx.x + blockIdx.x * blockDim.x;

        if (idx < paths.rawPaths.n_paths) {
                curandStatePhilox4_32_10_t localState = curandState[idx];

                int n = paths.rawPaths.n_waypoints;

                float4 dis = curand_uniform4(&localState);



                // printf("%f\n", dis.w);

                int pathIndex = idx * paths.rawPaths.n_waypoints * 3;

                if(dis.w < p_switch) {
                        //global pollinate

                        // TODO precompute large array of levy numbers?
                        //float *L = levy_p(paths.rawPaths.n_waypoints * 3, dis.x);
                        //  float gamma = 0.1;

                        paths.pollinatedPaths.elements[pathIndex + 3 * 0] = paths.rawPaths.elements[pathIndex + 3 * 0];
                        paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 1] = paths.rawPaths.elements[pathIndex + 3 * 0 + 1];
                        paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 2] = paths.rawPaths.elements[pathIndex + 3 * 0 + 2];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1];


                        for (int i = 1; i < paths.rawPaths.n_waypoints - 1; i++) {

                                float coord = paths.rawPaths.elements[pathIndex + 3 * i + 0];
                                float coord1 = paths.rawPaths.elements[pathIndex + 3 * i + 1];
                                float coord2 = paths.rawPaths.elements[pathIndex + 3 * i + 2];
                         //   paths.pollinatedPaths.elements[pathIndex + 3 * i] = paths.rawPaths.elements[pathIndex + 3 * i];
                         //   paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = paths.rawPaths.elements[pathIndex + 3 * i + 1];
                       //     paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = paths.rawPaths.elements[pathIndex + 3 * i + 2];



                            paths.pollinatedPaths.elements[pathIndex + 3 * i + 0] = coord + dis.z * (paths.fittestPath[3 * i + 0] - coord);
                            paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = coord1 + dis.z * (paths.fittestPath[3 * i + 1] - coord1);
                            paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = coord2 + dis.z * (paths.fittestPath[3 * i + 2] - coord2);

                               // paths.pollinatedPaths[pathIndex][3 * i + 0] =
                               //         coord + L[3 * i] * (paths.fittestPath[3 * i + k] - coord);


                        }

                } else {

                        //local pollinate
                        //float epsilon = eps(gen);
                        //epsilon = dis.x;

                        //std::uniform_int_distribution<> rand(0, paths.population-1);
                        // int j = rand(gen);
                        int j = __float2int_rd(dis.y * paths.rawPaths.n_waypoints * 3 -1);

                        // std::uniform_int_distribution<> rand2(0, paths.population-1);
                        //int l = rand2(gen);
                        int l = __float2int_rd(dis.z * paths.rawPaths.n_waypoints * 3 -1);

                        paths.pollinatedPaths.elements[pathIndex + 3 * 0] = paths.rawPaths.elements[pathIndex + 3 * 0];
                        paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 1] = paths.rawPaths.elements[pathIndex + 3 * 0 + 1];
                        paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 2] = paths.rawPaths.elements[pathIndex + 3 * 0 + 2];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 3];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 2];
                        paths.pollinatedPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1] = paths.rawPaths.elements[pathIndex + 3 * paths.rawPaths.n_waypoints - 1];

                        for (int i = 1; i < paths.rawPaths.n_waypoints - 1; i++) {

                                paths.pollinatedPaths.elements[pathIndex + 3 * i] = paths.rawPaths.elements[pathIndex + 3 * i] + dis.x * (paths.rawPaths.elements[j + 3 * i] - paths.rawPaths.elements[l + 3 * i]);
                                paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = paths.rawPaths.elements[pathIndex + 3 * i + 1] + dis.x * (paths.rawPaths.elements[j + 3 * i + 1] - paths.rawPaths.elements[l + 3 * i + 1]);
                                paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = paths.rawPaths.elements[pathIndex + 3 * i + 2] + dis.x * (paths.rawPaths.elements[j + 3 * i + 2] - paths.rawPaths.elements[l + 3 * i + 2]);

                                //for (int k = 0; k < 3; k++) {


                                     //  paths.pollinatedPaths.elements[0] = 1.0;// + dis.x * (paths.rawPaths.elements[j + 3 * i + k] - paths.rawPaths.elements[k + 3 * i + k]);
                               // }
                        }

                }


                //  paths.rawPaths = paths.pollinatedPaths;
                curandState[idx] = localState;
        }
}