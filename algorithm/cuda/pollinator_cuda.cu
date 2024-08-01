//
// Created by franklyn on 6/17/24.
//

#include "pollinator_cuda.cuh"
#include <iostream>

__global__ void setup_curand_kernel(curandStatePhilox4_32_10_t *state) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(4321, idx, 0, &state[idx]);
}


__device__ float4 levy_device(curandStatePhilox4_32_10_t localState, float sig_p, float oneOverBeta) {
    float4 levy;

    float4 norm = curand_normal4(&localState);
    float2 norm2 = curand_normal2(&localState);

    levy.x = abs(0.2 * norm.x * sig_p / pow(abs(norm.y), oneOverBeta));
    levy.y = abs(0.2 * norm.w * sig_p / pow(abs(norm.z), oneOverBeta));
    levy.z = abs(0.2 * norm2.x * sig_p / pow(abs(norm2.y), oneOverBeta));


    return levy;
}

__global__ void pollinate_cuda(
        Paths_cuda paths,
        float p_switch,
        int heightMapWidth,
        int heightMapHeight,
        curandStatePhilox4_32_10_t *curandState) {

    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    float heightMapWidth_f = __int2float_rd(heightMapWidth) - 1.0f;
    float heightMapHeight_f = __int2float_rd(heightMapHeight) - 1.0f;

    float beta = 1.5;
    float oneOverBeta = 1.0 / beta;
    float sig_p = pow(tgamma(1.0 + beta) * sin(M_PI * beta / 2.0) /
                      (tgamma((1.0 + beta) / 2.0) * beta * pow(2.0, (beta - 1.0) / 2.0)), oneOverBeta);

    float4 coord;
    float4 val;
    float4 bounded;

    if (idx < paths.rawPaths.n_paths) {
        curandStatePhilox4_32_10_t localState = curandState[idx];
        float4 dis = curand_uniform4(&localState);

        int pathIndex = idx * paths.rawPaths.n_waypoints * 3;

        if (dis.w < p_switch) {
            //global pollinate

            float4 L = levy_device(localState, sig_p, oneOverBeta);


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
                coord.x = paths.rawPaths.elements[pathIndex + 3 * i + 0];
                coord.y = paths.rawPaths.elements[pathIndex + 3 * i + 1];
                coord.z = paths.rawPaths.elements[pathIndex + 3 * i + 2];

                val.x = coord.x + L.x * (paths.fittestPath[3 * i + 0] - coord.x);
                val.y = coord.y + L.y * (paths.fittestPath[3 * i + 1] - coord.y);
                val.z = coord.z + L.z * (paths.fittestPath[3 * i + 2] - coord.z);

                bounded.x = max(min(val.x, heightMapWidth_f), 0.0f);
                bounded.y = max(min(val.y, heightMapHeight_f), 0.0f);
                bounded.z = max(min(val.z, 2799.9f), 1900.0f); //TODO get vlues from config!

                paths.pollinatedPaths.elements[pathIndex + 3 * i + 0] = bounded.x;
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = bounded.y;
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 2] = bounded.z;
            }

        } else {
            int j = __float2int_rd(dis.y * (paths.rawPaths.n_waypoints * 3 - 1));
            int l = __float2int_rd(dis.z * (paths.rawPaths.n_waypoints * 3 - 1));

            paths.pollinatedPaths.elements[pathIndex + 3 * 0] = paths.rawPaths.elements[pathIndex + 3 * 0];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 1] = paths.rawPaths.elements[pathIndex + 3 * 0 + 1];
            paths.pollinatedPaths.elements[pathIndex + 3 * 0 + 2] = paths.rawPaths.elements[pathIndex + 3 * 0 + 2];

            for (int i = 1; i < paths.rawPaths.n_waypoints - 1; i++) {
                float inc = paths.rawPaths.elements[pathIndex + 3 * i + 0] +
                            dis.x * (paths.rawPaths.elements[j + 3 * i + 0] - paths.rawPaths.elements[l + 3 * i + 0]);
                float boundedInc = max(min(inc, heightMapWidth_f), 0.0f);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 0] = boundedInc;

                inc = paths.rawPaths.elements[pathIndex + 3 * i + 1] +
                      dis.x * (paths.rawPaths.elements[j + 3 * i + 1] - paths.rawPaths.elements[l + 3 * i + 1]);
                boundedInc = max(min(inc, heightMapHeight_f), 0.0f);
                paths.pollinatedPaths.elements[pathIndex + 3 * i + 1] = boundedInc;

                inc = paths.rawPaths.elements[pathIndex + 3 * i + 2] +
                      dis.x * (paths.rawPaths.elements[j + 3 * i + 2] - paths.rawPaths.elements[l + 3 * i + 2]);
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