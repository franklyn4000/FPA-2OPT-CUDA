//
// Created by franklyn on 6/15/24.
//

#include "fitnessComputer_cuda.cuh"
#include <iostream>

__device__ void computeFitness_cuda(Paths_cuda paths,
                                    const float *heightMap,
                                    int heightMapWidth,
                                    int path_index,
                                    int startIndex,
                                    float N_wp, float max_asc_angle,
                                    float max_desc_angle, float a_utopia, float f_utopia, float resolution, float w1,
                                    float w2) {
    float d_ug = 0.0;
    float d_dz = 0.0;
    float d_ea = 0.0;
    float l_traj = 0.0;

    float a_cum = 0;
    float a_avg = 0;
    //float f_avg = 0;

    int n = paths.smoothedPaths.used_waypoints[path_index];

    int totalSteps = 1;

    int underground = 0;

    float4 P1_v;
    float4 P2_v;

    float steps_P1P2;
    float inv_steps;
    float step_length_P1P2;

    float4 diff;
    float4 interval;
    int2 pointXY;
    float pointZ;
    float currentAltitude;

    for (int i = 0; i < n - 1; i++) {
        P1_v.x = paths.smoothedPaths.elements[startIndex + i * 3 + 0];
        P1_v.y = paths.smoothedPaths.elements[startIndex + i * 3 + 1];
        P1_v.z = paths.smoothedPaths.elements[startIndex + i * 3 + 2];
        P2_v.x = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 0];
        P2_v.y = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 1];
        P2_v.z = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 2];

        diff.x = P2_v.x - P1_v.x;
        diff.y = P2_v.y - P1_v.y;
        diff.z = P2_v.z - P1_v.z;

        float distance_P1P2 = norm3df(
                diff.x,
                diff.y,
                diff.z
        );

        steps_P1P2 = floor(distance_P1P2 * resolution);


        if (steps_P1P2 > 0) {
            inv_steps = 1 / steps_P1P2;
            step_length_P1P2 = distance_P1P2 * inv_steps;
            interval.x = diff.x * inv_steps;
            interval.y = diff.y * inv_steps;
            interval.z = diff.z * inv_steps;
        } else {
            step_length_P1P2 = distance_P1P2;
            interval.x = 0;
            interval.y = 0;
            interval.z = 0;
        }

        float horizontal_length = hypotf(diff.x, diff.y);
        float angle_radians = atan2(diff.z, horizontal_length);


        l_traj += distance_P1P2;
        totalSteps += max(0.0f, steps_P1P2 - 2);
        for (float j = 1.0f; j < steps_P1P2; j++) {
            pointXY.x = __float2int_rn(P1_v.x + interval.x * j);
            pointXY.y = __float2int_rn(P1_v.y + interval.y * j);
            pointZ = P1_v.z + interval.z * j;

            currentAltitude = pointZ - heightMap[pointXY.x * heightMapWidth + pointXY.y];
            underground = currentAltitude < a_utopia;

            d_ug += step_length_P1P2 * underground;
            a_cum += currentAltitude;
        }

        pointXY.x = __float2int_rn(P2_v.x);
        pointXY.y = __float2int_rn(P2_v.y);

        currentAltitude = P2_v.z - heightMap[pointXY.x * heightMapWidth + pointXY.y];

        underground = currentAltitude < a_utopia;
        d_ug += step_length_P1P2 * underground;
        a_cum += currentAltitude;

        if (angle_radians > max_asc_angle || angle_radians < max_desc_angle) {
            d_ea += distance_P1P2;
        }

        totalSteps++;
    }


    //Penaly term P
    float P = d_ug + d_dz + d_ea + (N_wp * l_traj);

    //Fitness function F
    if (P == 0.0) {
        //Cost term C
        a_avg = a_cum / totalSteps;
        float C = w1 * (a_avg / a_utopia) + w2 * (l_traj / f_utopia);

        paths.fitnesses[path_index] = 1 + 1 / (1 + C);
    } else {
        paths.fitnesses[path_index] = 0 + 1 / (1 + P);
    }

}

__global__ void computeFitnesses_cuda(
        Paths_cuda paths,
        int max_elements,
        const float *heightMap,
        int heightMapWidth, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution, float w1, float w2) {

    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < paths.rawPaths.n_paths) {
        computeFitness_cuda(paths, heightMap, heightMapWidth, idx, idx * max_elements, paths.N_wps[idx],
                            max_asc_angle,
                            max_desc_angle,
                            a_utopia, f_utopia, resolution, w1, w2);


    }

}

__device__ __forceinline__ float atomicMaxFloat(float *addr, float value) {
    return __int_as_float(atomicMax((int *) addr, __float_as_int(value)));
}

__global__ void computeBestFitness_cuda(Paths_cuda paths) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    float fitness = idx < paths.rawPaths.n_paths ? paths.fitnesses[idx] : 0.0;

    for (int i = 1; i < 32; i *= 2) {
        fitness = max(fitness, __shfl_xor_sync(0xffffffff, fitness, i));
    }

    if (threadIdx.x % 32 == 0) {
        atomicMaxFloat(paths.bestFitness, fitness);
    }

    if (idx < paths.rawPaths.n_paths && __int_as_float(((int *) paths.bestFitness)[0]) == paths.fitnesses[idx]) {
        for (int i = 0; i < paths.rawPaths.n_waypoints * 3; i++) {
            paths.fittestPath[i] = paths.rawPaths.elements[idx * paths.rawPaths.n_waypoints * 3 + i];
        }
    }

}
