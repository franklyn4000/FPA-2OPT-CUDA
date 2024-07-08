//
// Created by franklyn on 6/15/24.
//

#include "fitnessComputer_cuda.cuh"
#include <iostream>

__device__ float computeFitness_cuda(Paths_cuda paths,
                     const float* heightMap,
                                     int heightMapWidth,
                     int path_index,
                     int startIndex,
                     float N_wp, float max_asc_angle,
                     float max_desc_angle, float a_utopia, float f_utopia, float resolution) {


    float w1 = 0.35;
    float w2 = 0.65;

    float d_ug = 0.0;
    float d_dz = 0.0;
    float d_ea = 0.0;
    float l_traj = 0.0;

    float a_cum = 0;
    float a_avg = 0;
    float f_avg = 0;

    int n = paths.smoothedPaths.used_waypoints[path_index];

    int totalSteps = 1;

    bool underground = false;
    bool undergroundLast = false;

    float P1[3];
    float P2[3];

    float interval_x;
    float interval_y;
    float interval_z;

    float steps_P1P2;
    float inv_steps;
    float step_length_P1P2;

    for (int i = 0; i < n - 1; i++) {
        P1[0] = paths.smoothedPaths.elements[startIndex + i * 3 + 0];
        P1[1] = paths.smoothedPaths.elements[startIndex + i * 3 + 1];
        P1[2] = paths.smoothedPaths.elements[startIndex + i * 3 + 2];
        P2[0] = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 0];
        P2[1] = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 1];
        P2[2] = paths.smoothedPaths.elements[startIndex + (i + 1) * 3 + 2];

        float diff_x = P2[0] - P1[0];
        float diff_y = P2[1] - P1[1];
        float diff_z = P2[2] - P1[2];

        float distance_P1P2 = sqrt(
                diff_x * diff_x +
                diff_y * diff_y +
                diff_z * diff_z
        );

        steps_P1P2 = floor(distance_P1P2 * resolution);


        if (steps_P1P2 > 0) {
            inv_steps = 1 / steps_P1P2;
            step_length_P1P2 = distance_P1P2 * inv_steps;
            interval_x = diff_x * inv_steps;
            interval_y = diff_y * inv_steps;
            interval_z = diff_z * inv_steps;
        } else {
            step_length_P1P2 = distance_P1P2;
            interval_x = 0;
            interval_y = 0;
            interval_z = 0;
        }

        float horizontal_length = sqrt(diff_x * diff_x + diff_y * diff_y);
        float angle_radians = atan2(diff_z, horizontal_length);
        float currentAltitude = 0.0f;

        for (int j = 1; j < steps_P1P2; j++) {

            int pointX = __float2int_rd(P1[0] + interval_x * j);
            int pointY = __float2int_rd(P1[1] + interval_y * j);
            int pointZ = __float2int_rd(P1[2] + interval_z * j);


            currentAltitude = pointZ - heightMap[pointY * heightMapWidth + pointX];
            underground = currentAltitude < a_utopia;
            a_cum += currentAltitude;


            if (underground && undergroundLast) {
                d_ug += step_length_P1P2;
            } else if (underground != undergroundLast) {
                d_ug += step_length_P1P2 * 0.5f;
           }
            undergroundLast = underground;
            l_traj += step_length_P1P2;
            totalSteps++;
        }



        int p2X = __float2int_rd(P2[0]);
        int p2Y = __float2int_rd(P2[1]);
        int p2Z = __float2int_rd(P2[2]);


        currentAltitude = p2Z - heightMap[p2Y * heightMapWidth + p2X];
        underground = currentAltitude < a_utopia;
        a_cum += currentAltitude;


        if (underground && undergroundLast) {
            d_ug += step_length_P1P2;
        } else if (underground != undergroundLast) {
            d_ug += step_length_P1P2 * 0.5;
        }

        undergroundLast = underground;
        l_traj += step_length_P1P2;



        if (angle_radians > max_asc_angle || angle_radians < max_desc_angle) {
            d_ea += distance_P1P2;
        }

        totalSteps++;
    }


    //Penaly term P
    float P = d_ug + d_dz + d_ea + (N_wp * l_traj);


    // printf("a_avg: %f, a_utopia: %f, l_traj: %f, f_utopia: %f \n", a_avg, a_utopia, l_traj, f_utopia);


   // printf("%i -- %.2f \n", path_index, d_ea);

    //Fitness function F
    if (P == 0.0) {
        //Cost term C

        a_avg = a_cum / totalSteps;
        float C = w1 * (a_avg / a_utopia) + w2 * (l_traj / f_utopia);


        paths.fitnesses[path_index] = 1 + 1 / (1 + C);
    } else {
        paths.fitnesses[path_index] = 0 + 1 / (1 + P);
    }

  //  printf("fit: %f \n",  paths.fitnesses[path_index]);

}

__global__ void computeFitnesses_cuda(
        Paths_cuda paths,
        int max_elements,
        const float* heightMap,
        int heightMapWidth, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution) {


    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < paths.rawPaths.n_paths) {



        computeFitness_cuda(paths, heightMap, heightMapWidth,idx, idx * max_elements, paths.N_wps[idx],
                                       max_asc_angle,
                                       max_desc_angle,
                                       a_utopia, f_utopia, resolution);


    }


    computeBestFitness_cuda(paths);


}

__device__ __forceinline__ float atomicMaxFloat (float * addr, float value) {
    return __int_as_float(atomicMax((int *)addr, __float_as_int(value)));
}

__device__ void computeBestFitness_cuda(Paths_cuda paths) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    float fitness = idx < paths.rawPaths.n_paths ? paths.fitnesses[idx] : 0.0;

    for(int i = 1; i < 32; i *=2) {
        fitness = max(fitness, __shfl_xor_sync(-1, fitness, i));
    }

    if(threadIdx.x % 32 == 0) {
        atomicMaxFloat(paths.bestFitness, fitness);

       // paths.bestFitnessStartIndex[0] = max(paths.bestFitness[0], fitness);
    }

  //  printf("max: %f\n", __int_as_float(((int *)paths.bestFitness)[0]));

    if(__int_as_float(((int *)paths.bestFitness)[0]) == paths.fitnesses[idx]) {
        //paths.fittestPathIndex = idx;

        //TODO race condition?
        for(int i = 0; i < paths.rawPaths.n_waypoints * 3; i++) {
            paths.fittestPath[i] = paths.rawPaths.elements[idx * paths.rawPaths.n_waypoints * 3 + i];
        }

    }


 /*
    for (int index = 0; index < paths.population; index++) {

        if (paths.fitnesses[index] > paths.bestFitness) {
            paths.fittestPath = paths.rawPaths[index];
            paths.bestFitness = paths.fitnesses[index];
        }
    }
    */

}
