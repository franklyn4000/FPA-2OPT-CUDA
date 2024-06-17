//
// Created by franklyn on 4/11/24.
//

#include "pathSmoother_cuda.cuh"
#include <iostream>

__device__ void smoothPath_cuda(
    Paths_cuda paths,
    int path_index,
    int smooth_startIndex,
    int raw_startIndex,
    float turnRadius, int n_pi) {
   // std::vector<float> smoothedPath;

    // std::vector<float> N_wps;
    float prevCPath = 0.0f;

    float unsmoothedVertices = 0;


    paths.smoothedPaths.elements[smooth_startIndex] = paths.rawPaths.elements[raw_startIndex];
    paths.smoothedPaths.elements[smooth_startIndex + 1] = paths.rawPaths.elements[raw_startIndex + 1];
    paths.smoothedPaths.elements[smooth_startIndex + 2] = paths.rawPaths.elements[raw_startIndex + 2];

    int smoothedPathLength = 1;

    int n = paths.rawPaths.n_waypoints;

    float P[3];
    float P1[3];
    float P2[3];

    float C[3];

    float tau_1[3];
    float tau_2[3];

    float mag_1;
    float mag_2;

    float mag_1_inv;
    float mag_2_inv;

    for (int i = 1; i < n - 1; i++) {
        P1[0] =  paths.rawPaths.elements[raw_startIndex + 3 * (i - 1)];
        P1[1] =  paths.rawPaths.elements[raw_startIndex + 3 * (i - 1) + 1];
        P1[2] =  paths.rawPaths.elements[raw_startIndex + 3 * (i - 1) + 2];

        P[0] = paths.rawPaths.elements[raw_startIndex + 3 * i];
        P[1] = paths.rawPaths.elements[raw_startIndex + 3 * i + 1];
        P[2] = paths.rawPaths.elements[raw_startIndex + 3 * i + 2];

        P2[0] = paths.rawPaths.elements[raw_startIndex + 3 * (i + 1)];
        P2[1] = paths.rawPaths.elements[raw_startIndex + 3 * (i + 1) + 1];
        P2[2] = paths.rawPaths.elements[raw_startIndex + 3 * (i + 1) + 2];

        //unit vector from P1 to P
        mag_1 = sqrt(
            (P[0] - P1[0]) * (P[0] - P1[0]) +
            (P[1] - P1[1]) * (P[1] - P1[1]) +
            (P[2] - P1[2]) * (P[2] - P1[2])
        );

        mag_1_inv = 1 / mag_1;

        tau_1[0] = (P[0] - P1[0]) * mag_1_inv;
        tau_1[1] = (P[1] - P1[1]) * mag_1_inv;
        tau_1[2] = (P[2] - P1[2]) * mag_1_inv;

        //unit vector from P to P2
        mag_2 = sqrt(
            (P2[0] - P[0]) * (P2[0] - P[0]) +
            (P2[1] - P[1]) * (P2[1] - P[1]) +
            (P2[2] - P[2]) * (P2[2] - P[2])
        );

        mag_2_inv = 1 / mag_2;

        tau_2[0] = (P2[0] - P[0]) * mag_2_inv;
        tau_2[1] = (P2[1] - P[1]) * mag_2_inv;
        tau_2[2] = (P2[2] - P[2]) * mag_2_inv;

        float dot = tau_1[0] * tau_2[0] + tau_1[1] * tau_2[1] + tau_1[2] * tau_2[2];

        //angle alpha between the two unit vectors
        float alpha = M_PI - acos(dot);
        float cscAlpha = 1 / sin(alpha);
        float cscAlphaTurnradius = cscAlpha * turnRadius;

        //compute Center C of tangent circle using C = P + turnRadius * csc(alpha) * (tau_2 - tau_1)
        for (int j = 0; j < 3; j++) {
            C[j] = P[j] + cscAlphaTurnradius * (tau_2[j] - tau_1[j]);
        }

        //calculate distance between P and C
        float distance_PC = sqrt(
            (P[0] - C[0]) * (P[0] - C[0]) +
            (P[1] - C[1]) * (P[1] - C[1]) +
            (P[2] - C[2]) * (P[2] - C[2])
        );

        float c_path = distance_PC * cos(alpha / 2);

        if (distance_PC > min(mag_1, mag_2) ||
            c_path + prevCPath > mag_1) {
            //cannot smooth trajectory

            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = P[0];
            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = P[1];
            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = P[2];
            smoothedPathLength++;

            unsmoothedVertices++;
            prevCPath = c_path;
            continue;
        }

        //compute the number n of waypoints required to draw a circular arc using n = max(3, ceil(n_pi * (pi - alpha)/pi))
        int n_waypoints = max(3, static_cast<int>(ceil(n_pi * (M_PI - alpha) / M_PI)));


        float oneOverNWaypoints = (M_PI - alpha) / (n_waypoints - 1);

        for (int j = 0; j < n_waypoints; j++) {
            float omega = j * oneOverNWaypoints;
            float cosOmega = cos(omega);
            float cosAlphaOmega = cos(alpha + omega);

            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = C[0] - cscAlphaTurnradius * tau_1[0] * cosAlphaOmega -
                cscAlphaTurnradius * tau_2[0] * cosOmega;
            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = C[1] - cscAlphaTurnradius * tau_1[1] * cosAlphaOmega -
                cscAlphaTurnradius * tau_2[1] * cosOmega;
            paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = C[2] - cscAlphaTurnradius * tau_1[2] * cosAlphaOmega -
                cscAlphaTurnradius * tau_2[2] * cosOmega;
            smoothedPathLength++;

        }

        prevCPath = c_path;
    }

   paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = paths.rawPaths.elements[raw_startIndex + paths.rawPaths.n_waypoints * 3 - 3];
   paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = paths.rawPaths.elements[raw_startIndex + paths.rawPaths.n_waypoints * 3 - 2];
   paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = paths.rawPaths.elements[raw_startIndex + paths.rawPaths.n_waypoints * 3 - 1];
    smoothedPathLength++;


    paths.smoothedPaths.used_waypoints[path_index] = smoothedPathLength;


    //TODO N_wp = unsmoothedVertices / n;


    //return smoothedPath;
}

__global__ void smoothPaths_cuda(
    Paths_cuda paths,
    int max_elements,
    float turnRadius, int n_pi, size_t pitch) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if (idx < paths.rawPaths.n_paths) {

        smoothPath_cuda(paths, idx, idx * max_elements, idx * paths.rawPaths.n_waypoints * 3, turnRadius, n_pi);

        /*
                for(int i = 0; i < paths.rawPaths.n_waypoints * 3; i++) {
                    paths.smoothedPaths.elements[idx * paths.smoothedPaths.n_waypoints * 3 + i] = paths.rawPaths.elements[idx * paths.rawPaths.n_waypoints * 3 + i];
                    //   paths.smoothedPaths.elements[idx * paths.smoothedPaths.n_waypoints * 3 + i] = idx * paths.rawPaths.n_waypoints * 3 + i;


                    paths.smoothedPaths[index] =
                       smoothPath_cuda(
                               paths.rawPaths[index],
                               turnRadius, n_pi,
                               paths.N_wps[index]);
                }*/
    }

    //


    //  paths.smoothedPaths.elements[idx * paths.smoothedPaths.n_waypoints * 3 + 20] = paths.rawPaths.elements[9];


    // paths.smoothedPaths.elements[0] = paths.rawPaths.elements[paths.smoothedPaths.n_waypoints * 3 -9];

    /*


    for (int index = 0; index < paths.population; index++) {
        paths.smoothedPaths[index] =
                smoothPath_cuda(
                        paths.rawPaths[index],
                        turnRadius, n_pi,
                        paths.N_wps[index]);

    }

*/
}
