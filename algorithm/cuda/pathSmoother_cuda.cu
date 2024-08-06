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

    float n_pi_f = __int2float_rn(n_pi);
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

    float4 tau_1;
    float4 tau_2;

    float mag_1;
    float mag_2;

    float mag_1_inv;
    float mag_2_inv;

    for (int i = 1; i < n - 1; i++) {
        P1[0] = paths.rawPaths.elements[raw_startIndex + 3 * (i - 1)];
        P1[1] = paths.rawPaths.elements[raw_startIndex + 3 * (i - 1) + 1];
        P1[2] = paths.rawPaths.elements[raw_startIndex + 3 * (i - 1) + 2];

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

        tau_1.x = (P[0] - P1[0]) * mag_1_inv;
        tau_1.y = (P[1] - P1[1]) * mag_1_inv;
        tau_1.z = (P[2] - P1[2]) * mag_1_inv;

        //unit vector from P to P2
        mag_2 = sqrt(
                (P2[0] - P[0]) * (P2[0] - P[0]) +
                (P2[1] - P[1]) * (P2[1] - P[1]) +
                (P2[2] - P[2]) * (P2[2] - P[2])
        );

        mag_2_inv = 1 / mag_2;

        tau_2.x = (P2[0] - P[0]) * mag_2_inv;
        tau_2.y = (P2[1] - P[1]) * mag_2_inv;
        tau_2.z = (P2[2] - P[2]) * mag_2_inv;

        float dot = tau_1.x * tau_2.x + tau_1.y * tau_2.y + tau_1.z * tau_2.z;

        //angle alpha between the two unit vectors
        float alpha = M_PI - acos(dot);
        float cscAlpha = 1 / sin(alpha);
        float cscAlphaTurnradius = cscAlpha * turnRadius;

        //compute Center C of tangent circle using C = P + turnRadius * csc(alpha) * (tau_2 - tau_1)
        C[0] = P[0] + cscAlphaTurnradius * (tau_2.x - tau_1.x);
        C[1] = P[1] + cscAlphaTurnradius * (tau_2.y - tau_1.y);
        C[2] = P[2] + cscAlphaTurnradius * (tau_2.z - tau_1.z);

        tau_1.x *= cscAlphaTurnradius;
        tau_1.y *= cscAlphaTurnradius;
        tau_1.z *= cscAlphaTurnradius;
        tau_2.x *= cscAlphaTurnradius;
        tau_2.y *= cscAlphaTurnradius;
        tau_2.z *= cscAlphaTurnradius;

        //calculate distance between P and C
        float distance_PC = sqrt(
                (P[0] - C[0]) * (P[0] - C[0]) +
                (P[1] - C[1]) * (P[1] - C[1]) +
                (P[2] - C[2]) * (P[2] - C[2])
        );

        float c_path = distance_PC * cos(alpha * 0.5f);

        if (distance_PC > min(mag_1, mag_2) ||
            c_path + prevCPath > mag_1 ||
            mag_1 == 0.0 ||
            mag_2 == 0.0 ||
            alpha != alpha) {
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
        float n_waypoints = max(3.0f, n_pi_f * (M_PI - alpha) / M_PI) - 1.0f;
        float oneOverNWaypoints = (M_PI - alpha) / n_waypoints;

        for (float j = 0; j < n_waypoints; j++) {
            float omega = j * oneOverNWaypoints;
            float cosOmega = cos(omega);
            float cosAlphaOmega = cos(alpha + omega);

            int index = smooth_startIndex + smoothedPathLength * 3;

            paths.smoothedPaths.elements[index + 0] = C[0] - tau_1.x * cosAlphaOmega - tau_2.x * cosOmega;
            paths.smoothedPaths.elements[index + 1] = C[1] - tau_1.y * cosAlphaOmega - tau_2.y * cosOmega;
            paths.smoothedPaths.elements[index + 2] = C[2] - tau_1.z * cosAlphaOmega - tau_2.z * cosOmega;

            smoothedPathLength++;

        }

        prevCPath = c_path;

    }

    paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = paths.rawPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 3];
    paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = paths.rawPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 2];
    paths.smoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = paths.rawPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 1];
    smoothedPathLength++;

    paths.smoothedPaths.used_waypoints[path_index] = smoothedPathLength;
    paths.N_wps[path_index] = unsmoothedVertices / n;
}

__global__ void smoothPaths_cuda(
        Paths_cuda paths,
        int max_elements,
        float turnRadius, int n_pi) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    if (idx < paths.rawPaths.n_paths) {
        smoothPath_cuda(paths, idx, idx * max_elements, idx * paths.rawPaths.n_waypoints * 3, turnRadius, n_pi);
    }

}

