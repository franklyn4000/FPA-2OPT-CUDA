//
// Created by franklyn on 3/20/24.
//

#include "pathSmoother_parallel.h"
#include <iostream>
#include "omp.h"
#include <math.h>
#include <cmath>

std::vector<float> smoothPath(
        std::vector<float> &path,
        float turnRadius, int n_pi, float &N_wp) {

    std::vector<float> smoothedPath;

    // std::vector<float> N_wps;
    float prevCPath = 0.0f;

    float unsmoothedVertices = 0;

    smoothedPath.push_back(path[0]);
    smoothedPath.push_back(path[1]);
    smoothedPath.push_back(path[2]);

    int n = path.size() / 3;

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
        P1[0] = path[3 * (i - 1)];
        P1[1] = path[3 * (i - 1) + 1];
        P1[2] = path[3 * (i - 1) + 2];

        P[0] = path[3 * i];
        P[1] = path[3 * i + 1];
        P[2] = path[3 * i + 2];

        P2[0] = path[3 * (i + 1)];
        P2[1] = path[3 * (i + 1) + 1];
        P2[2] = path[3 * (i + 1) + 2];

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

        if (distance_PC > std::min(mag_1, mag_2) ||
            c_path + prevCPath > mag_1) {
            //cannot smooth trajectory
            smoothedPath.push_back(P[0]);
            smoothedPath.push_back(P[1]);
            smoothedPath.push_back(P[2]);
            unsmoothedVertices++;
            prevCPath = c_path;
            continue;
        }

        //compute the number n of waypoints required to draw a circular arc using n = max(3, ceil(n_pi * (pi - alpha)/pi))
        int n_waypoints = std::max(3, static_cast<int>(ceil(n_pi * (M_PI - alpha) / M_PI)));


        float oneOverNWaypoints = (M_PI - alpha) / (n_waypoints - 1);

        for (int j = 0; j < n_waypoints; j++) {
            float omega = j * oneOverNWaypoints;
            float cosOmega = cos(omega);
            float cosAlphaOmega = cos(alpha + omega);

            smoothedPath.push_back(C[0] - cscAlphaTurnradius * tau_1[0] * cosAlphaOmega -
                                   cscAlphaTurnradius * tau_2[0] * cosOmega);
            smoothedPath.push_back(C[1] - cscAlphaTurnradius * tau_1[1] * cosAlphaOmega -
                                   cscAlphaTurnradius * tau_2[1] * cosOmega);
            smoothedPath.push_back(C[2] - cscAlphaTurnradius * tau_1[2] * cosAlphaOmega -
                                   cscAlphaTurnradius * tau_2[2] * cosOmega);

        }

        prevCPath = c_path;
    }

    smoothedPath.push_back(path[3 * n - 3]);
    smoothedPath.push_back(path[3 * n - 2]);
    smoothedPath.push_back(path[3 * n - 1]);

    N_wp = unsmoothedVertices / n;


    return smoothedPath;
}

void smoothPaths(
        Paths &paths,
        float turnRadius, int n_pi) {

    //paths.smoothedPaths.clear();

#pragma omp parallel for
    for (int index = 0; index < paths.population; index++) {


        paths.smoothedPaths[index] =
                smoothPath(
                        paths.rawPaths[index],
                        turnRadius, n_pi,
                        paths.N_wps[index]);

    }


}