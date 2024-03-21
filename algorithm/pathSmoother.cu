//
// Created by franklyn on 3/20/24.
//

#include "pathSmoother.cuh"
#include <iostream>

std::vector <std::vector<float>> smoothPaths(
        std::vector <std::vector<float>> paths,
        float turnRadius, int n_pi, float* N_wps) {

    std::vector <std::vector<float>> smoothedPaths;
    float prevCPath = 0.0f;

    // std::vector<float> N_wps;
    int pathIndex = 0;
    for (const std::vector<float> &path: paths) {
        std::vector<float> smoothedPath;
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
            float mag_1 = sqrt(
                    (P[0] - P1[0]) * (P[0] - P1[0]) +
                    (P[1] - P1[1]) * (P[1] - P1[1]) +
                    (P[2] - P1[2]) * (P[2] - P1[2])
            );
            tau_1[0] = (P[0] - P1[0]) / mag_1;
            tau_1[1] = (P[1] - P1[1]) / mag_1;
            tau_1[2] = (P[2] - P1[2]) / mag_1;

            //unit vector from P to P2
            float mag_2 = sqrt(
                    (P2[0] - P[0]) * (P2[0] - P[0]) +
                    (P2[1] - P[1]) * (P2[1] - P[1]) +
                    (P2[2] - P[2]) * (P2[2] - P[2])
            );
            tau_2[0] = (P2[0] - P[0]) / mag_2;
            tau_2[1] = (P2[1] - P[1]) / mag_2;
            tau_2[2] = (P2[2] - P[2]) / mag_2;

            float dot = tau_1[0] * tau_2[0] + tau_1[1] * tau_2[1] + tau_1[2] * tau_2[2];

            //angle alpha between the two unit vectors
            float alpha = M_PI - acos(dot);
            float oneOverSinAlpha = (1 / sin(alpha));

            //compute Center C of tangent circle using C = P + turnRadius * csc(alpha) * (tau_2 - tau_1)
            for (int j = 0; j < 3; j++) {
                C[j] = P[j] + turnRadius * oneOverSinAlpha * (tau_2[j] - tau_1[j]);
            }

            //calculate distance between P and C
            float distance_PC = sqrt(
                    (P[0] - C[0]) * (P[0] - C[0]) +
                    (P[1] - C[1]) * (P[1] - C[1]) +
                    (P[2] - C[2]) * (P[2] - C[2])
            );

            float c_path = distance_PC * cos(alpha / 2);

            //printf("\n dist1: %f dist2: %f distC: %f cPath: %f\n", mag_1, mag_2, distance_PC, c_path + prevCPath);

            if (distance_PC > min(mag_1, mag_2) ||
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
            int n_waypoints = std::max(3, static_cast<int>(std::ceil(n_pi * (M_PI - alpha) / M_PI)));

            float cscAlpha = 1 / sin(alpha);

            float oneOverNWaypoints = 1 / (n_waypoints - 1);
            for (int j = 0; j < n_waypoints; j++) {
                //compute angle omega = j * (pi - alpha) / (n - 1)
                float omega = j * (M_PI - alpha) * oneOverNWaypoints;
                float cosOmega = cos(omega);
                float cosAlphaOmega = cos(alpha + omega);

                smoothedPath.push_back(C[0] - turnRadius * cscAlpha * tau_1[0] * cosAlphaOmega -
                                       turnRadius * cscAlpha * tau_2[0] * cosOmega);
                smoothedPath.push_back(C[1] - turnRadius * cscAlpha * tau_1[1] * cosAlphaOmega -
                                       turnRadius * cscAlpha * tau_2[1] * cosOmega);
                smoothedPath.push_back(C[2] - turnRadius * cscAlpha * tau_1[2] * cosAlphaOmega -
                                       turnRadius * cscAlpha * tau_2[2] * cosOmega);

            }

            prevCPath = c_path;
        }

        smoothedPath.push_back(path[3 * n - 3]);
        smoothedPath.push_back(path[3 * n - 2]);
        smoothedPath.push_back(path[3 * n - 1]);

        N_wps[pathIndex] = unsmoothedVertices / n;
        pathIndex++;
        smoothedPaths.push_back(smoothedPath);
    }

    // save_to_csv(N_wps, "../heightMapper/N_wps.csv");

    return smoothedPaths;
}

std::vector<float> smoothPathSingle(
        std::vector<float> path,
        float turnRadius, int n_pi, float* N_wps) {
    float prevCPath = 0.0f;

    // std::vector<float> N_wps;

    std::vector<float> smoothedPath;
    float unsmoothedVertices = 0;

    smoothedPath.push_back(path[0]);
    smoothedPath.push_back(path[1]);
    smoothedPath.push_back(path[2]);

    int n = path.size() / 3;

    for (int i = 1; i < n - 1; i++) {
        std::vector<float> P1;
        std::vector<float> P;
        std::vector<float> P2;
        P1.push_back(path[3 * (i - 1)]);
        P1.push_back(path[3 * (i - 1) + 1]);
        P1.push_back(path[3 * (i - 1) + 2]);
        P.push_back(path[3 * i]);
        P.push_back(path[3 * i + 1]);
        P.push_back(path[3 * i + 2]);
        P2.push_back(path[3 * (i + 1)]);
        P2.push_back(path[3 * (i + 1) + 1]);
        P2.push_back(path[3 * (i + 1) + 2]);



        //unit vector from P1 to P
        std::vector<float> tau_1;
        float mag_1 = sqrt(
                (P[0] - P1[0]) * (P[0] - P1[0]) +
                (P[1] - P1[1]) * (P[1] - P1[1]) +
                (P[2] - P1[2]) * (P[2] - P1[2])
        );
        tau_1.push_back((P[0] - P1[0]) / mag_1);
        tau_1.push_back((P[1] - P1[1]) / mag_1);
        tau_1.push_back((P[2] - P1[2]) / mag_1);

        //unit vector from P to P2
        std::vector<float> tau_2;
        float mag_2 = sqrt(
                (P2[0] - P[0]) * (P2[0] - P[0]) +
                (P2[1] - P[1]) * (P2[1] - P[1]) +
                (P2[2] - P[2]) * (P2[2] - P[2])
        );
        tau_2.push_back((P2[0] - P[0]) / mag_2);
        tau_2.push_back((P2[1] - P[1]) / mag_2);
        tau_2.push_back((P2[2] - P[2]) / mag_2);

        float dot = tau_1[0] * tau_2[0] + tau_1[1] * tau_2[1] + tau_1[2] * tau_2[2];

        //angle alpha between the two unit vectors
        float alpha = M_PI - acos(dot);

        //compute Center C of tangent circle using C = P + turnRadius * csc(alpha) * (tau_2 - tau_1)
        std::vector<float> C(3, 0);
        for (int j = 0; j < 3; j++) {


            C[j] = P[j] + turnRadius * (1 / sin(alpha)) * (tau_2[j] - tau_1[j]);
            //printf("\ncalculating C %f: %f + %f * %f * (%f - %f)  \n", C[j], P[j], turnRadius, (1 / sin(alpha)),
            //      tau_2[j], tau_1[j]);


        }


        //calculate distance between P and C
        float distance_PC = sqrt(
                (P[0] - C[0]) * (P[0] - C[0]) +
                (P[1] - C[1]) * (P[1] - C[1]) +
                (P[2] - C[2]) * (P[2] - C[2])
        );

        float c_path = distance_PC * cos(alpha / 2);

        //printf("\n dist1: %f dist2: %f distC: %f cPath: %f\n", mag_1, mag_2, distance_PC, c_path + prevCPath);

        if (distance_PC > min(mag_1, mag_2) ||
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
        int n_waypoints = std::max(3, static_cast<int>(std::ceil(n_pi * (M_PI - alpha) / M_PI)));

        float cscAlpha = 1 / sin(alpha);

        for (int j = 0; j < n_waypoints; j++) {
            //compute angle omega = j * (pi - alpha) / (n - 1)
            float omega = j * (M_PI - alpha) / (n_waypoints - 1);


            float cosOmega = cos(omega);
            float cosAlphaOmega = cos(alpha + omega);


            //compute point C_i = C - turnRadius * csc(alpha) * tau_1 * cos(alpha + omega) - turnRadius * csc(alpha) * tau_2 * cos(omega)

            smoothedPath.push_back(C[0] - turnRadius * cscAlpha * tau_1[0] * cosAlphaOmega -
                                   turnRadius * cscAlpha * tau_2[0] * cosOmega);
            smoothedPath.push_back(C[1] - turnRadius * cscAlpha * tau_1[1] * cosAlphaOmega -
                                   turnRadius * cscAlpha * tau_2[1] * cosOmega);
            smoothedPath.push_back(C[2] - turnRadius * cscAlpha * tau_1[2] * cosAlphaOmega -
                                   turnRadius * cscAlpha * tau_2[2] * cosOmega);

        }
/*
            smoothedPath.push_back(C[0]);
            smoothedPath.push_back(C[1]);
            smoothedPath.push_back(C[2]);
            smoothedPath.push_back(P[0]);
            smoothedPath.push_back(P[1]);
            smoothedPath.push_back(P[2]);

            smoothedPath.push_back(path[3 * i]);
            smoothedPath.push_back(path[3 * i + 1]);
            smoothedPath.push_back(path[3 * i + 2]);*/
        prevCPath = c_path;
    }

    smoothedPath.push_back(path[3 * n - 3]);
    smoothedPath.push_back(path[3 * n - 2]);
    smoothedPath.push_back(path[3 * n - 1]);



    // save_to_csv(N_wps, "../heightMapper/N_wps.csv");

    return smoothedPath;
}