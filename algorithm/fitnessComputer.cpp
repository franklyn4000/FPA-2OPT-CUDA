//
// Created by franklyn on 3/20/24.
//

#include "fitnessComputer.h"
#include <iostream>
#include "omp.h"
#include <math.h>

float computeFitness(std::vector<float> path,
                     const std::vector <std::vector<double>> &heightMap,
                     float N_wp, float max_asc_angle,
                     float max_desc_angle, float a_utopia, float f_utopia) {

    float interval = 1 / 30.0f;
    float w1 = 0.4;
    float w2 = 0.6;

    float d_ug = 0.0;
    float d_dz = 0.0;
    float d_ea = 0.0;
    float l_traj = 0.0;

    float a_cum = 0;
    float a_avg = 0;
    float f_avg = 0;

    int n = path.size() / 3;
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
        P1[0] = path[3 * i];
        P1[1] = path[3 * i + 1];
        P1[2] = path[3 * i + 2];
        P2[0] = path[3 * (i + 1)];
        P2[1] = path[3 * (i + 1) + 1];
        P2[2] = path[3 * (i + 1) + 2];

        float diff_x = P2[0] - P1[0];
        float diff_y = P2[1] - P1[1];
        float diff_z = P2[2] - P1[2];

        float distance_P1P2 = std::sqrt(
                diff_x * diff_x +
                diff_y * diff_y +
                diff_z * diff_z
        );

        steps_P1P2 = std::floor(distance_P1P2 * interval);

        inv_steps = 1 / steps_P1P2;

        if (steps_P1P2 > 0) {
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

        float horizontal_length = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        float angle_radians = std::atan2(diff_z, horizontal_length);

        for (int i = 1; i < steps_P1P2; i++) {

            int pointX = static_cast<int>(std::round(P1[0] + interval_x * i));
            int pointY = static_cast<int>(std::round(P1[1] + interval_y * i));

            if (
                    pointY > heightMap.size() - 1 ||
                    pointY < 0 ||
                    pointX > heightMap[0].size() - 1 ||
                    pointX < 0
                    ) {
                underground = true;
            } else {
                underground = heightMap[pointY][pointX] >= P1[2] + interval_z * i;
            }

            if (underground && undergroundLast) {
                d_ug += step_length_P1P2;
            } else if (underground != undergroundLast) {
                d_ug += step_length_P1P2 / 2;
            }
            undergroundLast = underground;
            l_traj += step_length_P1P2;

        }

        int p2X = static_cast<int>(std::round(P2[0]));
        int p2Y = static_cast<int>(std::round(P2[1]));
        int p2Z = static_cast<int>(std::round(P2[2]));

        float height2;
        if (
                p2Y > heightMap.size() - 1 ||
                p2Y < 0 ||
                p2X > heightMap[0].size() - 1 ||
                p2X < 0
                ) {
            height2 = 99999;
            underground = true;
        } else {
            height2 = p2Z - heightMap[p2Y][p2X];
        }


        underground = height2 < 0;
        if (underground && undergroundLast) {
            d_ug += step_length_P1P2;
        } else if (underground != undergroundLast) {
            d_ug += step_length_P1P2 / 2;
        }
        undergroundLast = underground;
        l_traj += step_length_P1P2;

        a_cum += height2;

        if (angle_radians > max_asc_angle || angle_radians < max_desc_angle) {
            d_ea += distance_P1P2;
        }

    }


    //Penaly term P
    float P = d_ug + d_dz + d_ea + (N_wp * l_traj);

    a_avg = a_cum / n;
    // printf("a_avg: %f, a_utopia: %f, l_traj: %f, f_utopia: %f \n", a_avg, a_utopia, l_traj, f_utopia);


    //Fitness function F
    if (P == 0) {
        //Cost term C
        float C = w1 * (a_avg / a_utopia) + w2 * (l_traj / f_utopia);

        return 1 + 1 / (1 + C);
    }
    return 0 + 1 / (1 + P);

}

void computeFitnesses(
        Paths &paths,
        const std::vector <std::vector<double>> &heightMap, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia) {


    for (int index = 0; index < paths.population; index++) {

        float F =
                computeFitness(paths.smoothedPaths[index],
                               heightMap,
                               paths.N_wps[index],
                               max_asc_angle,
                               max_desc_angle,
                               a_utopia, f_utopia);

        //printf("fitn %f\n", F);

        if (F > paths.bestFitness) {
            paths.fittestPath = paths.rawPaths[index];
            paths.bestFitness = F;
        }

    }

    //printf("avg fitness: %f %f %i", cumulative_fitness / paths.size(), cumulative_fitness, paths.size());
    // printf("best fitness: %f\n", bestFitness);

}


