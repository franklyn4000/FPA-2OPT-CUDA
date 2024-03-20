//
// Created by franklyn on 3/20/24.
//

#include "fitnessComputer.cuh"
#include <iostream>

std::vector<float> computeFitnesses(
        std::vector <std::vector<float>> paths,
        const std::vector <std::vector<double>> &heightMap,
        std::vector<float> N_wps, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia) {

    std::vector<float> fitnesses;

    //penalty term

    //P = d_ug + d_dz + d_ea + (N_wp_unsmoothed / N_wp * l_traj)
    float w1 = 0.5;
    float w2 = 0.5;

    float f_cum = 0;


    float interval = 20.0f;

    for (const std::vector<float> &path: paths) {
        int n = path.size() / 3;

        for (int i = 0; i < n - 0; i++) {
            printf("\nP X: %f Y: %f Z: %f", path[3 * i], path[3 * i + 1], path[3 * i + 2]);
        }

    }

    int index = 0;
    for (const std::vector<float> &path: paths) {
        float d_ug = 0.0;
        float d_dz = 0.0;
        float d_ea = 0.0;
        float N_wp = 0.0;
        float l_traj = 0.0;


        float a_avg = 0;
        float f_avg = 0;

        int n = path.size() / 3;
        bool underground = false;
        bool undergroundLast = false;

        for (int i = 0; i < n - 1; i++) {
            std::vector<float> P1;
            std::vector<float> P2;
            P1.push_back(path[3 * i]);
            P1.push_back(path[3 * i + 1]);
            P1.push_back(path[3 * i + 2]);
            P2.push_back(path[3 * (i + 1)]);
            P2.push_back(path[3 * (i + 1) + 1]);
            P2.push_back(path[3 * (i + 1) + 2]);

            float distance_P1P2 = sqrt(
                    (P2[0] - P1[0]) * (P2[0] - P1[0]) +
                    (P2[1] - P1[1]) * (P2[1] - P1[1]) +
                    (P2[2] - P1[2]) * (P2[2] - P1[2])
            );

            int steps_P1P2 = static_cast<int>(std::floor(distance_P1P2 / interval));
            float step_length_P1P2 = steps_P1P2 > 0 ? distance_P1P2 / steps_P1P2 : distance_P1P2;


            printf("\ndistance: %f steps: %i length: %f \n", distance_P1P2, steps_P1P2, step_length_P1P2);

            float diff_x = P2[0] - P1[0];
            float diff_y = P2[1] - P1[1];
            float diff_z = P2[2] - P1[2];

            float interval_x = steps_P1P2 > 0 ? diff_x / steps_P1P2 : 0;
            float interval_y = steps_P1P2 > 0 ? diff_y / steps_P1P2 : 0;
            float interval_z = steps_P1P2 > 0 ? diff_z / steps_P1P2 : 0;


            double horizontal_length = std::sqrt(diff_x * diff_x + diff_y * diff_y);

            double angle_radians = std::atan2(diff_z, horizontal_length);


            printf("\nSUBPATH\n");
            for (int i = 1; i < steps_P1P2; i++) {
                printf("x: %f y: %f z:%f\n", P1[0] + interval_x * i, P1[1] + interval_y * i, P1[2] + interval_z * i);

                printf("distance since last: %f\n", step_length_P1P2);

                printf("DEBUG %f\n", P1[0] + interval_x * i);
                printf("DEBUG %f\n", P1[1] + interval_y * i);

                int pointX = static_cast<int>(std::round(P1[0] + interval_x * i));
                int pointY = static_cast<int>(std::round(P1[1] + interval_y * i));


                printf("X: %i, Y: %i \n", pointX, pointY);

                underground = heightMap[pointY][pointX] >= P1[2] + interval_z * i;
                if (underground && undergroundLast) {
                    d_ug += step_length_P1P2;
                } else if (underground != undergroundLast) {
                    d_ug += step_length_P1P2 / 2;
                }
                undergroundLast = underground;
                l_traj += step_length_P1P2;

            }
            printf("x: %f y: %f z:%f\n", P2[0], P2[1], P2[2]);
            printf("distance since last: %f\n", step_length_P1P2);

            int p1X = static_cast<int>(std::round(P1[0]));
            int p1Y = static_cast<int>(std::round(P1[1]));

            int p2X = static_cast<int>(std::round(P2[0]));
            int p2Y = static_cast<int>(std::round(P2[1]));

            int p1Z = static_cast<int>(std::round(P1[2]));
            int p2Z = static_cast<int>(std::round(P2[2]));

            underground = heightMap[p2Y][p2X] >= P2[2];
            if (underground && undergroundLast) {
                d_ug += step_length_P1P2;
            } else if (underground != undergroundLast) {
                d_ug += step_length_P1P2 / 2;
            }
            undergroundLast = underground;
            l_traj += step_length_P1P2;

            float height1 = p1Z - heightMap[p1Y][p1X];
            float height2 = p2Z - heightMap[p2Y][p2X];

            f_cum += (height1 + height2) / 2;

            printf("current angle %f°\n", angle_radians * (180.0 / M_PI));

            if (angle_radians > max_asc_angle || angle_radians < max_desc_angle) {
                d_ea += distance_P1P2;
            }


        }
        printf("total excessive flight path distance: %f\n", d_ea);
        printf("total underground  distance: %f\n", d_ug);

        N_wp = N_wps[index];
        //Penaly term P
        float P = d_ug + d_dz + d_ea + (N_wp * l_traj);

        f_avg = f_cum / n;

        //Cost term C
        float C = w1 * (l_traj / a_utopia) + w2 * (f_avg / f_utopia);

        //Fitness function F
        float F;
        if (P == 0) {
            F = 1 + 1 / (1 + C);
        } else {
            F = 0 + 1 / (1 + P);
        }


        fitnesses.push_back(F);
        index++;
    }


    return fitnesses;
}