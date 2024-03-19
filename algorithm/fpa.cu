//
// Created by franklyn on 3/17/24.
//

#include "fpa.cuh"
#include <vector>
#include <random>
#include <math.h>
#include "iolib.cuh"

std::vector<float>
generateVector(float x1, float y1, float z1, float xn, float yn, float zn, float x_min, float x_max, float y_min,
               float y_max, float z_min, float z_max) {
    // Generate a random size for the vector
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(5, 5); // replace 10 with maximum possible value of n

    std::uniform_real_distribution<> distrX(x_min, x_max);
    std::uniform_real_distribution<> distrY(y_min, y_max);
    std::uniform_real_distribution<> distrZ(z_min, z_max);

    int n = distr(gen);

    std::vector<float> vector(3 * n, 0);

    // Set initial values
    vector[0] = x1;
    vector[1] = y1;
    vector[2] = z1;

    for (int i = 1; i < n - 1; ++i) {
        vector[3 * i] = distrX(gen);
        vector[3 * i + 1] = distrY(gen);
        vector[3 * i + 2] = distrZ(gen);
    }

    // Set final values
    vector[3 * n - 3] = xn;
    vector[3 * n - 2] = yn;
    vector[3 * n - 1] = zn;

    return vector;
}

std::vector <std::vector<float>> smoothPaths(std::vector <std::vector<float>> paths, float turnRadius, int n_pi) {
    std::vector <std::vector<float>> smoothedPaths;

    for (const std::vector<float> &path: paths) {
        std::vector<float> smoothedPath;

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
                printf("\ncalculating C %f: %f + %f * %f * (%f - %f)  \n", C[j], P[j], turnRadius, (1 / sin(alpha)),
                       tau_2[j], tau_1[j]);
            }


            //calculate distance between P and C
            float distance_PC = sqrt(
                    (P[0] - C[0]) * (P[0] - C[0]) +
                    (P[1] - C[1]) * (P[1] - C[1]) +
                    (P[2] - C[2]) * (P[2] - C[2])
            );

            printf("\n dist1: %f dist2: %f distC: %f \n", mag_1, mag_2, distance_PC);
            if (distance_PC > min(mag_1, mag_2)) {
                //cannot smooth trajectory
                smoothedPath.push_back(P[0]);
                smoothedPath.push_back(P[1]);
                smoothedPath.push_back(P[2]);
                continue;
            }

            //compute the number n of waypoints required to draw a circular arc using n = max(3, ceil(n_pi * (pi - alpha)/pi))
            int n_waypoints = std::max(3, static_cast<int>(std::ceil(n_pi * (M_PI - alpha) / M_PI)));


            for (int j = 0; j < n_waypoints; j++) {
                //compute angle omega = j * (pi - alpha) / (n - 1)
                float omega = j * (M_PI - alpha) / (n_waypoints - 1);


                for (int k = 0; k < 3; k++) {
                    /*
                    smoothedPath.push_back(
                            C[k] -
                            turnRadius *
                            (1 / sin(alpha)) *
                            tau_1[k] *
                            cos(alpha + omega) -
                            turnRadius *
                            (1 / sin(alpha)) *
                            tau_2[k] *
                            cos(omega)
                    );
                     */
                }

                //compute point C_i = C - turnRadius * csc(alpha) * tau_1 * cos(alpha + omega) - turnRadius * csc(alpha) * tau_2 * cos(omega)
                // VERIFY

                smoothedPath.push_back(C[0] - turnRadius * (1 / sin(alpha)) * tau_1[0] * cos(alpha + omega) -
                                       turnRadius * (1 / sin(alpha)) * tau_2[0] * cos(omega));
                smoothedPath.push_back(C[1] - turnRadius * (1 / sin(alpha)) * tau_1[1] * cos(alpha + omega) -
                                       turnRadius * (1 / sin(alpha)) * tau_2[1] * cos(omega));
                smoothedPath.push_back(C[2] - turnRadius * (1 / sin(alpha)) * tau_1[2] * cos(alpha + omega) -
                                       turnRadius * (1 / sin(alpha)) * tau_2[2] * cos(omega));

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
        }

        smoothedPath.push_back(path[3 * n - 3]);
        smoothedPath.push_back(path[3 * n - 2]);
        smoothedPath.push_back(path[3 * n - 1]);


        smoothedPaths.push_back(smoothedPath);
    }

    return smoothedPaths;
}

std::vector<float> computeFitnesses(std::vector <std::vector<float>> paths, const std::vector <std::vector<double>> &heightMap) {
    std::vector<float> fitnesses;

    //penalty term

    //P = d_ug + d_dz + d_ea + (N_wp_unsmoothed / N_wp * l_traj)



    float interval = 20.0f;

    for (const std::vector<float> &path: paths) {
        int n = path.size() / 3;

        for (int i = 0; i < n - 0; i++) {
            printf("\nP X: %f Y: %f Z: %f", path[3 * i], path[3 * i + 1], path[3 * i + 2]);
        }

    }


    for (const std::vector<float> &path: paths) {
        float d_ug = 0.0;
        float d_dz = 0.0;
        float d_ea = 0.0;
        float N_wp_unsmoothed = 0.0;
        float N_wp = 50.0;
        float l_traj = 0.0;

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

            printf("\nSUBPATH\n");
            for (int i = 1; i < steps_P1P2; i++) {
                printf("x: %f y: %f z:%f\n", P1[0] + interval_x * i, P1[1] + interval_y * i, P1[2] + interval_z * i);

                printf("distance since last: %f\n", step_length_P1P2);



                int pointX = static_cast<int>(std::round(P1[0] + interval_x * i));
                int pointY = static_cast<int>(std::round(P1[1] + interval_y * i));


                printf("X: %i, Y: %i \n", pointX, pointY);

                underground = heightMap[pointY][pointX] >= P1[2] + interval_z * i;
                if(underground && undergroundLast) {
                    d_ug += step_length_P1P2;
                } else if (underground != undergroundLast) {
                    d_ug += step_length_P1P2 / 2;
                }
                undergroundLast = underground;



            }
            printf("x: %f y: %f z:%f\n", P2[0], P2[1], P2[2]);
            printf("distance since last: %f\n", step_length_P1P2);

            int p2X = static_cast<int>(std::round(P2[0]));
            int p2Y = static_cast<int>(std::round(P2[1]));

            underground = heightMap[p2Y][p2X] >= P2[2];
            if(underground && undergroundLast) {
                d_ug += step_length_P1P2;
            } else if (underground != undergroundLast) {
                d_ug += step_length_P1P2 / 2;
            }
            undergroundLast = underground;

        }
        printf("total underground  distance: %f\n", d_ug);

        //Penaly term P
        float P = d_ug + d_dz + d_ea + (N_wp_unsmoothed / N_wp * l_traj);

        //Fitness function F
        float F;
        if(P == 0) {
            F = 1 + 1 / (1+ 0/*C*/);
        } else {
            F = 0 + 1 / (1 + P);
        }



        fitnesses.push_back(F);

    }


    return fitnesses;
}

std::vector <std::vector<float>> computeFPA(
        const std::vector <std::vector<double>> &heightMap,
        int iter_max,
        int population,
        float p_switch,
        float epsilon_init,
        float epsilon_final,
        int two_opt_freq) {

    size_t x_mid = heightMap.size() / 2;
    size_t y_mid = (heightMap.empty()) ? 0 : heightMap[0].size() / 2;

    float x_min = 0.0f;
    float x_max = heightMap.size();
    float y_min = 0.0f;
    float y_max = heightMap[0].size();
    float z_min = -500.0f;
    float z_max = 500.0f;
    float x1 = 5.0f;
    float y1 = (float) y_mid;
    float z1 = heightMap[y1][x1] + 10;
    float xn = heightMap.size() - 5.0f;
    float yn = (float) y_mid;
    float zn = heightMap[yn][xn] + 10;



    //Objective min or max f(x), x = (x1, x2, ..., xd)
    //Initialize a population of n flowers/pollen gametes with random solutions
    std::vector <std::vector<float>> outputVector;
    for (int i = 0; i < population; i++) {
        outputVector.push_back(generateVector(x1, y1, z1, xn, yn, zn, x_min, x_max, y_min, y_max, z_min, z_max));
    }

    std::vector <std::vector<float>> smoothedPaths;
    smoothedPaths = smoothPaths(outputVector, 12.0f, 52);

    std::vector <std::vector<float>> testPaths;

    std::vector<float> test;

    test.push_back(10);
    test.push_back(10);
    test.push_back(1000);

    test.push_back(10);
    test.push_back(10);
    test.push_back(-1000);

    testPaths.push_back(test);

    std::vector<float> test3;

    test3.push_back(10);
    test3.push_back(140);
    test3.push_back(1000);

    test3.push_back(10);
    test3.push_back(140);
    test3.push_back(-1000);

    testPaths.push_back(test3);

    std::vector<float> test1;

    test1.push_back(140);
    test1.push_back(10);
    test1.push_back(1000);

    test1.push_back(140);
    test1.push_back(10);
    test1.push_back(-1000);

    testPaths.push_back(test1);

    std::vector<float> test2;

    test2.push_back(140);
    test2.push_back(140);
    test2.push_back(1000);

    test2.push_back(140);
    test2.push_back(140);
    test2.push_back(-1000);

    testPaths.push_back(test2);


    std::vector<float> fitnesses;
    fitnesses = computeFitnesses(smoothedPaths, heightMap);
    for (float fitness: fitnesses) {
        printf("\nfitness: %f", fitness);
    }
    printf("\n");

    save_to_csv(fitnesses, "../heightMapper/fitnesses.csv");


    //Find the best solution g∗ in the initial population

    //while (t <MaxGeneration)
    //   for i = 1 : n (all n flowers in the population)
    //if rand < p,
    //           Draw a (d-dimensional) step vector L which obeys a L´evy distribution
    //Global pollination via x

    //  else
    //  Draw ǫ from a uniform distribution in [0,1]
    //  Randomly choose j and k among all the solutions
    // Do local pollination via x

    //   end if
    //      Evaluate new solutions
    //  If new solutions are better, update them in the population
    //  end for
    //  Find the current best solution g∗
    //  end while

    return smoothedPaths;
}

