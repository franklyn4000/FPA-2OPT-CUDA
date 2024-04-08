//
// Created by franklyn on 3/21/24.
//

#include "pollinator_parallel.h"
#include <random>
#include <math.h>
#include <cmath>
#include <iostream>
#include "omp.h"

float beta = 1.5;
float gam = 0.01;
static double sig_p = pow(std::tgamma(1.0 + beta) * sin(M_PI * beta / 2.0) /
                        (std::tgamma((1.0 + beta) / 2.0) * beta * pow(2.0, (beta - 1.0) / 2.0)), 1.0 / beta);

double normal_p(float sigma_squared) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> disU(0, sigma_squared);

    return disU(gen);
}

double sigma_p() {
    return pow(std::tgamma(1.0 + beta) * sin(M_PI * beta / 2.0) /
               (std::tgamma((1.0 + beta) / 2.0) * beta * pow(2.0, (beta - 1.0) / 2.0)), 1.0 / beta);
}

double *levy_p(int n) {
    std::random_device rd;
    std::mt19937 gen(rd());

    double *step = new double[n];
    double norm1, norm2;
    double oneOverBeta = 1.0 / beta;
    for (int i = 0; i < n; ++i) {
        std::normal_distribution<> disU(0, 1);
        norm1 = disU(gen);
        std::normal_distribution<> disV(0, 1);
        norm2 = disV(gen);
        step[i] = std::abs(0.2 * norm1 * sig_p / std::pow(std::abs(norm2), oneOverBeta));
    }
    return step;
}

void pollinate_parallel(
        Paths &paths,
        float p_switch) {

    std::random_device rd;
    std::mt19937 gen(rd());

//#pragma omp parallel for TODO local pollination race condition
    for (int pathIndex = 0; pathIndex < paths.rawPaths.size(); pathIndex++) {
        int n = paths.rawPaths[pathIndex].size() / 3;


        std::uniform_real_distribution<> dis(0.0, 1.0);
        // printf("%f\n", dis(gen));

        if (dis(gen) < p_switch) {
            //global pollinate

            // TODO Levy flight
            double *L = levy_p(n * 3);
            //  float gamma = 0.1;


            for (int i = 0; i < n - 1; i++) {
                for (int k = 0; k < 3; k++) {
                    float coord = paths.rawPaths[pathIndex][3 * i + k];
                    paths.rawPaths[pathIndex][3 * i + k] =
                            coord + L[3 * i] * (paths.fittestPath[3 * i + k] - coord);

                }
            }
        } else {
            //local pollinate
            std::uniform_real_distribution<> eps(0.0, 1.0);
            float epsilon = eps(gen);

            std::uniform_int_distribution<> rand(0, paths.population-1);
            int j = rand(gen);

            std::uniform_int_distribution<> rand2(0, paths.population-1);
            int l = rand2(gen);

            for (int i = 0; i < n - 1; i++) {
                for (int k = 0; k < 3; k++) {
                    paths.rawPaths[pathIndex][3 * i + k] =
                            paths.rawPaths[pathIndex][3 * i + k] + epsilon * (paths.rawPaths[j][3 * i + k] - paths.rawPaths[k][3 * i + k]);
                }
            }
        }
    }
}

