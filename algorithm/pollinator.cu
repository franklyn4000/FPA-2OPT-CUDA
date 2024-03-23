//
// Created by franklyn on 3/20/24.
//

#include "pollinator.cuh"
#include <random>
#include <math.h>
#include <cmath>
#include <iostream>
#include "omp.h"
/*
float beta = 1.5;
float gam = 0.01;
static double sig = pow(std::tgamma(1.0 + beta) * sin(M_PI * beta / 2.0) /
                        (std::tgamma((1.0 + beta) / 2.0) * beta * pow(2.0, (beta - 1.0) / 2.0)), 1.0 / beta);

double normal(float sigma_squared) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> disU(0, sigma_squared);

    return disU(gen);
}

double sigma() {
    return pow(std::tgamma(1.0 + beta) * sin(M_PI * beta / 2.0) /
               (std::tgamma((1.0 + beta) / 2.0) * beta * pow(2.0, (beta - 1.0) / 2.0)), 1.0 / beta);
}

double *levy(int n) {
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
        step[i] = std::abs(0.2 * norm1 * sig / std::pow(std::abs(norm2), oneOverBeta));
    }
    return step;
}

void pollinate(
        std::vector <std::vector<float>> &paths,
        std::vector<float> fittestPath,
        float p_switch) {

    std::random_device rd;
    std::mt19937 gen(rd());

#pragma omp parallel for
    for (int pathIndex = 0; pathIndex < paths.size(); pathIndex++) {
        int n = paths[pathIndex].size() / 3;


        std::uniform_real_distribution<> dis(0.0, 1.0);
        // printf("%f\n", dis(gen));

        if (dis(gen) < p_switch) {
            //global pollinate

            // TODO Levy flight
            double *L = levy(n * 3);
            //  float gamma = 0.1;


            for (int i = 0; i < n - 1; i++) {
                for (int k = 0; k < 3; k++) {
                    float coord = paths[pathIndex][3 * i + k];
                    paths[pathIndex][3 * i + k] =
                            coord + L[3 * i] * (fittestPath[3 * i + k] - coord);

                }
            }
        } else {
            //local pollinate
            std::uniform_real_distribution<> eps(0.0, 1.0);
            float epsilon = eps(gen);

            std::uniform_int_distribution<> rand(0, n);
            int j = rand(gen);

            std::uniform_int_distribution<> rand2(0, n);
            int l = rand2(gen);

            for (int i = 0; i < n - 1; i++) {
                for (int k = 0; k < 3; k++) {
                    paths[pathIndex][3 * i + k] =
                            paths[pathIndex][3 * i + k] + epsilon * (paths[j][3 * i + k] - paths[k][3 * i + k]);
                }
            }
        }
    }
}

*/