//
// Created by franklyn on 3/20/24.
//

#include "pollinator.cuh"
#include <random>
#include <math.h>
#include <cmath>
#include <iostream>

float beta = 1.5;
float gam = 0.01;

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
    double *step = new double[n];
    for (int i = 0; i < n; ++i) {
        step[i] = std::abs (0.2 * normal(1) * sigma() / pow(abs(normal(1)), 1.0 / beta) );
    }
    return step;
}

float drawLevy2() {


    float lambda = 1.5;
    float s = 1;

    float L1 = (lambda * std::tgamma(lambda) + std::sin(M_PI * lambda / 2)) / M_PI;
    float L2 = 1 / (std::pow(s, 1 + lambda));

    return L1 * L2;

}

float drawLevy() {
    float beta = 1.5;


    float one = std::tgamma(1 + beta);
    float two = beta * std::tgamma((1 + beta) / 2);
    float three = std::sin((M_PI * beta) / 2);
    float four = std::pow(2, (beta - 1) / 2);

    float sigma_squared = std::pow((one / two) * (three / four), 1 / beta);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> disU(0, sigma_squared);
    std::normal_distribution<> disV(0, 1);


    float U = disU(gen);
    float V = disV(gen);

    float s = U / (std::pow(std::abs(V), 1 / beta));

    //  printf("U: %f, V: %f, s: %f\n", U, V, s);

    float g = std::tgamma(beta + 1);
    float si = std::sin(M_PI * beta / 2);

    float left = (g * si) / M_PI;


    float right = 1 / std::pow(std::abs(s), 1 + beta);

    //TODO ceil??
    float L = left * right;

    return L;
}

void pollinate(
        std::vector <std::vector<float>> &paths,
        std::vector<float> fittestPath,
        float p_switch) {

    std::random_device rd;
    std::mt19937 gen(rd());


    for (int pathIndex = 0; pathIndex < paths.size(); pathIndex++) {
        int n = paths[pathIndex].size() / 3;


        std::uniform_real_distribution<> dis(0.0, 1.0);
        // printf("%f\n", dis(gen));

        if (dis(gen) < p_switch) {
            //global pollinate

            // TODO Levy flight
            double *L = levy(n*3);
          //  float gamma = 0.1;


            for (int i = 0; i < n - 1; i++) {
                for (int k = 0; k < 3; k++) {

                    //printf("%f   %f\n", L[i] * paths[pathIndex][3 * i + k],
                    //       gamma * L[i] * (fittestPath[3 * i + k] - paths[pathIndex][3 * i + k]));
                  //   printf("%f \n",  L[3*i]);
                    paths[pathIndex][3 * i + k] =
                            paths[pathIndex][3 * i + k] +
                            L[3*i] * (fittestPath[3 * i + k] - paths[pathIndex][3 * i + k]);

                }
            }
        } else {
            //local pollinate

            std::uniform_real_distribution<> eps(0.0, 1.0);

            float epsilon = eps(gen);

            std::uniform_int_distribution<> rand(0, 10);

            int j = rand(gen);

            std::uniform_int_distribution<> rand2(0, 10);
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

