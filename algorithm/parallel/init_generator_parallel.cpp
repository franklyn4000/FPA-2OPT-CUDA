//
// Created by franklyn on 3/20/24.
//

#include "init_generator_parallel.h"
#include <random>

std::vector <std::vector<float>> generateSolutions(
        InitialConditions &init,
        int length, int population) {
    // Generate a random size for the vector
    std::random_device rd;

    std::vector <std::vector<float>> solutions;

    for (int i = 0; i < population; i++) {
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(length, length); // path length

        std::uniform_real_distribution<> distrX(init.x_min, init.x_max);
        std::uniform_real_distribution<> distrY(init.y_min, init.y_max);
        std::uniform_real_distribution<> distrZ(init.z_min, init.z_max);

        int n = distr(gen);

        std::vector<float> vector(3 * n, 0);

        // Set initial values
        vector[0] = init.x1;
        vector[1] = init.y1;
        vector[2] = init.z1;

        // printf("\nVECTOR\n");
        for (int j = 1; j < n - 1; ++j) {
            vector[3 * j] = distrX(gen);
            vector[3 * j + 1] = distrY(gen);
            vector[3 * j + 2] = distrZ(gen);
            //printf("test.push_back(%f);\n", vector[3 * i]);
            //printf("test.push_back(%f);\n", vector[3 * i + 1]);
            //printf("test.push_back(%f);\n", vector[3 * i + 2]);
        }

        // Set final values
        vector[3 * n - 3] = init.xn;
        vector[3 * n - 2] = init.yn;
        vector[3 * n - 1] = init.zn;

        solutions.push_back(vector);
        //solutions[i] = vector;
    }

    return solutions;
}

float** generateSolutions_cuda_old(
        InitialConditions &init,
        const int length, int population) {
    // Generate a random size for the vector
    std::random_device rd;

    float** solutions = new float*[population];

    for (int i = 0; i < population; i++) {
        std::mt19937 gen(rd());

        std::uniform_real_distribution<> distrX(init.x_min, init.x_max);
        std::uniform_real_distribution<> distrY(init.y_min, init.y_max);
        std::uniform_real_distribution<> distrZ(init.z_min, init.z_max);

        const int n = length;

        float* vector = new float[3 * n];

        // Set initial values
        vector[0] = init.x1;
        vector[1] = init.y1;
        vector[2] = init.z1;

        for (int j = 1; j < n - 1; ++j) {
            vector[3 * j] = distrX(gen);
            vector[3 * j + 1] = distrY(gen);
            vector[3 * j + 2] = distrZ(gen);
        }

        // Set final values
        vector[3 * n - 3] = init.xn;
        vector[3 * n - 2] = init.yn;
        vector[3 * n - 1] = init.zn;

        solutions[i] = vector;
        //solutions[i] = vector;
    }

    return solutions;
}

std::vector <std::vector<float>> generateTestSolutions() {
    std::vector <std::vector<float>> testSolutions;
    std::vector<float> test;

    test.push_back(50);
    test.push_back(100);
    test.push_back(550);

    test.push_back(10);
    test.push_back(126);
    test.push_back(552);

    test.push_back(10);
    test.push_back(56);
    test.push_back(552);

    testSolutions.push_back(test);

    return testSolutions;
}