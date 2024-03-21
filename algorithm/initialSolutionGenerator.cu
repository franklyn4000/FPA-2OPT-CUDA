//
// Created by franklyn on 3/20/24.
//

#include "initialSolutionGenerator.cuh"
#include <random>

std::vector <std::vector<float>> generateSolutions(
        float x1, float y1, float z1,
        float xn, float yn, float zn,
        float x_min, float x_max, float y_min,
        float y_max, float z_min, float z_max,
        int length, int population) {
    // Generate a random size for the vector
    std::random_device rd;

    std::vector <std::vector<float>> solutions;

    for (int i = 0; i < population; i++) {
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(length, length); // path length

        std::uniform_real_distribution<> distrX(x_min, x_max);
        std::uniform_real_distribution<> distrY(y_min, y_max);
        std::uniform_real_distribution<> distrZ(z_min, z_max);

        int n = distr(gen);

        std::vector<float> vector(3 * n, 0);

        // Set initial values
        vector[0] = x1;
        vector[1] = y1;
        vector[2] = z1;

       // printf("\nVECTOR\n");
        for (int i = 1; i < n - 1; ++i) {
            vector[3 * i] = distrX(gen);
            vector[3 * i + 1] = distrY(gen);
            vector[3 * i + 2] = distrZ(gen);
            //printf("test.push_back(%f);\n", vector[3 * i]);
            //printf("test.push_back(%f);\n", vector[3 * i + 1]);
            //printf("test.push_back(%f);\n", vector[3 * i + 2]);
        }

        // Set final values
        vector[3 * n - 3] = xn;
        vector[3 * n - 2] = yn;
        vector[3 * n - 1] = zn;


        solutions.push_back(vector);
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