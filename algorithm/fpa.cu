//
// Created by franklyn on 3/17/24.
//

#include "fpa.cuh"
#include <vector>
#include <random>

std::vector<float> generateVector(float x1, float y1, float z1, float xn, float yn, float zn ,float x_min, float x_max) {
    // Generate a random size for the vector
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(2, 10); // replace 10 with maximum possible value of n

   // std::uniform_real_distribution<> distrX(x_min, x_max);

    int n = distr(gen);

    std::vector<float> vector(3 * n, 0);

    // Set initial values
    vector[0] = x1;
    vector[1] = y1;
    vector[2] = z1;

    for (int i = 1; i < n-1; ++i) {
        vector[3 * i] = 5 + i * 10;
        vector[3 * i + 1] = 6 + i * 10;
        vector[3 * i + 2] = 7 + i * 10;
    }

    // Set final values
    vector[3 * n - 3] = xn;
    vector[3 * n - 2] = yn;
    vector[3 * n - 1] = zn;

    return vector;
}

std::vector <std::vector<float>> computeFPA(
        const std::vector <std::vector<double>> &heightMap,
        int iter_max,
        int population,
        float p_switch,
        float epsilon_init,
        float epsilon_final,
        int two_opt_freq) {

    float x_min = 0.0f;
    float x_max = 1.0f;
    float x1 = 0.1f;
    float y1 = 0.2f;
    float z1 = 0.3f;
    float xn = 0.4f;
    float yn = 0.5f;
    float zn = 0.6f;

    std::vector<std::vector<float>> outputVector;
    outputVector.push_back(generateVector(x1, y1, z1, xn, yn, zn, x_min, x_max));

    //Objective min or max f(x), x = (x1, x2, ..., xd)
    //Initialize a population of n flowers/pollen gametes with random solutions
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

    return outputVector;
}

