//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_INITIALSOLUTIONGENERATOR_CUH
#define ALGORITHM_INITIALSOLUTIONGENERATOR_CUH


#include <vector>

std::vector <std::vector<float>> generateSolutions(
        float x1, float y1, float z1,
        float xn, float yn, float zn,
        float x_min, float x_max, float y_min,
        float y_max, float z_min, float z_max,
        int length, int population);

std::vector <std::vector<float>> generateTestSolutions();

#endif //ALGORITHM_INITIALSOLUTIONGENERATOR_CUH
