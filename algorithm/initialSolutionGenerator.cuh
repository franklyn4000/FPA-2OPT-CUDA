//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_INITIALSOLUTIONGENERATOR_CUH
#define ALGORITHM_INITIALSOLUTIONGENERATOR_CUH


#include <vector>
#include "objects/initialConditions.h"

std::vector <std::vector<float>> generateSolutions(
        InitialConditions &init,
        int length, int population);

float** generateSolutions_cuda_old(
        InitialConditions &init,
        const int length, int population);

std::vector <std::vector<float>> generateTestSolutions();

#endif //ALGORITHM_INITIALSOLUTIONGENERATOR_CUH
