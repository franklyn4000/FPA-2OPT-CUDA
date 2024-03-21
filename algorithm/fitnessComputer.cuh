//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_FITNESSCOMPUTER_CUH
#define ALGORITHM_FITNESSCOMPUTER_CUH

#include <vector>

float computeFitnesses(
        std::vector <std::vector<float>> paths,
        std::vector<float> *fittestPath,
        float bestFitness,
        std::vector <std::vector<float>> unsmoothedPaths,
        const std::vector <std::vector<double>> &heightMap,
        std::vector<float> N_wps, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia);

#endif //ALGORITHM_FITNESSCOMPUTER_CUH
