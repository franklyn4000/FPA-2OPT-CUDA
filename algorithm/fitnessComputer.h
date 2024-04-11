//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_FITNESSCOMPUTER_CUH
#define ALGORITHM_FITNESSCOMPUTER_CUH

#include <vector>
#include "paths.h"

void computeFitnesses(
        Paths &paths,
        const std::vector <std::vector<double>> &heightMap, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution);

float computeFitness(std::vector<float> path,
                     const std::vector <std::vector<double>> &heightMap,
                     float N_wp, float max_asc_angle,
                     float max_desc_angle, float a_utopia, float f_utopia, float resolution);

void computeBestFitness(Paths &paths);

#endif //ALGORITHM_FITNESSCOMPUTER_CUH
