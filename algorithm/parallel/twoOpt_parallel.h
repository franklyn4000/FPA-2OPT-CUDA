//
// Created by franklyn on 3/21/24.
//

#ifndef ALGORITHM_TWOOPT_PARALLEL_H
#define ALGORITHM_TWOOPT_PARALLEL_H

#include <vector>
#include "../objects/paths.h"

void twoOptParallel(
        Paths &paths,
        float turnRadius, int n_pi,
        const std::vector <std::vector<double>> &heightMap, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution, float w1, float w2);

#endif //ALGORITHM_TWOOPT_PARALLEL_H
