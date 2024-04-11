//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_PATHSMOOTHER_CUH
#define ALGORITHM_PATHSMOOTHER_CUH

#include <vector>
#include "../objects/paths.h"

void smoothPaths(
        Paths &paths,
        float turnRadius, int n_pi);

std::vector<float> smoothPath(
        std::vector<float> &path,
        float turnRadius, int n_pi, float &N_wp);

#endif //ALGORITHM_PATHSMOOTHER_CUH
