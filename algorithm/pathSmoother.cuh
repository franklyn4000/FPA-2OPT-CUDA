//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_PATHSMOOTHER_CUH
#define ALGORITHM_PATHSMOOTHER_CUH

#include <vector>

std::vector <std::vector<float>> smoothPaths(
        std::vector <std::vector<float>> paths,
        float turnRadius, int n_pi, std::vector<float> *N_wps);

std::vector <float> smoothPathSingle(
        std::vector<float> path,
        float turnRadius, int n_pi, std::vector<float> *N_wps);

#endif //ALGORITHM_PATHSMOOTHER_CUH
