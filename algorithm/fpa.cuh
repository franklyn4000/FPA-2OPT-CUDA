//
// Created by franklyn on 3/17/24.
//

#ifndef ALGORITHM_FPA_CUH
#define ALGORITHM_FPA_CUH

#include <vector>

std::vector <std::vector<float>> computeFPA(
        const std::vector <std::vector<double>> &heightMap,
        int iter_max,
        int population,
        float p_switch,
        float epsilon_init,
        float epsilon_final,
        int two_opt_freq);



#endif //ALGORITHM_FPA_CUH
