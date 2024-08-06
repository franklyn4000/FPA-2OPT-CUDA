#ifndef CONFIG_H
#define CONFIG_H

#include <vector>

class Config {
public:
    float time_limit;
    int iter_max;
    int population;
    int two_opt_freq;
    int path_length;
    int n_pi;
    float resolution;
    float w1;
    float w2;
    float p_switch;
    float epsilon_init;
    float epsilon_final;
    std::vector<std::vector<double>> heightMap;
    float* heightMap_cuda;
    int heightMap_cols;
    int heightMap_rows;
};

#endif //CONFIG_H