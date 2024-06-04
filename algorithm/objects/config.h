#ifndef CONFIG_H
#define CONFIG_H

#include <vector>

class Config {
public:
    int iter_max;
    int population;
    int two_opt_freq;
    int path_length;
    float resolution;
    float p_switch;
    float epsilon_init;
    float epsilon_final;
    std::vector<std::vector<double>> heightMap;
};

#endif //CONFIG_H