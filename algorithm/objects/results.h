//
// Created by franklyn on 8/6/24.
//

#ifndef RESULTS_H
#define RESULTS_H

#include <vector>

class Results {
public:
    float setup_and_transfer_time;
    float pollination_time;
    float smoothing_time;
    float fitness_time;
    float twoopt_time;
    float total_time;
    float best_fitness;
    std::vector<float> fitnesses;
};


#endif //RESULTS_H
