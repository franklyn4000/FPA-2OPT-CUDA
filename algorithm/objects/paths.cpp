//
// Created by franklyn on 3/27/24.
//

#include "paths.h"
#include <vector>

Paths::Paths(int pop) {
    population = pop;

    rawPaths = std::vector<std::vector<float>>(pop);
    pollinatedPaths = std::vector<std::vector<float>>(pop);
    smoothedPaths = std::vector<std::vector<float>>(pop);
    N_wps = std::vector<float>(pop, 0.0f);
    fitnesses = std::vector<float>(pop, 0.0f);
    bestFitness = -1.0f;
    fittestPathIndex = 0;
}