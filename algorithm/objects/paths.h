#ifndef PATHS_H
#define PATHS_H

#include <vector>

class Paths {
public:
    std::vector<std::vector<float>> rawPaths;
    std::vector<std::vector<float>> pollinatedPaths;
    std::vector<std::vector<float>> smoothedPaths;
    std::vector<float> N_wps;
    std::vector<float> fitnesses;

    int fittestPathIndex;
    float bestFitness;
    std::vector<float> fittestPath;

    int population;
    Paths(int pop);
};

typedef struct {
    int n_waypoints;
    int n_paths;
    int* used_waypoints;
    float* elements;
} PathMatrix;

struct Paths_cuda {
    PathMatrix rawPaths;
    PathMatrix pollinatedPaths;
    PathMatrix smoothedPaths;
    PathMatrix tempPaths;
    PathMatrix tempSmoothedPaths;

    float* N_wps;
    float* fitnesses;

    int fittestPathIndex;
    float* fittestPath;
    float* bestFitness;
    int fittestPathThread;

    int* twoOptCountFinishedSolutions;
    int* twoOptFinishedSolutions;
    int* twoOptCurrentI;
    int* twoOptCurrentJ;
};



#endif //PATHS_H