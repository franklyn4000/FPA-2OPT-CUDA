#include "init_generator_cuda.cuh"
#include <random>

float* generateSolutions_cuda(
    InitialConditions& init,
    int n_waypoints, int population) {
    auto* hostPaths = new float[population * n_waypoints * 3];
    std::random_device rd;

    for (int i = 0; i < population; i++) {
        std::mt19937 gen(rd());

        std::uniform_real_distribution<> distrX(init.x_min, init.x_max);
        std::uniform_real_distribution<> distrY(init.y_min, init.y_max);
        std::uniform_real_distribution<> distrZ(init.z_min, init.z_max);


        hostPaths[i * n_waypoints * 3 + 0] = init.x1;
        hostPaths[i * n_waypoints * 3 + 1] = init.y1;
        hostPaths[i * n_waypoints * 3 + 2] = init.z1;

        for (int j = 1; j < n_waypoints - 1; ++j) {
            hostPaths[i * n_waypoints * 3 + (3 * j)] = distrX(gen);
            hostPaths[i * n_waypoints * 3 + (3 * j + 1)] = distrY(gen);
            hostPaths[i * n_waypoints * 3 + (3 * j + 2)] = distrZ(gen);
        }

        hostPaths[(i + 1) * n_waypoints - 3] = init.xn;
        hostPaths[(i + 1) * n_waypoints - 2] = init.yn;
        hostPaths[(i + 1) * n_waypoints - 1] = init.zn;
    }

    return hostPaths;
}
