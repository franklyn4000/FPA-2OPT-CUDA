//
// Created by franklyn on 3/20/24.
//

#include "utils.cuh"

float calculateFUtopia(InitialConditions &init) {
    std::vector<float> start;
    std::vector<float> end;
    start.push_back(init.x1);
    start.push_back(init.y1);
    start.push_back(init.z1);

    end.push_back(init.xn);
    end.push_back(init.yn);
    end.push_back(init.zn);

    float distance_P1P2 = sqrt(
            (end[0] - start[0]) * (end[0] - start[0]) +
            (end[1] - start[1]) * (end[1] - start[1]) +
            (end[2] - start[2]) * (end[2] - start[2])
    );

    return distance_P1P2;
}