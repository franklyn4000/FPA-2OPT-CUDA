//
// Created by franklyn on 3/20/24.
//

#include "utils.cuh"

float calculateFUtopia(float x1, float y1, float z1, float xn, float yn, float zn) {
    std::vector<float> start;
    std::vector<float> end;
    start.push_back(x1);
    start.push_back(y1);
    start.push_back(z1);

    end.push_back(xn);
    end.push_back(yn);
    end.push_back(zn);

    float distance_P1P2 = sqrt(
            (end[0] - start[0]) * (end[0] - start[0]) +
            (end[1] - start[1]) * (end[1] - start[1]) +
            (end[2] - start[2]) * (end[2] - start[2])
    );

    return distance_P1P2;
}