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


struct array2D malloc_2d(int m, int n) {
/*
    float** p = (float **) malloc(m * sizeof(double **) + m * n * sizeof(double *));


    if (p == NULL) {
        perror("p was null");
        exit(-1);
    }

    for (int i = 0; i < m; i++) {
        p[i] = (double **) p + m + i * n;
    }

    double *a = (double *) malloc(m * n * k * sizeof(double));
    if (a == NULL) {
        free(p);
        perror("a was null");
    }

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            p[i][j] = a + (i * n * k) + (j * k);
        }
    }

    if (p == NULL) {
        perror("allocation failed");
        exit(-1);
    }
    struct array3D ret = {.array = p, .data = a};
    return ret;*/
}

struct array2D d_malloc_2d(int m, int n) {
/*
    double ***p =
            omp_target_alloc(m * sizeof(double **) + m * n * sizeof(double *),
                             omp_get_default_device());
    if (p == NULL) {
        perror("p was null");
        exit(-1);
    }*/
}