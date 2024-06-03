//
// Created by franklyn on 3/20/24.
//

#ifndef ALGORITHM_UTILS_CUH
#define ALGORITHM_UTILS_CUH

#include <vector>
#include "objects/initialConditions.h"

float calculateFUtopia(InitialConditions &init);

struct array2D {
    double **array;
    double *data;
};



struct array2D malloc_2d(int m, int n);
struct array2D d_malloc_2d(int m, int n);

#endif //ALGORITHM_UTILS_CUH
