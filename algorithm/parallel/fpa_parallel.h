//
// Created by franklyn on 3/17/24.
//

#ifndef ALGORITHM_FPA_PARALLEL_H
#define ALGORITHM_FPA_PARALLEL_H

#include <vector>
#include "../objects/config.h"
#include "../objects/drone.h"
#include "../objects/initialConditions.h"

void computeFPA_parallel(
        Config &config, Drone &drone, InitialConditions &init);



#endif //ALGORITHM_FPA_PARALLEL_H
