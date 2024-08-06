//
// Created by franklyn on 3/21/24.
//

#include "twoOpt_parallel.h"
#include "fitnessComputer_parallel.h"
#include "pathSmoother_parallel.h"
#include <iostream>
#include <omp.h>

void twoOptParallel(
        Paths &paths,
        float turnRadius, int n_pi,
        const std::vector <std::vector<double>> &heightMap, float max_asc_angle,
        float max_desc_angle, float a_utopia, float f_utopia, float resolution, float w1, float w2) {


#pragma omp parallel for
    for (int pathIndex = 0; pathIndex < paths.rawPaths.size(); pathIndex++) {
        std::vector<float> path = paths.rawPaths[pathIndex];
        int n = path.size() / 3;
        float fitness = paths.fitnesses[pathIndex];
        float N_wp = 0;

        bool improved = true;

        int iter = 0;
        // printf("current fitness: %f ", fitness);
        start_again:
        std::vector<float> newPath(path);

        float newPathFitness = 0;


        for (int i = 1; i < n - 2; i++) {
            for (int j = i + 1; j < n - 1; j++) {

                std::swap(newPath[3 * i], newPath[3 * j]);
                std::swap(newPath[3 * i + 1], newPath[3 * j + 1]);
                std::swap(newPath[3 * i + 2], newPath[3 * j + 2]);

                float oldNwp = N_wp;

                std::vector<float> smoothedNewPath =
                        smoothPath(
                                newPath,
                                turnRadius, n_pi,
                                N_wp);

                if(N_wp > oldNwp) {
                    continue;
                }


                newPathFitness = computeFitness(smoothedNewPath,
                                                heightMap,
                                                N_wp,
                                                max_asc_angle,
                                                max_desc_angle,
                                                a_utopia, f_utopia, resolution, w1, w2);


                // printf("iter: %i - current fitness: %f new fitness: %f\n", iter, fitness, newPathFitness);
                if (newPathFitness > fitness) {
                    path = newPath;
                    fitness = newPathFitness;
                    iter++;
                    goto start_again;
                }
            }
        }

        //   printf(" new fitness: %f \n", fitness);
        paths.rawPaths[pathIndex] = path;
        paths.fitnesses[pathIndex] = fitness;

    }

/*
    for (int pathIndex = 0; pathIndex < paths.size(); pathIndex++)
          std::vector<float> path = paths[pathIndex];
           int n = path.size() / 3;

           float fitness = 0.0f;// TODO get fitness of path

           for(int i = 0; i < n - 1; i++) {
               for(int j = 1; j < n; j++) {


                   //std::vector<float> temp_path = twoOpt_swap(path, i, j);
                   float temp_fitness = compute_fitness(temp_path);

                   if (temp_fitness > fitness) {
                       path = temp_path;
                       fitness = temp_fitness;
                       // TODO calc this path again
                   }


               }
           }


       }
*/
}