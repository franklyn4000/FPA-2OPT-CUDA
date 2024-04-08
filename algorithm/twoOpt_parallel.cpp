//
// Created by franklyn on 3/21/24.
//

#include "twoOpt_parallel.h"

void twoOptParallel(
        Paths &paths) {
/*

    for (int pathIndex = 0; pathIndex < paths.rawPaths.size(); pathIndex++) {
        std::vector<float> path = paths.rawPaths[pathIndex];
        int n = path.size() / 3;


        float fitness = paths.fitnesses[pathIndex];


        for (int i = 0; i < n - 1; i++) {
            for (int j = 1; j < n; j++) {
                std::vector<float> newPath(path);
                std::swap(newPath[i], newPath[j]);
                std::swap(newPath[i + 1], newPath[j + 1]);
                std::swap(newPath[i + 2], newPath[j + 2]);

              //TODO  std::vector<float> smoothedNewPath = smoot
                N_wp = 0;

                float newPathFitness = computeFitness(newPath,
                                                      heightMap,
                                                      N_wp,
                                                      max_asc_angle,
                                                      max_desc_angle,
                                                      a_utopia, f_utopia);


            }
        }


    }
*/
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