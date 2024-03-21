
#include "iolib.cuh"
#include <iostream>
#include <iomanip>  // for std::setprecision and std::fixed
#include "fpa.cuh"
#include "omp.h"

int main() {

    int iter_max = 100;

    int population = 10000;
    float p_switch = 0.8;
    float epsilon_init = 0.25;
    float epsilon_final = 0.02;
    int two_opt_freq = 10;

    std::string filename = "../heightMapper/height_map.csv";
    auto height_map = load_height_map(filename);

    double start_time = omp_get_wtime();

    std::vector<std::vector<float>> result = computeFPA(height_map, iter_max, population, p_switch, epsilon_init, epsilon_final, two_opt_freq);
    double time_taken = omp_get_wtime() - start_time;


    save_to_csv(result, "../heightMapper/paths.csv");


    std::cout << "\nTotal Execution time: " << time_taken << " seconds.\n";

    return 0;
}

