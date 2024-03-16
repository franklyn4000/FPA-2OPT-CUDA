
#include "iolib.h"
#include <iostream>
#include <iomanip>  // for std::setprecision and std::fixed
#include "omp.h"

int main() {
    double start_time = omp_get_wtime();

    std::string filename = "../heightMapper/height_map.csv";
    auto height_map = load_height_map(filename);

    double time_taken = omp_get_wtime() - start_time;
/*
    // Print the height map
    for(const auto &row : height_map) {
        for(const auto &val : row) {
            std::cout << std::setprecision(16) << std::fixed << val << " ";
        }
        std::cout << '\n';
    }
*/
    std::cout << "Execution time: " << time_taken << " seconds.\n";

    return 0;
}