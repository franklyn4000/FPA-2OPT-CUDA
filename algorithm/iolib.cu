//
// Created by franklyn on 3/15/24.
//
#include "iolib.cuh"
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

std::vector<std::vector<double>> load_height_map(const std::string &filename) {
    std::vector<std::vector<double>> height_map;

    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string val;

        while (std::getline(ss, val, ',')) {
            row.push_back(std::stof(val));
        }

        height_map.push_back(row);
    }

    return height_map;
}