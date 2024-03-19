//
// Created by franklyn on 3/15/24.
//
#include "iolib.cuh"
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

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

void save_to_csv(const std::vector<std::vector<float>> &data, const std::string &file_name) {
    // Create an output filestream object
    std::ofstream out_file(file_name);

    // Check if the file was opened successfully
    if (!out_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }

    // Iterate through each vector in data
    for (const auto &row : data) {
        // Write each number to the file followed by a comma
        for (const auto num : row) {
            out_file << num << ',';
        }
        // Write a newline character at the end of each row
        out_file << '\n';
    }

    // Close the file
    out_file.close();
}

void save_to_csv(const std::vector<float> &data, const std::string &file_name) {
    // Create an output filestream object
    std::ofstream out_file(file_name);

    // Check if the file was opened successfully
    if (!out_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }

    // Write each number to the file followed by a newline
    for (const auto num : data) {
        out_file << num << '\n';
    }

    // Close the file
    out_file.close();
}