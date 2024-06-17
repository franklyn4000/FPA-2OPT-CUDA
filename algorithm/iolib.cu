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

float* load_height_map_cuda(const std::string &filename, int cols, int rows) {
    float* height_map = new float[cols * rows];;

    std::ifstream file(filename);
    std::string line;
    int rowIndex = 0;
    int colIndex = 0;

    while (std::getline(file, line)) {
        //float* row;
        std::stringstream ss(line);
        std::string val;

        while (std::getline(ss, val, ',')) {
            height_map[rowIndex * rows + colIndex] = std::stof(val);
            colIndex++;
            //row.push_back(std::stof(val));
        }
        colIndex = 0;
        //height_map.push_back(row);
        rowIndex++;
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

void save_to_csv_cuda(const float* data, int length, const std::string &file_name) {
    // Create an output filestream object
    std::ofstream out_file(file_name);

    // Check if the file was opened successfully
    if (!out_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }

    // Iterate through each vector in data
    for (int i = length*2; i < length*3; i++) {

        // Write each number to the file followed by a comma
        //for (const auto num : row) {
            out_file << data[i];
      //  }
        // Write a newline character at the end of each row
        out_file << '\n';
    }

    // Close the file
    out_file.close();
}