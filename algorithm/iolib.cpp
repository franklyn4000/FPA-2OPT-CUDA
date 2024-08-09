//
// Created by franklyn on 3/15/24.
//
#include "iolib.h"
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <filesystem>

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
            height_map[colIndex * rows + rowIndex] = std::stof(val);
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

void save_to_csv_cuda(float* data, int length, const std::string &file_name) {
    // Create an output filestream object
    std::ofstream out_file(file_name);

    // Check if the file was opened successfully
    if (!out_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return;
    }

    // Iterate through each vector in data
    for (int i = 0; i < length; i++) {

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



// for convenience
using json = nlohmann::json;

json readJsonFile(const std::string& filePath) {
    std::ifstream i(filePath);
    json j;
    i >> j;
    return j;
}

bool file_exists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

void createEmptyFile(const std::string& filename) {
    if (file_exists(filename)) {
        return;
    } else {
        std::ofstream file(filename);
        if (!file) {
            std::cout << "Unable to create file.";
        }
    }
}

void createAndReplaceEmptyFile(const std::string& filename) {

    std::ofstream file(filename);
    if (!file) {
        std::cout << "Unable to create file.";
    }

}

void appendLineToFile(const std::string& filename, const std::string& line) {
    std::ofstream file;

    file.open(filename, std::ios_base::app); // Use 'app' for appending
    if (file.is_open())
    {
        file << line << "\n";
        file.close();
    }
    else
    {
        std::cout << "Unable to open file.";
    }
}

void createDirectory(const std::string& dirPath) {
    std::filesystem::path dir(dirPath);

    if (!std::filesystem::exists(dir)) {
        std::cout << "Directory does not exist, creating now" << std::endl;

        // Create the directory
        if (std::filesystem::create_directory(dir)) {
            std::cout << "Directory created successfully" << std::endl;
        } else {
            std::cout << "Failed to create directory" << std::endl;
        }
    }
}