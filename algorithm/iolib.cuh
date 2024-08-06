#ifndef IOLIB_H_INCLUDED
#define IOLIB_H_INCLUDED

#include <vector>
#include <string>
#include "json.hpp"

std::vector<std::vector<double>> load_height_map(const std::string &filename);
float* load_height_map_cuda(const std::string &filename, int cols, int rows);

void save_to_csv(const std::vector<std::vector<float>> &data, const std::string &file_name);
void save_to_csv(const std::vector<float> &data, const std::string &file_name);

void save_to_csv_cuda(float* data, int length, const std::string &file_name);

nlohmann::json readJsonFile(const std::string& filePath);

void appendLineToFile(const std::string& filename, const std::string& line);
void createEmptyFile(const std::string& filename);

#endif //IOLIB_H_INCLUDED