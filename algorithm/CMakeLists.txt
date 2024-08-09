cmake_minimum_required(VERSION 3.22)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
project(algorithm CXX CUDA)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=${-fopenmp}>")

find_package(OpenMP REQUIRED)


add_executable(algorithm main.cu
        iolib.cpp
        parallel/fpa_parallel.cpp
        parallel/init_generator_parallel.cpp
        parallel/pathSmoother_parallel.cpp
        utils.cu
        parallel/pollinator_parallel.cpp
        parallel/twoOpt_parallel.cpp
        parallel/fitnessComputer_parallel.cpp
        objects/paths.cpp
        objects/drone.cpp
        objects/config.cpp
        objects/initialConditions.cpp
        cuda/fpa_cuda.cu
        cuda/pathSmoother_cuda.cu
        cuda/init_generator_cuda.cu
        cuda/init_generator_cuda.cuh
        cuda/fitnessComputer_cuda.cu
        cuda/fitnessComputer_cuda.cuh
        cuda/pollinator_cuda.cu
        cuda/pollinator_cuda.cuh
        cuda/twoOpt_cuda.cu
        objects/results.cpp
        objects/results.h)


target_link_libraries(algorithm PRIVATE OpenMP::OpenMP_CXX)

set_target_properties(algorithm PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
