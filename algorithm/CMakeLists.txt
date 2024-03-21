cmake_minimum_required(VERSION 3.22)
project(algorithm CXX CUDA)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ggdb")

find_package(OpenMP REQUIRED)


add_executable(algorithm main.cu
        iolib.cu
        fpa.cu
        initialSolutionGenerator.cu
        pathSmoother.cu
        fitnessComputer.cu
        utils.cu
        pollinator.cu)


target_link_libraries(algorithm PRIVATE OpenMP::OpenMP_CXX)

set_target_properties(algorithm PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
