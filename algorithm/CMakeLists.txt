cmake_minimum_required(VERSION 3.22)
project(algorithm CXX CUDA)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=${-fopenmp}>")

find_package(OpenMP REQUIRED)


add_executable(algorithm main.cu
        iolib.cu
        fpa.cu
        initialSolutionGenerator.cu
        pathSmoother_parallel.cpp
        fitnessComputer_seq.cu
        utils.cu
        pollinator.cu
        pollinator_parallel.cpp
        twoOpt_parallel.cpp
        fitnessComputer.cpp)


target_link_libraries(algorithm PRIVATE OpenMP::OpenMP_CXX)

set_target_properties(algorithm PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
