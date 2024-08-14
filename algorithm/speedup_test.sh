#!/bin/bash

# Program to run


# Different values for OMP_NUM_THREADS
THREAD_VALUES=(1 2 4 6 14)

for threads in "${THREAD_VALUES[@]}"; 
do
  PROGRAM="./algorithm -omp -t -o speedup"
  echo "Running with OMP_NUM_THREADS=$threads"
  export OMP_NUM_THREADS=$threads
  $PROGRAM
done
