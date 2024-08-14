import subprocess
import json
from utils import copy_configs, update_json_file, write_array_to_file

copy_configs()
values = [4, 5, 6, 7, 8, 9, 10, 11, 12]
update_json_file("config.json", "iter_max", 200)

averages = []

for i in values:
    update_json_file("config.json", "path_length", i)
    filename = "2length-val-" + str(i)

    for _ in range(5):
        subprocess.run(["../algorithm/algorithm", "-omp", "-o", filename], shell=False, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

    data_filename = "../data/OMP_fitnesses-" + filename + ".dat"
    timings_filename = "../data/OMP_timings-" + filename + ".dat"

    sums = [0.0] * 1
    timings = [0.0] * 1
    count = 0

    with open(data_filename, 'r') as file:
        for line in file:
            # Split line into float numbers
            numbers = list(map(float, line.split()))
            # Add each number to the appropriate sum
            for j in range(1):
                sums[j] += numbers[j]
            count += 1

    with open(timings_filename, 'r') as file:
        for line in file:
            # Split line into float numbers
            numbers = list(map(float, line.split()))
            # Add each number to the appropriate sum

            for j in range(1):
                timings[j] += numbers[0]

    average = [round(s / count, 5) for s in sums]
    average_timing = [round(t / count, 5) for t in timings][0]

    average.insert(0, average_timing)
    average.insert(0, i)
    averages.append(average)

write_array_to_file("pathevo.dat", averages)
