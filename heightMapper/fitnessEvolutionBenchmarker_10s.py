import subprocess
import json
from utils import copy_configs, update_json_file, write_array_to_file

copy_configs()
values = [1, 5, 10, 25, 50, 100, 250, 500, 1000, 1500, 2000, 3000, 5000]
update_json_file("config.json", "time_limit", 10)

averages = []

for i in values:
    update_json_file("config.json", "population_parallel", i)
    filename = "fitevo10-pop-" + str(i)

    for _ in range(5):
        subprocess.run(["../algorithm/algorithm", "-omp", "-o", filename], shell=False, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)

    data_filename = "../data/OMP_fitnesses-" + filename + ".dat"

    sums = [0.0] * 1
    count = 0

    with open(data_filename, 'r') as file:
        for line in file:
            # Split line into float numbers
            numbers = list(map(float, line.split()))
            # Add each number to the appropriate sum
            for j in range(1):
                sums[j] += numbers[j]
            count += 1

    average = [round(s / count, 5) for s in sums]
    average.insert(0, i)
    averages.append(average)

write_array_to_file("fitevo.dat", averages)
