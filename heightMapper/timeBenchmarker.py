import subprocess
import json
from utils import copy_configs

def update_json_file(filename, key, new_value):
    # Read the existing file into a dictionary
    with open(filename, 'r') as json_file:
        data = json.load(json_file)

    # Update the dictionary
    data[key] = new_value

    # Write the updated dictionary back to the file
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)


values1 = list(range(1, 10))  # Single steps from 1 to 9
values2 = list(range(10, 51, 5))  # Five steps from 10 to 50
values3 = list(range(70, 401, 20))  # Twenty steps from 70 to 400

iterations = values1 + values2

copy_configs()

for i in iterations:
    update_json_file("../config/config.json", "iter_max", i)
    filename = "iter-" + str(i)


    for _ in range(5):
        subprocess.run(["../algorithm/algorithm", "-omp", "-o", filename], shell=False, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

    data_filename = "../data/OMP_timings-" + filename + ".dat"
    
    sums = [0.0] * 6
    count = 0

    with open(data_filename, 'r') as file:
        for line in file:
            # Split line into float numbers
            numbers = list(map(float, line.split()))
            # Add each number to the appropriate sum
            for i in range(6):
                sums[i] += numbers[i]
            count += 1

    averages = [round(s / count, 5) for s in sums]
    print(averages)
