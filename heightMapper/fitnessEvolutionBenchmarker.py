import subprocess
from utils import update_json_file

values = [1, 10, 50, 100, 250, 500, 1000, 2000, 4000]
update_json_file("config.json", "time_limit", 10)

for i in values:
    update_json_file("config.json", "population", i)
    filename = "fitevo-pop-" + str(i)

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
            for i in range(1):
                sums[i] += numbers[i]
            count += 1

    averages = [round(s / count, 5) for s in sums]
    print(averages)