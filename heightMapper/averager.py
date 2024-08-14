import glob


def calculate_line_averages(files):
    averages = []
    for i in range(11):  # Assuming there are 10 lines in each file
        sum_list = [0, 0, 0, 0, 0, 0]
        for file in files:
            with open(file, 'r') as f:
                lines = f.readlines()
                numbers = list(map(float, lines[i].split()))
                sum_list = [x + y for x, y in zip(sum_list, numbers)]
        averages.append([x / len(files) for x in sum_list])
    return averages


def write_averages_to_file(averages, filename):
    with open(filename, 'w') as f:
        for average in averages:
            f.write(' '.join(map(str, average)) + '\n')


if __name__ == "__main__":
    files = glob.glob("OMP_timings-speedup-gracy-*.dat")  # specify the file type or a path to the directory
    averages = calculate_line_averages(files)
    write_averages_to_file(averages, "line_averages_gracy.txt")
