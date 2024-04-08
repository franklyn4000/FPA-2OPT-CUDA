import numpy as np
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors
import os
import random
import csv


def read_paths_from_csv(filename):
    paths = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            path = []
            for i in range(0, len(row), 3):  # group by three (x, y, z)
                if i + 2 < len(row):  # to avoid index out of bound error
                    path.append((row[i], row[i + 1], row[i + 2]))
            paths.append(path)
    return paths


def read_fitnesses_from_csv(filename):
    fitnesses = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row:  # check if row is not empty
                fitnesses.append(float(row[0]))  # convert string to float
    return fitnesses


def read_fittest_from_file(file_path):
    """Read the fittest path from a CSV file and return it as a list of points."""

    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Create a generator for every three lines
    path_points = zip(*(iter(lines),) * 3)

    # Create path list where every point is a tuple (X, Y, Z)
    path = [(float(x.strip()), float(y.strip()), float(z.strip())) for x, y, z in path_points]

    return path

def generate_terrain(size, min_val=-500, max_val=600):
    raw_noise = np.random.uniform(-1, 1, size=(size, size))

    half = round(size / 2)
    min = round(size / 12)

    raw_noise[0:size, half - min:half + min] -= random.randint(1, 40) / 40
    raw_noise[half - min:half + min, half + min:size] -= random.randint(1, 13) / 13
    raw_noise[0:size, 0:min] += random.randint(1, 50) / 50
    terrain = gaussian_filter(raw_noise, sigma=size // 50)

    # normalize terrain data within given min_val and max_val
    min_terrain = np.min(terrain)
    max_terrain = np.max(terrain)
    norm_terrain = (terrain - min_terrain) / (max_terrain - min_terrain)  # normalize to 0-1 range
    terrain = norm_terrain * (max_val - min_val) + min_val  # scaling to desired range

    return terrain


def visualize_height_maps(height_map, fittest_path, fittest_path2, fittest_path3, fittest_path4):
    cmap = mcolors.LinearSegmentedColormap.from_list("", ["red", "yellow", "green", "blue", "purple", "white"])

    fittest_path_x = [float(point[0]) for point in fittest_path]
    fittest_path_y = [float(point[1]) for point in fittest_path]
    fittest_path_z = [float(point[2]) for point in fittest_path]

    fittest_path2_x = [float(point[0]) for point in fittest_path2]
    fittest_path2_y = [float(point[1]) for point in fittest_path2]
    fittest_path2_z = [float(point[2]) for point in fittest_path2]

    fittest_path3_x = [float(point[0]) for point in fittest_path3]
    fittest_path3_y = [float(point[1]) for point in fittest_path3]
    fittest_path3_z = [float(point[2]) for point in fittest_path3]

    fittest_path4_x = [float(point[0]) for point in fittest_path4]
    fittest_path4_y = [float(point[1]) for point in fittest_path4]
    fittest_path4_z = [float(point[2]) for point in fittest_path4]



    # Create subplots
    fig, ax = plt.subplots(nrows=2, ncols=2, figsize=(16, 12))
    ax1, ax2, ax3, ax4 = ax.flatten()

    #min_fitness_idx = fitnesses.index(max(fitnesses))  # get the index of path with lowest fitness

    ax1.plot(fittest_path_x, fittest_path_y, color='#ff3300', linewidth=1)
    ax2.plot(fittest_path_x, fittest_path_z, color='#ff3300', linewidth=1)
    ax3.plot(fittest_path_y, fittest_path_z, color='#ff3300', linewidth=1)

    ax1.plot(fittest_path2_x, fittest_path2_y, color='#ff8800', linewidth=1)
    ax2.plot(fittest_path2_x, fittest_path2_z, color='#ff8800', linewidth=1)
    ax3.plot(fittest_path2_y, fittest_path2_z, color='#ff8800', linewidth=1)

    ax1.plot(fittest_path3_x, fittest_path3_y, color='#ffaa00', linewidth=1)
    ax2.plot(fittest_path3_x, fittest_path3_z, color='#ffaa00', linewidth=1)
    ax3.plot(fittest_path3_y, fittest_path3_z, color='#ffaa00', linewidth=1)

    ax1.plot(fittest_path4_x, fittest_path4_y, color='#ffff00', linewidth=1)
    ax2.plot(fittest_path4_x, fittest_path4_z, color='#ffff00', linewidth=1)
    ax3.plot(fittest_path4_y, fittest_path4_z, color='#ffff00', linewidth=1)

    # 2D Plot from top
    img = ax1.imshow(height_map, cmap=cmap, vmin=-1000, vmax=2000)
    fig.colorbar(img, ax=ax1, shrink=0.5, aspect=5, label='Height')

    # 2D Plot from the side (Y-axis)
    ax2.plot(range(len(height_map)), height_map[len(height_map) // 2, :],  color='black',  linewidth=3)

    ax2.set_title('Side View (Y-axis)')

    # 2D Plot from the side (X-axis)
    ax3.plot(range(len(height_map)), height_map[:, len(height_map[0]) // 2],  color='black',  linewidth=3)

    ax3.set_title('Side View (X-axis)')

    # 3D plot
    x = np.arange(0, height_map.shape[1])
    y = np.arange(0, height_map.shape[0])
    X, Y = np.meshgrid(x, y)
    ax4 = fig.add_subplot(224, projection='3d')
    surf = ax4.plot_surface(X, Y, height_map, cmap=cmap, vmin=-1000, vmax=2000, linewidth=0)
    fig.colorbar(surf, ax=ax4, shrink=0.5, aspect=5, label='Height')
   # for idx, path in enumerate(paths):
    #    path_x = [float(point[0]) for point in path]
     #   path_y = [float(point[1]) for point in path]
      #  path_z = [float(point[2]) for point in path]
       # ax4.invert_yaxis()
        #if idx == min_fitness_idx:
         #   ax4.plot(path_x, path_y, path_z, color='blue')  # highlight path with lowest fitness in blue color
          #  ax4.scatter(path_x, path_y, path_z, color='blue', s=20)
       # else:
            #ax4.plot(path_x, path_y, path_z, color='red')
            #ax4.scatter(path_x, path_y, path_z, color='red', s=20)

    plt.show()


def save_height_map(height_map, filename):
    np.save(filename + ".npy", height_map)
    np.savetxt(filename + ".csv", height_map, delimiter=',')


def load_height_map(filename):
    return np.load(filename + ".npy")


filename = "height_map"
paths_csv_filename = 'paths.csv'
fitnesses_csv_filename = 'fitnesses.csv'
if not os.path.exists(filename + ".npy"):
    height_map = generate_terrain(1000)
    save_height_map(height_map, filename)
else:
    height_map = load_height_map(filename)
#paths = read_paths_from_csv(paths_csv_filename)
#fitnesses = read_fitnesses_from_csv(fitnesses_csv_filename)
fittest_path_file = read_fittest_from_file("fittest1.csv")
fittest_path_file2 = read_fittest_from_file("fittest2.csv")
fittest_path_file3 = read_fittest_from_file("fittest3.csv")
fittest_path_file4 = read_fittest_from_file("fittest4.csv")
visualize_height_maps(height_map, fittest_path_file, fittest_path_file2, fittest_path_file3, fittest_path_file4)
