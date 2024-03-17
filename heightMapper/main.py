import numpy as np
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors
import os
import random


def generate_terrain(size, min_val=-500, max_val=600):
    raw_noise = np.random.uniform(-1, 1, size=(size, size))

    half = round(size/2)
    min = round(size/12)

    raw_noise[0:size, half-min:half+min] -= random.randint(1, 40) / 40
    raw_noise[half-min:half+min, half+min:size] -= random.randint(1, 13) / 13
    raw_noise[0:size, 0:min] += random.randint(1, 50) / 50
    terrain = gaussian_filter(raw_noise, sigma=size // 50)

    # normalize terrain data within given min_val and max_val
    min_terrain = np.min(terrain)
    max_terrain = np.max(terrain)
    norm_terrain = (terrain - min_terrain) / (max_terrain - min_terrain)  # normalize to 0-1 range
    terrain = norm_terrain * (max_val - min_val) + min_val  # scaling to desired range

    return terrain


def visualize_height_maps(height_map):
    cmap = mcolors.LinearSegmentedColormap.from_list("", ["red", "yellow", "green", "blue", "purple", "white"])

    fig = plt.figure(figsize=(14, 6))

    # 2D plot
    ax1 = fig.add_subplot(121)
    img = ax1.imshow(height_map, cmap=cmap, vmin=-1000, vmax=2000)
    fig.colorbar(img, ax=ax1, shrink=0.5, aspect=5, label='Height')

    # 3D plot
    ax2 = fig.add_subplot(122, projection='3d')
    x = np.arange(0, height_map.shape[1])
    y = np.arange(0, height_map.shape[0])
    X, Y = np.meshgrid(x, y)
    surf = ax2.plot_surface(X, Y, height_map, cmap=cmap, vmin=-1000, vmax=2000, linewidth=0)
    fig.colorbar(surf, ax=ax2, shrink=0.5, aspect=5, label='Height')

    plt.show()


def save_height_map(height_map, filename):
    np.save(filename + ".npy", height_map)
    np.savetxt(filename + ".csv", height_map, delimiter=',')


def load_height_map(filename):
    return np.load(filename)


filename = "height_map"

# if not os.path.exists(filename):
height_map = generate_terrain(150)
save_height_map(height_map, filename)
# else:
#    height_map = load_height_map(filename)

visualize_height_maps(height_map)
