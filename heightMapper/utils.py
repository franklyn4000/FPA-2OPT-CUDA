import json
import os
import shutil
import numpy as np

def copy_configs():
    shutil.copyfile("../config/init.json", "../config/init.json.copy")
    shutil.copyfile("../config/drone.json", "../config/drone.json.copy")
    shutil.copyfile("../config/config.json", "../config/config.json.copy")

def load_and_make_array(filename):
    data1 = np.loadtxt(filename, skiprows=1)

    x = data1[:, 0]
    y = data1[:, 1]
    z = data1[:, 2]

    z_dict = {(x, y): z for x, y, z in zip(x, y, z)}

    # Find unique X and Y values
    unique_x = np.unique(x)
    unique_y = np.unique(y)

    # Creating a 2D array
    z_2d_array = np.empty((len(unique_y), len(unique_x)))

    for i, x_value in enumerate(unique_x):
        for j, y_value in enumerate(unique_y):
            # fill the array with z values
            z_2d_array[j][i] = z_dict[(x_value, y_value)]

    return z_2d_array


def update_json_file(filename, key, new_value):
    copyfilename = '../config/' + filename + '.copy'

    with open(copyfilename, 'r') as json_file:
        data = json.load(json_file)

    data[key] = new_value

    with open(copyfilename, 'w') as json_file:
        json.dump(data, json_file, indent=4)


def write_array_to_file(filename, array):
    directory = "../plotdata"

    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(directory + '/' + filename, 'w') as f:
        for item in array:
            for it in item:
                f.write("%s " % it)
            f.write("\n")
