import numpy as np
import json

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

data1 = load_and_make_array('heightmaps/switzerland3.xyz')
data2 = load_and_make_array('heightmaps/switzerland4.xyz')
data3 = load_and_make_array('heightmaps/switzerland5.xyz')
data4 = load_and_make_array('heightmaps/switzerland6.xyz')

top_row = np.concatenate((data1, data2), axis=1)
bottom_row = np.concatenate((data3, data4), axis=1)

# concatenate top_row and bottom_row along rows to get final 2000x2000 data
final_data = np.concatenate((bottom_row, top_row), axis=0)

np.savetxt("heightmaps/switzerland.csv", final_data, delimiter=",")


with open('heightmaps/switzerland.json', 'r') as source_file:
    data = json.load(source_file)

with open('init.json', 'w') as destination_file:
    json.dump(data, destination_file)
