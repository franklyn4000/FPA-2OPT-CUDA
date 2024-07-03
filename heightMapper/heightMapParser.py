import numpy as np

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

# Load the four height maps, remember to replace 'filename1.txt', ... with your file names.
#data1 = np.loadtxt('switzerland3.xyz', skiprows=1)
#data2 = np.loadtxt('switzerland4.xyz', skiprows=1)
#data3 = np.loadtxt('switzerland5.xyz', skiprows=1)
#data4 = np.loadtxt('switzerland6.xyz', skiprows=1)


data1 = load_and_make_array('switzerland3.xyz')
data2 = load_and_make_array('switzerland4.xyz')
data3 = load_and_make_array('switzerland5.xyz')
data4 = load_and_make_array('switzerland6.xyz')

top_row = np.concatenate((data1, data2), axis=1)
bottom_row = np.concatenate((data3, data4), axis=1)

# concatenate top_row and bottom_row along rows to get final 2000x2000 data
final_data = np.concatenate((bottom_row, top_row), axis=0)

np.savetxt("switzerland.csv", final_data, delimiter=",")


