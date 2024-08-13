import numpy as np
import json

from utils import copy_configs, load_and_make_array


def parse_switzerland1():
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

    with open('../config/init.json', 'w') as destination_file:
        json.dump(data, destination_file)


def parse_switzerland2():
    data1 = load_and_make_array('heightmaps/s1.xyz')
    data2 = load_and_make_array('heightmaps/s2.xyz')
    data3 = load_and_make_array('heightmaps/s3.xyz')
    data4 = load_and_make_array('heightmaps/s4.xyz')
    data5 = load_and_make_array('heightmaps/s5.xyz')
    data6 = load_and_make_array('heightmaps/s6.xyz')
    data7 = load_and_make_array('heightmaps/s7.xyz')
    data8 = load_and_make_array('heightmaps/s8.xyz')
    data9 = load_and_make_array('heightmaps/s9.xyz')
    data10 = load_and_make_array('heightmaps/s10.xyz')
    data11 = load_and_make_array('heightmaps/s11.xyz')
    data12 = load_and_make_array('heightmaps/s12.xyz')
    data13 = load_and_make_array('heightmaps/s13.xyz')
    data14 = load_and_make_array('heightmaps/s14.xyz')
    data15 = load_and_make_array('heightmaps/s15.xyz')
    data16 = load_and_make_array('heightmaps/s16.xyz')

    top_row = np.concatenate((data1, data3, data5, data7, data9, data11, data13, data15), axis=1)
    bottom_row = np.concatenate((data2, data4, data6, data8, data10, data12, data14, data16), axis=1)

    # concatenate top_row and bottom_row along rows to get final 2000x2000 data
    final_data = np.concatenate((top_row, bottom_row), axis=0)

    np.savetxt("heightmaps/switzerland2.csv", final_data, delimiter=",")

    with open('heightmaps/switzerland2.json', 'r') as source_file:
        data = json.load(source_file)

    with open('../config/init.json', 'w') as destination_file:
        json.dump(data, destination_file)


parse_switzerland1()
copy_configs()
