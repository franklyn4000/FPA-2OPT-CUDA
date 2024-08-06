import json
import os
import shutil

def update_json_file(filename, key, new_value):
    copyfilename = '../config/' + filename + '.copy'
    shutil.copyfile("../config/init.json", "../config/init.json.copy")
    shutil.copyfile("../config/drone.json", "../config/drone.json.copy")
    shutil.copyfile("../config/config.json", "../config/config.json.copy")

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
