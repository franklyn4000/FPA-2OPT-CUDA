import json
import os


def update_json_file(filename, key, new_value):
    # Read the existing file into a dictionary
    with open(filename, 'r') as json_file:
        data = json.load(json_file)

    # Update the dictionary
    data[key] = new_value

    # Write the updated dictionary back to the file
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)


def write_array_to_file(filename, array):
    directory = "../plotdata"

    # create the directory if it does not exist
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(directory + '/' + filename, 'w') as f:
        for item in array:
            for it in item:
                f.write("%s " % it)
            f.write("\n")
