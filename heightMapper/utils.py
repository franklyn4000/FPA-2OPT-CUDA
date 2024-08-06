def update_json_file(filename, key, new_value):
    # Read the existing file into a dictionary
    with open(filename, 'r') as json_file:
        data = json.load(json_file)

    # Update the dictionary
    data[key] = new_value

    # Write the updated dictionary back to the file
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)