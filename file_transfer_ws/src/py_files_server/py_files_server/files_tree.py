import os

import os
import json

FOLDER_PATH = './'

def build_json_file_tree():
    """
    Function to build a file tree as a nested dictionary.
    """
    file_tree = {}

    # List all files and folders in the given path
    items = os.listdir(FOLDER_PATH)

    # Iterate through each item
    for item in items:
        # Create the full path by joining the current path and the item
        full_path = os.path.join(FOLDER_PATH, item)

        # Check if the item is a directory
        if os.path.isdir(full_path):
            # Recursively call the function to build the file tree inside this directory
            file_tree[item] = build_json_file_tree(full_path)
        else:
            # It's a file, so mark it as such
            file_tree[item] = None

    json_string = json.dumps(file_tree, indent=4)
    return json_string



