import os
import json

if os.environ.get("ROBOT_ENV") == "SIMULATION":
    ROS_WS = "../ros_ws/src"
else:
    ROS_WS = "/home/nvidia/INF3995-Robot/ros_ws/src"

node_counter = 0

def build_file_tree(path=ROS_WS, new_tree=False):
    """
    Function to build a file tree as a nested dictionary.
    """
    global node_counter

    if new_tree:
        node_counter = 0  

    file_tree = []

    items = os.listdir(path)

    for item in items:
        full_path = os.path.join(path, item)
        node_counter += 1

        node = {
            "name": item,
            "id": node_counter,
            "children": build_file_tree(full_path) if os.path.isdir(full_path) else None
        }
        file_tree.append(node)

    return file_tree

def get_file_tree(path=ROS_WS):
    """
    Function to get a file tree as a string.
    """
    file_tree =  build_file_tree(path, new_tree=True)
    return json.dumps(file_tree, indent=4, ensure_ascii=False)

def get_full_path_recursive(node_id, name, file_tree, path=""):
    """
    Function to get the full path of a file or directory.
    """
    for node in file_tree:
        if node["id"] == node_id and node["name"] == name:
            return os.path.join(path, name)
        elif node["children"]:
            full_path = get_full_path_recursive(node_id, name, node["children"], os.path.join(path, node["name"]))
            if full_path:
                return full_path
    return None

def get_full_path(node_id, name, path=ROS_WS):
    """
    Function to get the full path of a file or directory.
    """
    file_tree = build_file_tree(path, new_tree=True)
    return get_full_path_recursive(node_id, name, file_tree, path)

