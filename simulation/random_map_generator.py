#!/usr/bin/env python
from pysdf import SDF, Link, Model, Visual, Geometry, Pose, Collision
import os
import random

ROBOTS_POSITION = {'x': -4, 'y': 0, 'z': 0}

def read_sdf(destination_sdf_path):
    with open(destination_sdf_path, "r") as f:
        gazebo_map_string = f.read()
    return SDF.from_xml(gazebo_map_string)

def is_permitted_obstacle_position(x, obstacles_x=[]):
    is_in_robot_startup_zone = ROBOTS_POSITION['x'] - 2 < x < ROBOTS_POSITION['x'] + 2
    is_in_obstacle_zone = any([
        obstacle_x - 2 < x and x < obstacle_x + 2 for obstacle_x in obstacles_x
    ])
    return not is_in_robot_startup_zone and not is_in_obstacle_zone

def generate_obstacles():
    obstacles = Model(
        Link(name="link"),
        name="obstacles"
    )

    obstacles_x = list()
    for i in range(3):
        while True:
            sign = random.choice([-1, 1])
            length = random.randint(3, 6)
            y = sign * (5 - 0.5 * length)
            x = random.randrange(-4, 4)
            if is_permitted_obstacle_position(x, obstacles_x): break
        obstacles_x.append(x)

        obstacles.links[0].add(
            Visual(
                Geometry(
                    Geometry.Box(
                        Geometry.Box.Size()
                    )
                ),
                name=f"visual_{i+1}"
            ),
            Collision(
                Geometry(
                    Geometry.Box(
                        Geometry.Box.Size()
                    )
                ),
                name=f"collision_{i+1}"
            )
        )

        obstacles.links[0].visuals[i].geometry.box.size = [0.1, length, 2]
        obstacles.links[0].visuals[i].add(Pose(text = f"{x} {y} 0 0 0 0"))
        obstacles.links[0].colliders[i].geometry.box.size = [0.1, length, 2]
        obstacles.links[0].colliders[i].add(Pose(text = f"{x} {y} 0 0 0 0"))
    return obstacles

def add_static_tag(gazebo_map_string):
    index = gazebo_map_string.find("obstacles\"><link name=\"link\">") + 11
    gazebo_map_string = gazebo_map_string[:index] + "<static>true</static>" + gazebo_map_string[index:]
    return gazebo_map_string

def write_sdf(gazebo_map_string, destination_sdf_path):
    try:
        os.remove(destination_sdf_path)
    except FileNotFoundError:
        pass

    with open(destination_sdf_path, "w") as f:
        f.write(gazebo_map_string)

if __name__ == "__main__":
    template_sdf_path = f"{os.path.dirname(os.path.dirname(os.path.realpath(__file__)))}/ros_ws/src/ros_gz_gazebo/worlds/diff_drive_template.sdf"
    destination_sdf_path = f"{os.path.dirname(os.path.dirname(os.path.realpath(__file__)))}/ros_ws/src/ros_gz_gazebo/worlds/diff_drive.sdf"

    gazebo_map_sdf = read_sdf(template_sdf_path)
    obstacles = generate_obstacles()

    gazebo_map_sdf.worlds[0].add(obstacles)
    gazebo_map_string = gazebo_map_sdf.to_xml(pretty_print=True)

    gazebo_map_string = add_static_tag(gazebo_map_string)

    write_sdf(gazebo_map_string, destination_sdf_path)
