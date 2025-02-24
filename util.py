import random
import os
from spatialmath import SE3
import numpy as np
from spatialgeometry import Mesh, Cuboid

def rand_rgba(alpha=1.0):
    return (random.random(), random.random(), random.random(), alpha)

def change_link_color(robot, i, rgba):
    robot.link_colors[i] = tuple(map(float, rgba))

def change_robot_color(robot, rgba):
    for i, _ in enumerate(robot.link_colors):
        change_link_color(robot, i, rgba)

def change_links_color_rand(robot, alpha=1.0):
    for i, _ in enumerate(robot.link_colors):
        change_link_color(robot, i, rand_rgba(alpha))

def refresh_robot(env, robot):
    env.remove(robot)
    env.add(robot)

def broken_robot_snapshot(env, robot, id):
    del env.swift_objects[id]
    env._send_socket('remove', id)
    env.add(robot)

def robot_snapshot(env, robot):
    decoy = robot.__class__()
    change_robot_color(decoy, (1, 0, 0, 0.5))
    decoy.q = robot.q
    decoy.base = robot.base
    decoy.link_colors = robot.link_colors
    env.add(decoy)

def add_platfrom(env):
    platform_filename = os.path.join(os.getcwd(),'assets/stl/platform.stl')
    base_transform = SE3([0, 0, 0])

    platform = Mesh(
        filename=str(platform_filename),
        scale=(0.01,) * 3,
        pose=base_transform,
        color=[100, 100, 100, 255],
    )
    env.add(platform)

def add_polymtl(env):
    polymtl_filename = os.path.join(os.getcwd(),'assets/stl/polymtl.stl')
    base_transform = SE3([0, -1, 0.5]) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    polymtl = Mesh(
        filename=str(polymtl_filename),
        scale=(0.005,) * 3,
        pose=base_transform,
        color=[1, 1, 1, 255],
    )
    
    cube_scale = 0.0018
    cube_width = 75
    cube_height = 44
    cube_x = 0.64
    cube_z = 0.74

    blue_cube = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z]),
        color=[65, 170, 230, 255],
    )
    
    green_cube = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-2*cube_height*cube_scale]),
        color=[140, 200, 60, 255],
    )
    
    orange_cube = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-4*cube_height*cube_scale]),
        color=[250, 150, 30, 255],
    )

    red_cube = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-6*cube_height*cube_scale]),
        color=[185, 30, 50, 255],
    )
    
    black_cube1 = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-1*cube_height*cube_scale]),
        color=[1, 1, 1, 255],
    )
    
    black_cube2 = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-3*cube_height*cube_scale]),
        color=[1, 1, 1, 255],
    )
    
    black_cube3 = Cuboid(
        scale=(cube_width*cube_scale, 0.025, cube_height*cube_scale),
        pose=SE3([cube_x, -0.985, cube_z-5*cube_height*cube_scale]),
        color=[1, 1, 1, 255],
    )

    env.add(polymtl)
    env.add(blue_cube)
    env.add(black_cube1)
    env.add(green_cube)
    env.add(black_cube2)
    env.add(orange_cube)
    env.add(black_cube3)
    env.add(red_cube)