import roboticstoolbox as rtb
from spatialmath import SE3
import swift
import spatialmath as sm
from spatialgeometry import Mesh, Cuboid, Sphere
import numpy as np

import random
import time
import os

import asyncio
import websockets
import json


########################################## KinovaGen3.14 class #########################################

class KinovaGen3_14(rtb.models.URDF.KinovaGen3):
    def __init__(self):
        super().__init__()
        self.name = "gen3.14"
        self.link_colors = [(1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0),
                            (1.0, 1.0, 1.0, 1.0)]
    
    def _to_dict(self, robot_alpha=1.0, collision_alpha=0.0):
        ob = []

        for i, link in enumerate(self.links):
            for gi in link.geometry:
                gi.color = self.link_colors[i]

            if robot_alpha > 0:
                for gi in link.geometry:
                    # gi.set_alpha(robot_alpha)
                    ob.append(gi.to_dict())
            if collision_alpha > 0:
                for gi in link.collision:
                    gi.set_alpha(collision_alpha)
                    ob.append(gi.to_dict())

        # Do the grippers now
        for gripper in self.grippers:
            for link in gripper.links:
                if robot_alpha > 0:
                    for gi in link.geometry:
                        gi.set_alpha(robot_alpha)
                        ob.append(gi.to_dict())
                if collision_alpha > 0:
                    for gi in link.collision:
                        gi.set_alpha(collision_alpha)
                        ob.append(gi.to_dict())
        # for o in ob:
        #     print(o)
        
        return ob

########################################################################################################

######################################## visualization functions #######################################

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
    decoy = KinovaGen3_14()
    change_robot_color(decoy, (1, 0, 0, 0.5))
    decoy.q = robot.q
    decoy.base = robot.base
    decoy.link_colors = robot.link_colors
    env.add(decoy)

########################################################################################################

######################################## mathematical functions ########################################

def check_singularity(robot, singular_threshold=1e-3, condition_number_threshold=30):
    in_singularity = False
    singular_links = []

    J = robot.jacobe(robot.q)
    
    condition_number = np.linalg.cond(J)
    if condition_number > condition_number_threshold:
        print("General singularity detected: High condition number")
        in_singularity = True
        singular_links.extend([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    J_wrist = J[:, 3:6]  # Extract wrist-related part of Jacobian
    if np.linalg.matrix_rank(J_wrist) < 3:
        print("Wrist Singularity detected: Loss of wrist DOF")
        in_singularity = True
        singular_links.extend([5, 6, 7])

    # 2. **Shoulder Singularity (Type I)**: First three columns of Jacobian are linearly dependent
    J_shoulder = J[:, 0:3]  # Extract shoulder-related part of Jacobian
    if np.linalg.matrix_rank(J_shoulder) < 3:
        print("Shoulder Singularity detected: Loss of shoulder DOF")
        in_singularity = True
        singular_links.extend([0, 1, 2])

    # 3. **Elbow Singularity**: Check if the arm is fully extended
    elbow_joint_index = 2  # Adjust based on robot model
    if abs(robot.q[elbow_joint_index]) < singular_threshold or abs(robot.q[elbow_joint_index] - np.pi) < singular_threshold:
        print("Elbow Singularity detected: Arm fully extended")
        in_singularity = True
        singular_links.extend([3, 4, 5])
    
    return in_singularity, singular_links

########################################################################################################

# launch swift environment
env = swift.Swift()
env.launch(realtime=True)
# env.launch(realtime=True, headless=True)

# create a Kinova instance
robot = KinovaGen3_14()

# print robot links and joints
# print(robot)

# assign robot base position
robot.base = SE3(0, 0, 0.11)

# assign ready pose to current pose
robot.q = robot.qr

############################# example ##############################
# Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [1, 0, 0])
# sol = robot.ik_LM(Tep, slimit=100)         # solve IK
# print(sol)

# q_pickup = sol[0]
# print(robot.fkine(q_pickup))

# qt = rtb.jtraj(robot.qr, q_pickup, 50)
# robot.plot(qt.q, backend='pyplot', movie='kinova.gif')
####################################################################


######################################## position calculation ##########################################
# define robot target position
x, y, z = 0.5, 0.5, 0.5+0.11

# parallal with red line
oa = [[0, 1, 0],
      [1, 0, 0]]
# parallal with green line
oa = [[1, 0, 0],
      [0, 1, 0]]
# parallal with blue line
oa = [[1, 0, 0],
      [0, 0, 1]]

# calculate target end-effector position
Tep = SE3.Trans(x, y, z) * SE3.OA(*oa)

# solve IK
# sol = robot.ik_LM(Tep, slimit=100)
# q_target = sol[0]
# print(q_target)
########################################################################################################

platform_filename = os.path.join(os.getcwd(),'assets/stl/platform.stl')
base_transform = SE3([0, 0, 0])

platform = Mesh(
    filename=str(platform_filename),
    scale=(0.01,) * 3,
    pose=base_transform,
    color=[100, 100, 100, 255],
)
env.add(platform)

# cube = Cuboid((1, 1, 1))
# env.add(cube)
# sphere = Sphere(0.5)
# env.add(sphere)


# simulation costumizations
dt = 0.05
take_snapshot = True
singular_threshold = 1e-2
condition_number_threshold = 30

# simulation while-loop internal variables (DONT TOUCH)
arrived = False
env.add(robot)
change_color_flag = False


# while not arrived:

#     # resolved-rate motion control
#     v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, 1)
#     robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v

#     # check if robot is in singularity and which links are involved
#     in_singularity, singular_links = check_singularity(robot, singular_threshold, condition_number_threshold)


#     # back robot color to normal if already changed
#     if change_color_flag:
#         # make robot non-transparent white
#         change_robot_color(robot, (1, 1, 1, 1))

#         # refresh robot 3D model with new colors
#         refresh_robot(env, robot)
        
#         change_color_flag = False


#     # change robot color if singularity is happenning
#     if in_singularity:
#         # make robot transparent white
#         change_robot_color(robot, (1, 1, 1, 0.5))
        
#         # red singular links
#         for i in singular_links:
#             change_link_color(robot, i, (1, 0, 0, 0.5))
        
#         # refresh robot 3D model with new colors
#         refresh_robot(env, robot)
        
#         if take_snapshot:
#             robot_snapshot(env, robot)
        
#         change_color_flag = True
    

#     env.step(dt)
#     # time.sleep(0.1)

# env.hold()


async def listen():
    uri = "ws://localhost:8765"  # Replace with your WebSocket server URI
    async with websockets.connect(uri) as websocket:
        global change_color_flag
        try:
            while True:

                data = await websocket.recv()
                joints = json.loads(data)
                print(f"Received data: {joints}")

                robot.qd = joints

                # check if robot is in singularity and which links are involved
                in_singularity, singular_links = check_singularity(robot, singular_threshold, condition_number_threshold)


                # back robot color to normal if already changed
                if change_color_flag:
                    # make robot non-transparent white
                    change_robot_color(robot, (1, 1, 1, 1))

                    # refresh robot 3D model with new colors
                    refresh_robot(env, robot)
                    
                    change_color_flag = False


                # change robot color if singularity is happenning
                if in_singularity:
                    # make robot transparent white
                    change_robot_color(robot, (1, 1, 1, 0.5))
                    
                    # red singular links
                    for i in singular_links:
                        change_link_color(robot, i, (1, 0, 0, 0.5))
                    
                    # refresh robot 3D model with new colors
                    refresh_robot(env, robot)
                    
                    if take_snapshot:
                        robot_snapshot(env, robot)
                    
                    change_color_flag = True
                

                env.step(dt)
                # time.sleep(0.1)
        except websockets.ConnectionClosed as e:
            print(f"Connection closed with error: {e}")

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(listen())