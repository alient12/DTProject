import swift
from spatialmath import SE3
import numpy as np
import roboticstoolbox as rtb
from robot import KinovaGen3_14
from util import *
from typing import Union
from typing import Literal as L

class Simulator:
    def __init__(
            self,
            mode: L['online', 'offline'] = 'offline'
            ):
        
        # simulation costumizations
        self.dt = 0.05
        self.take_snapshot = True
        self.singular_threshold = 1e-2
        self.condition_number_threshold = 30
        
        self._offset_z = 0.11 # due to platform height

        self.mode = mode
        
        # simulation while-loop internal variables (DONT TOUCH)
        self._arrived = False
        self._change_color_flag = False
    
    def setup_env(self):
        self.env = swift.Swift()
        self.env.launch(realtime=True)
        # self.env.launch(realtime=True, no_browser=True)

        # create a Kinova instance
        self.robot = KinovaGen3_14()

        self.set_base_pos(0, 0, self._offset_z)
        self.ready_pos()
        
        self.env.add(self.robot)
        add_platfrom(self.env)
        add_polymtl(self.env)
    
    def set_base_pos(
            self,
            x: float = 0.0,
            y: float = 0.0,
            z: float = 0.0,
            ):
        # assign robot base position
        self.robot.base = SE3(x, y, z)
    
    def set_robot_pos(self, q):
        self.robot.q = q
    
    def ready_pos(self):
        # assign ready pose to current pose
        self.robot.q = self.robot.qr

    def set_target_pos(
            self,
            x: float = 0.5,
            y: float = 0.5,
            z: float = 0.5,
            oa: Union[list, np.ndarray] = [[1, 0, 0],
                                           [0, 0, 1]]
    ):
        z += self._offset_z
        self._Tep = SE3.Trans(x, y, z) * SE3.OA(*oa)

    def start_simulation(self):
        if self.mode == 'offline':
            self.set_target_pos()

            while not self._arrived:

                # resolved-rate motion control
                v, self._arrived = rtb.p_servo(self.robot.fkine(self.robot.q), self._Tep, 1)
                self.robot.qd = np.linalg.pinv(self.robot.jacobe(self.robot.q)) @ v

                # check if robot is in singularity and which links are involved
                in_singularity, singular_links = self.robot.check_singularity(self.singular_threshold, self.condition_number_threshold)


                # back robot color to normal if already changed
                if self._change_color_flag:
                    # make robot non-transparent white
                    change_robot_color(self.robot, (1, 1, 1, 1))

                    # refresh robot 3D model with new colors
                    refresh_robot(self.env, self.robot)
                    
                    self._change_color_flag = False


                # change robot color if singularity is happenning
                if in_singularity:
                    # make robot transparent white
                    change_robot_color(self.robot, (1, 1, 1, 0.5))
                    
                    # red singular links
                    for i in singular_links:
                        change_link_color(self.robot, i, (1, 0, 0, 0.5))
                    
                    # refresh robot 3D model with new colors
                    refresh_robot(self.env, self.robot)
                    
                    if self.take_snapshot:
                        robot_snapshot(self.env, self.robot)
                    
                    self._change_color_flag = True
                

                self.env.step(self.dt)
                # time.sleep(0.1)

            self.env.hold()
        elif self.mode == 'online':
            pass

if __name__ == '__main__':
    sim = Simulator()
    sim.setup_env()
    sim.start_simulation()