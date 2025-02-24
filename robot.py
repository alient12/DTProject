import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

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
    
    def check_singularity(self, singular_threshold=1e-3, condition_number_threshold=30):
        in_singularity = False
        singular_links = []

        J = self.jacobe(self.q)
        
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
        if abs(self.q[elbow_joint_index]) < singular_threshold or abs(self.q[elbow_joint_index] - np.pi) < singular_threshold:
            print("Elbow Singularity detected: Arm fully extended")
            in_singularity = True
            singular_links.extend([3, 4, 5])
        
        return in_singularity, singular_links