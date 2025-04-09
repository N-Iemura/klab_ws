import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

class BVHParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.joint_hierarchy = {}
        self.offsets = {}
        self.motion_data = []
        self.channels = {}
        self.frame_time = 0
        self.frames = 0
        self.root_joint = None
        self._parse()

    def _parse(self):
        with open(self.filepath, 'r') as f:
            lines = f.readlines()
        
        stack = []
        motion_start = None
        is_motion = False
        current_joint = None

        for i, line in enumerate(lines):
            tokens = line.strip().split()
            if not tokens:
                continue
            
            if tokens[0] == "HIERARCHY":
                continue
            elif tokens[0] == "ROOT" or tokens[0] == "JOINT":
                joint_name = tokens[1]
                self.joint_hierarchy[joint_name] = []
                self.channels[joint_name] = []
                if stack:
                    parent = stack[-1]
                    self.joint_hierarchy[parent].append(joint_name)
                else:
                    self.root_joint = joint_name
                stack.append(joint_name)
                current_joint = joint_name
            elif tokens[0] == "End":
                stack.append("End_" + stack[-1])
            elif tokens[0] == "OFFSET":
                offset = np.array([float(tokens[1]), float(tokens[2]), float(tokens[3])])
                self.offsets[stack[-1]] = offset
            elif tokens[0] == "CHANNELS":
                self.channels[current_joint] = tokens[2:]
            elif tokens[0] == "}":
                stack.pop()
            elif tokens[0] == "MOTION":
                is_motion = True
            elif is_motion and tokens[0] == "Frames:":
                self.frames = int(tokens[1])
            elif is_motion and tokens[0] == "Frame" and tokens[1] == "Time:":
                self.frame_time = float(tokens[2])
            elif is_motion:
                motion_start = i
                break
        
        if motion_start:
            self.motion_data = np.loadtxt(self.filepath, skiprows=motion_start)

    def get_joint_positions(self, frame=0):
        positions = {}
        rotations = {}
        
        data_idx = 0
        root_pos = self.motion_data[frame, :3]
        positions[self.root_joint] = root_pos
        data_idx += 3

        def compute_position(joint, parent_pos, parent_rot):
            if joint not in self.offsets:
                return
            
            offset = self.offsets[joint]
            rotation = np.eye(3)
            
            if joint in self.channels:
                channel_data = self.motion_data[frame, data_idx:data_idx+len(self.channels[joint])]
                data_idx_local = 0
                
                z_angle = x_angle = y_angle = 0
                if "Zrotation" in self.channels[joint]:
                    z_idx = self.channels[joint].index("Zrotation")
                    z_angle = channel_data[z_idx]
                    data_idx_local += 1
                if "Xrotation" in self.channels[joint]:
                    x_idx = self.channels[joint].index("Xrotation")
                    x_angle = channel_data[x_idx]
                    data_idx_local += 1
                if "Yrotation" in self.channels[joint]:
                    y_idx = self.channels[joint].index("Yrotation")
                    y_angle = channel_data[y_idx]
                    data_idx_local += 1
                
                data_idx_local += data_idx
                rotation = R.from_euler('zxy', [z_angle, x_angle, y_angle], degrees=True).as_matrix()
                rotations[joint] = rotation
                
            joint_pos = parent_pos + parent_rot @ offset
            positions[joint] = joint_pos
            
            for child in self.joint_hierarchy.get(joint, []):
                compute_position(child, joint_pos, parent_rot @ rotation)
        
        compute_position(self.root_joint, root_pos, np.eye(3))
        return positions

    def plot_frame(self, frame=0):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        positions = self.get_joint_positions(frame)

        def draw_bone(joint):
            if joint in positions:
                joint_pos = positions[joint]
                for child in self.joint_hierarchy.get(joint, []):
                    if child in positions:
                        child_pos = positions[child]
                        ax.plot([joint_pos[0], child_pos[0]],
                                [joint_pos[1], child_pos[1]],
                                [joint_pos[2], child_pos[2]], 'bo-')
                        draw_bone(child)

        draw_bone(self.root_joint)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"BVH Frame {frame}")
        plt.show()

    def animate(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        def update(frame):
            ax.cla()
            positions = self.get_joint_positions(frame)
            def draw_bone(joint):
                if joint in positions:
                    joint_pos = positions[joint]
                    for child in self.joint_hierarchy.get(joint, []):
                        if child in positions:
                            child_pos = positions[child]
                            ax.plot([joint_pos[0], child_pos[0]],
                                    [joint_pos[1], child_pos[1]],
                                    [joint_pos[2], child_pos[2]], 'bo-')
                            draw_bone(child)
            draw_bone(self.root_joint)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(f"BVH Frame {frame}")

            # 軸の範囲を再設定
            ax.set_xlim([-100, 00])
            ax.set_ylim([-00, 100])
            ax.set_zlim([100, 200])

        ani = FuncAnimation(fig, update, frames=self.frames, interval=self.frame_time * 1000)
        plt.show()

# 使用例
bvh_parser = BVHParser("bvh/01_02.bvh")  # BVHファイルを指定
bvh_parser.animate()  # アニメーションを作成