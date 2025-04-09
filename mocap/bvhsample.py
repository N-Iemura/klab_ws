from bvh import Bvh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# BVH ファイル読み込み
with open('bvh/49_05.bvh', 'r') as f:
    mocap = Bvh(f.read())

# アニメーション描画
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for frame in range(mocap.nframes):
    ax.cla()  # フレームごとにクリア
    positions = []
    for joint in mocap.get_joints_names():
        # 位置データが利用可能なジョイントのみを対象
        channels = mocap.joint_channels(joint)
        if 'Xposition' in channels and 'Yposition' in channels and 'Zposition' in channels:
            pos = mocap.frame_joint_channels(frame, joint, ['Xposition', 'Yposition', 'Zposition'])
            positions.append(pos)
    positions = np.array(positions)
    if len(positions) > 0:
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-')
        for pos in positions:
            ax.plot([pos[0]], [pos[1]], [pos[2]], 'o')
        ax.set_xlim(-100, 100)  # スケールはデータに応じて調整
        ax.set_ylim(-100, 100)
        ax.set_zlim(-100, 100)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.pause(0.033)  # 30fps

plt.show()