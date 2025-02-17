import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np

# CSVファイルの読み込み
csv_file = 'csv/leg_landmarks_20241226_194253.csv'
df = pd.read_csv(csv_file)

# プロットの準備
fig, ax = plt.subplots()

# フレーム数を取得
frames = df['Frame'].unique()

# 除外するランドマーク
exclude_landmarks = ["LEFT_HEEL", "LEFT_FOOT_INDEX", "RIGHT_HEEL", "RIGHT_FOOT_INDEX", "RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"]

# 表示するランドマーク順序を指定
landmark_order = [
    "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"
]

# 対象のランドマークのみ選択
landmark_coords = {
    lm: {'X': df[f"{lm}_X"], 'Y': df[f"{lm}_Y"]}
    for lm in landmark_order
    if lm not in exclude_landmarks
}

# 角度計算関数
def calculate_angle(x1, y1, x2, y2):
    vertical_vector = np.array([0, -1])  # 真下のベクトル
    segment_vector = np.array([x2 - x1, y2 - y1])
    dot_product = np.dot(segment_vector, vertical_vector)
    segment_magnitude = np.linalg.norm(segment_vector)
    vertical_magnitude = np.linalg.norm(vertical_vector)
    angle = np.arccos(dot_product / (segment_magnitude * vertical_magnitude))
    
    # Cross product to determine the sign
    cross_product = np.cross(vertical_vector, segment_vector)
    if cross_product > 0:
        angle = -angle
    
    return np.degrees(angle)

# アニメーション更新関数
def update(frame):
    ax.clear()
    ax.set_title(f"Frame {frame}")
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    for lm in landmark_order:
        if lm not in exclude_landmarks:
            ax.plot(landmark_coords[lm]['X'][frame], landmark_coords[lm]['Y'][frame], 'o', label=lm)
            ax.text(landmark_coords[lm]['X'][frame], landmark_coords[lm]['Y'][frame], lm, fontsize=12, ha='right')
    
    # 左のHIP, KNEE, ANKLEの座標を取得
    left_hip_x = landmark_coords['LEFT_HIP']['X'][frame]
    left_hip_y = landmark_coords['LEFT_HIP']['Y'][frame]
    left_knee_x = landmark_coords['LEFT_KNEE']['X'][frame]
    left_knee_y = landmark_coords['LEFT_KNEE']['Y'][frame]
    left_ankle_x = landmark_coords['LEFT_ANKLE']['X'][frame]
    left_ankle_y = landmark_coords['LEFT_ANKLE']['Y'][frame]
    
    # 角度を計算
    hip_knee_angle = calculate_angle(left_hip_x, left_hip_y, left_knee_x, left_knee_y)
    knee_ankle_angle = calculate_angle(left_knee_x, left_knee_y, left_ankle_x, left_ankle_y)
    
    # 角度をプロットに表示
    ax.text(0.05, 0.95, f"HIP-KNEE Angle: {hip_knee_angle:.2f}°", transform=ax.transAxes, verticalalignment='top')
    ax.text(0.05, 0.90, f"KNEE-ANKLE Angle: {knee_ankle_angle:.2f}°", transform=ax.transAxes, verticalalignment='top')
    
    # HIPからKNEEへの線を描画
    ax.plot([left_hip_x, left_knee_x], [left_hip_y, left_knee_y], 'r-')
    # KNEEからANKLEへの線を描画
    ax.plot([left_knee_x, left_ankle_x], [left_knee_y, left_ankle_y], 'b-')
    
    # 角度の位置にラベルを表示
    hip_knee_angle_x = (left_hip_x + left_knee_x) / 2
    hip_knee_angle_y = (left_hip_y + left_knee_y) / 2
    ax.text(hip_knee_angle_x, hip_knee_angle_y, f"{hip_knee_angle:.2f}°", fontsize=12, color='red')
    
    knee_ankle_angle_x = (left_knee_x + left_ankle_x) / 2
    knee_ankle_angle_y = (left_knee_y + left_ankle_y) / 2
    ax.text(knee_ankle_angle_x, knee_ankle_angle_y, f"{knee_ankle_angle:.2f}°", fontsize=12, color='blue')

# アニメーションの設定
ani = FuncAnimation(fig, update, frames=frames, repeat=False)
plt.show()