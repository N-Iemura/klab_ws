import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np

# CSVファイルの読み込み
csv_file = 'csv/leg_landmarks_20250218_075702.csv'
df = pd.read_csv(csv_file)

# プロットの準備
fig, ax = plt.subplots()

# フレーム数を取得
frames = df['Frame'].unique()

# 使用する脚を選択（'LEFT' または 'RIGHT'）
side = 'RIGHT'

# ランドマーク名を設定
hip = f"{side}_HIP"
knee = f"{side}_KNEE"
ankle = f"{side}_ANKLE"

# 対象のランドマークのみ選択
landmark_coords = {
    lm: {'X': df[f"{lm}_X"], 'Y': df[f"{lm}_Y"]}
    for lm in [hip, knee, ankle]
}

# 角度計算関数
def calculate_angle(x1, y1, x2, y2):
    vertical_vector = np.array([0, -1])  # 真下のベクトル
    segment_vector = np.array([-x2 + x1, -y2 + y1])
    dot_product = np.dot(segment_vector, vertical_vector)
    segment_magnitude = np.linalg.norm(segment_vector)
    vertical_magnitude = np.linalg.norm(vertical_vector)
    angle = np.arccos(dot_product / (segment_magnitude * vertical_magnitude))
    
    # クロスプロダクトを使って回転方向を決定
    cross_product = np.cross(vertical_vector, segment_vector)
    if cross_product > 0:
        angle = -angle
    
    return np.degrees(angle)

# アニメーションの更新関数
def update(frame):
    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(1, 0)
    
    # 現在のフレームのランドマーク座標を取得
    hip_x = landmark_coords[hip]['X'][frame]
    hip_y = landmark_coords[hip]['Y'][frame]
    knee_x = landmark_coords[knee]['X'][frame]
    knee_y = landmark_coords[knee]['Y'][frame]
    ankle_x = landmark_coords[ankle]['X'][frame]
    ankle_y = landmark_coords[ankle]['Y'][frame]
    
    # 角度を計算
    hip_knee_angle = -calculate_angle(hip_x, hip_y, knee_x, knee_y)
    knee_ankle_angle = calculate_angle(knee_x, knee_y, ankle_x, ankle_y)
    
    # ランドマークをプロット
    ax.scatter([hip_x, knee_x, ankle_x], [hip_y, knee_y, ankle_y], c='r')
    
    # ランドマーク名を表示
    ax.text(hip_x, hip_y, f'{side}_HIP')
    ax.text(knee_x, knee_y, f'{side}_KNEE')
    ax.text(ankle_x, ankle_y, f'{side}_ANKLE')
    
    # 角度を表示
    ax.text(0.05, 0.95, f"HIP-KNEE Angle: {hip_knee_angle:.2f}°", transform=ax.transAxes, verticalalignment='top')
    ax.text(0.05, 0.90, f"KNEE-ANKLE Angle: {knee_ankle_angle:.2f}°", transform=ax.transAxes, verticalalignment='top')
    
    # フレーム数を表示
    ax.text(0.05, 0.85, f"Frame: {frame}", transform=ax.transAxes, verticalalignment='top')
    
    # HIPからKNEEへの線を描画
    ax.plot([hip_x, knee_x], [hip_y, knee_y], 'r-')
    # KNEEからANKLEへの線を描画
    ax.plot([knee_x, ankle_x], [knee_y, ankle_y], 'b-')
    
    # 角度の位置にラベルを表示
    hip_knee_angle_x = (hip_x + knee_x) / 2
    hip_knee_angle_y = (hip_y + knee_y) / 2
    ax.text(hip_knee_angle_x, hip_knee_angle_y, f"{hip_knee_angle:.2f}°", fontsize=12, color='red')
    
    knee_ankle_angle_x = (knee_x + ankle_x) / 2
    knee_ankle_angle_y = (knee_y + ankle_y) / 2
    ax.text(knee_ankle_angle_x, knee_ankle_angle_y, f"{knee_ankle_angle:.2f}°", fontsize=12, color='blue')

# アニメーションの設定
ani = FuncAnimation(fig, update, frames=frames, repeat=False)
plt.show()