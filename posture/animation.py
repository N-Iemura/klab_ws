import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd

# CSVファイルの読み込み
csv_file = 'csv/leg_landmarks_by_frame.csv'
df = pd.read_csv(csv_file)

# プロットの準備
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# フレーム数を取得  
frames = df['Frame'].unique()

# 除外するランドマーク
exclude_landmarks = ["LEFT_HEEL", "LEFT_FOOT_INDEX", "RIGHT_HEEL", "RIGHT_FOOT_INDEX"]

# 表示するランドマーク順序を指定
landmark_order = [
    "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE",
    "RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"
]

# 対象のランドマークのみ選択
landmark_coords = {
    lm: {'X': df[f"{lm}_X"], 'Y': df[f"{lm}_Y"], 'Z': df[f"{lm}_Z"]}
    for lm in landmark_order
    if lm not in exclude_landmarks
}

# アニメーション更新関数
def update(frame):
    ax.clear()
    ax.set_title(f"Frame {frame}")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_zlim(0, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Plot LEFT side
    left_x = [landmark_coords['LEFT_HIP']['X'][frame], landmark_coords['LEFT_KNEE']['X'][frame], landmark_coords['LEFT_ANKLE']['X'][frame]]
    left_y = [landmark_coords['LEFT_HIP']['Y'][frame], landmark_coords['LEFT_KNEE']['Y'][frame], landmark_coords['LEFT_ANKLE']['Y'][frame]]
    left_z = [landmark_coords['LEFT_HIP']['Z'][frame], landmark_coords['LEFT_KNEE']['Z'][frame], landmark_coords['LEFT_ANKLE']['Z'][frame]]
    ax.plot(left_x, left_y, left_z, color='blue')
    
    # Plot RIGHT side
    right_x = [landmark_coords['RIGHT_HIP']['X'][frame], landmark_coords['RIGHT_KNEE']['X'][frame], landmark_coords['RIGHT_ANKLE']['X'][frame]]
    right_y = [landmark_coords['RIGHT_HIP']['Y'][frame], landmark_coords['RIGHT_KNEE']['Y'][frame], landmark_coords['RIGHT_ANKLE']['Y'][frame]]
    right_z = [landmark_coords['RIGHT_HIP']['Z'][frame], landmark_coords['RIGHT_KNEE']['Z'][frame], landmark_coords['RIGHT_ANKLE']['Z'][frame]]
    ax.plot(right_x, right_y, right_z, color='red')

    scatter_points = []  # 凡例順序を保つためのポイントリスト
    for landmark in landmark_order:
        if landmark in landmark_coords:
            coords = landmark_coords[landmark]
            x = coords['X'][frame]
            y = coords['Y'][frame]
            z = coords['Z'][frame]
            scatter = ax.scatter(x, y, z, label=landmark)
            scatter_points.append(scatter)

    # 凡例を指定した順序で表示
    ax.legend(handles=scatter_points, loc='upper left')

# アニメーション作成
ani = FuncAnimation(fig, update, frames=len(frames), interval=50)

# アニメーションの表示
plt.show()
