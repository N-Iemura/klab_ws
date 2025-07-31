import cv2
import mediapipe as mp
import csv
from datetime import datetime
import pyrealsense2 as rs
import numpy as np
import time  # 追加

# Mediapipeのセットアップ
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# 脚のランドマーク
landmark_names = [
    "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE", "LEFT_HEEL", "LEFT_FOOT_INDEX",
    "RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE", "RIGHT_HEEL", "RIGHT_FOOT_INDEX"
]
landmark_ids = [
    mp_pose.PoseLandmark[landmark] for landmark in landmark_names
]

# CSVファイルの準備
# 現在の日時を取得してファイル名に使用
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f'csv/leg_landmarks_{timestamp}.csv'
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)

# ヘッダーの作成
header = ["Frame"]
for name in landmark_names:
    header.extend([f"{name}_X", f"{name}_Y", f"{name}_Z"])
csv_writer.writerow(header)

# RealSenseのキャプチャ
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

cap = pipeline

frame_count = 0
frame_duration = 1 / 30  # 30FPSの場合、1フレームあたりの時間は約33ms
try:
    while True:
        start_time = time.time()  # フレーム処理開始時刻を記録

        frames = cap.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert RealSense frame to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # フレームをRGBに変換
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Mediapipeで姿勢推定を実行
        results = pose.process(rgb_frame)

        if results.pose_landmarks:
            # フレーム数をカウント
            frame_count += 1

            # ランドマーク座標の記録
            row = [frame_count]
            for landmark_id in landmark_ids:
                landmark = results.pose_landmarks.landmark[landmark_id]
                row.extend([landmark.x, landmark.y, landmark.z])

            # CSVに書き込み
            csv_writer.writerow(row)

            # 描画
            mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # 映像を表示
        cv2.imshow('RealSense', frame)

        # 'q'キーが押されたらループを終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # フレーム処理時間を一定にするために待機
        elapsed_time = time.time() - start_time
        if elapsed_time < frame_duration:
            time.sleep(frame_duration - elapsed_time)

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    cap.stop()
    csv_file.close()
    cv2.destroyAllWindows()