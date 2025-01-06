import cv2
import mediapipe as mp
import csv
from datetime import datetime

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

# Webカメラのキャプチャ
cap = cv2.VideoCapture(0)

frame_count = 0
try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

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
        cv2.imshow('Leg Landmarks', frame)

        # 'q'キーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    csv_file.close()
    cv2.destroyAllWindows()
