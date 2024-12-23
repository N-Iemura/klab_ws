import cv2
import mediapipe as mp

# Mediapipeのセットアップ
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# 姿勢推定の初期化
pose = mp_pose.Pose()

# カメラキャプチャの開始
cap = cv2.VideoCapture(0)  # カメラIDが0のデバイスを使用

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("カメラからのフレームを取得できませんでした")
        break

    # フレームをRGBに変換
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 姿勢推定の実行
    results = pose.process(rgb_frame)

    # キーポイントを描画
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # 結果を表示
    cv2.imshow('Real-Time Pose Estimation', frame)

    # 'q'キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# リソースの解放
cap.release()
cv2.destroyAllWindows()
