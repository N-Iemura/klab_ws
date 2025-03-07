import cv2
import torch
from ultralytics import YOLO
import time
import csv

# 学習済みのYOLOv5モデルをロード
model = YOLO("yolo11n.pt")  # YOLOv8の最新モデルを使用

# カメラの初期化
cap = cv2.VideoCapture(0)

# 前フレームの位置と時間
prev_position = None
prev_time = None
start_time = time.time()

# CSVファイルの初期化
with open('ball_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Speed', 'Position_X', 'Position_Y'])

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # フレームをYOLOモデルで推論
        results = model(frame)

        # サッカーボールを検出
        detected_balls = []
        result_index = 0
        while result_index < len(results):
            result = results[result_index]
            box_index = 0
            while box_index < len(result.boxes.data):  # 検出された物体のボックス
                box = result.boxes.data[box_index]
                x1, y1, x2, y2, confidence, cls = box.tolist()
                if int(cls) == 32:  # クラスID 32はCOCOデータセットの"Sports Ball"
                    detected_balls.append((int(x1), int(y1), int(x2), int(y2), confidence))
                box_index += 1
            result_index += 1

        # サッカーボールが検出された場合、位置と速度を表示
        if detected_balls:
            x1, y1, x2, y2, confidence = detected_balls[0]
            current_position = ((x1 + x2) / 2, (y1 + y2) / 2)
            current_time = time.time() - start_time

            if prev_position is not None and prev_time is not None:
                distance = ((current_position[0] - prev_position[0]) ** 2 + (current_position[1] - prev_position[1]) ** 2) ** 0.5
                time_elapsed = current_time - prev_time
                speed = distance / time_elapsed

                # CSVファイルに速度と位置データを書き込む
                writer.writerow([current_time, speed, current_position[0], current_position[1]])

                # 速度と位置をフレームに描画
                cv2.putText(frame, f"Speed: {speed:.2f} px/s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Position: {current_position}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # 前の位置から現在の位置までの線を描画
                cv2.line(frame, (int(prev_position[0]), int(prev_position[1])), (int(current_position[0]), int(current_position[1])), (255, 0, 0), 2)

            # サッカーボールのバウンディングボックスを描画
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            prev_position = current_position
            prev_time = current_time

        # フレームを表示
        # cv2.imshow("YOLO Soccer Ball Detection", frame)

        # qキーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()