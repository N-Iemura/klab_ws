import cv2
import torch
from ultralytics import YOLO
import time

# 学習済みのYOLOv5モデルをロード
model = YOLO("yolov8n.pt")  # YOLOv8の最新モデルを使用

# カメラの初期化
cap = cv2.VideoCapture(0)

# 前フレームの位置と時間
prev_position = None
prev_time = None

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
    current_time = time.time()
    for x1, y1, x2, y2, confidence in detected_balls:
        x_center = (x1 + x2) // 2
        y_center = (y1 + y2) // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"Soccer Ball ({confidence:.2f})", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 速度を計算
        if prev_position is not None and prev_time is not None:
            dx = x_center - prev_position[0]
            dy = y_center - prev_position[1]
            dt = current_time - prev_time
            speed = (dx**2 + dy**2)**0.5 / dt
            cv2.putText(frame, f"Speed: {speed:.2f} pixels/sec", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        prev_position = (x_center, y_center)
        prev_time = current_time

    # フレームを表示
    cv2.imshow("YOLO Soccer Ball Detection", frame)

    # qキーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
