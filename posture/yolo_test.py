import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO  # YOLOv8用のライブラリ

# Load YOLOv8 model
model = YOLO('yolov8s.pt')  # YOLOv8の小型モデルをロード

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert RealSense frame to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform YOLOv8 object detection
        results = model.predict(frame, imgsz=640)  # 画像サイズを指定して推論

        # Render results on the frame
        annotated_frame = results[0].plot()  # YOLOv8の結果を描画

        # Display the frame
        cv2.imshow('YOLOv8 + RealSense', annotated_frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()